/*
  driver.c - driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2020-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "platform.h"

#include "grbl/hal.h"
#include "grbl/state_machine.h"

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#ifndef SQUARING_ENABLED
#define SQUARING_ENABLED 0
#endif

static spindle_id_t spindle_id;
static bool probe_invert;
static uint32_t ticks = 0;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static on_execute_realtime_ptr on_execute_realtime;

// --- simulated machine model ----------------------------------------------------------------------
// grbl resets sys.position to 0 at the start of each homing approach, so an independent absolute
// step counter is needed to trip the simulated limit and probe switches. It is advanced one step per
// axis per stepper pulse (see stepperPulseStart) and read by sim_update_inputs().
static int32_t sim_axis_pos[N_AXIS] = {0};

// Simulated probe: the probe input trips when the Z axis has descended PROBE_TRIP_MM (a positive
// magnitude) below where probing started. Matches a touch-off that triggers after a fixed approach;
// set to your sensor's trip distance (e.g. 4.0). Probe start is captured the first tick a probe move
// is active (see sim_update_inputs).
#define PROBE_AXIS   Z_AXIS
#define PROBE_TRIP_MM 4.0f
static int32_t sim_probe_start;
static bool sim_probe_armed = false;

// To keep $H fast in the sim (it steps at a fraction of real time during the homing loop, so driving
// the full max-travel to the switch is slow), the simulated carriage is parked this far from each home
// switch when a homing move starts - so the seek only has to cover ~this distance, not the whole travel.
#define HOMING_SEEK_OFFSET_MM 6.0f
static void sim_park_for_homing (void);

void SysTick_Handler (void);
void Stepper_IRQHandler (void);
void Limits0_IRQHandler (void);
void Control_IRQHandler (void);

#if SQUARING_ENABLED
static axes_signals_t motors_0 = {AXES_BITMASK}, motors_1 = {AXES_BITMASK};
void Limits1_IRQHandler (void);
#endif

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        systick_timer.enable = 1;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

#if SQUARING_ENABLED

inline static void set_step_outputs (axes_signals_t step_out_0)
{
    axes_signals_t step_out_1;

    step_out_1.bits = (step_out_0.bits & motors_1.bits) ^ settings.steppers.step_invert.bits;
    step_out_0.bits = (step_out_0.bits & motors_0.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out_0.bits, AXES_BITMASK);
    mcu_gpio_set(&gpio[STEP_PORT1], step_out_1.bits, AXES_BITMASK);
}

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        ganged.x = On;
    } else {
        ganged.x = On;
    }

    return ganged;
}

#else

inline static void set_step_outputs (axes_signals_t step_out)
{
    step_out.bits = (step_out.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out.bits, AXES_BITMASK);
}

#endif

inline static void set_dir_outputs (axes_signals_t dir_out)
{
    mcu_gpio_set(&gpio[DIR_PORT], dir_out.value ^ settings.steppers.dir_invert.mask, AXES_BITMASK);
}

static void stepperEnable (axes_signals_t enable, bool hold)
{
    mcu_gpio_set(&gpio[STEPPER_ENABLE_PORT], enable.value ^ settings.steppers.enable_invert.mask, AXES_BITMASK);
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    static bool homing_announced = false;
    if(state_get() == STATE_HOMING) {
        if(!homing_announced) {     // one feedback line at the start of a homing cycle - the sim runs
            homing_announced = true; // slowly + silently during $H, so give the sender something to show
            hal.stream.write("[MSG:Homing]" ASCII_EOL);
        }
        sim_park_for_homing();
    } else
        homing_announced = false;

    timer[STEPPER_TIMER].load = 5000;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;

//    hal.stepper_interrupt_callback();   // start the show
}

// Called at the start of each homing move: park any axis that is more than HOMING_SEEK_OFFSET_MM from
// its home switch right up close to the switch, so the seek is short and $H finishes quickly. Only
// moves axes that are far away, so it shortcuts the long search pass but leaves the pull-off / locate
// passes (which start near the switch) untouched. This only shifts WHEN the switch trips - grbl tracks
// its own position independently, so homing accuracy is unaffected.
static void sim_park_for_homing (void)
{
    uint_fast8_t i = 0;
    do {
        int32_t travel = (int32_t)(-settings.axis[i].max_travel * settings.axis[i].steps_per_mm);
        if(travel > 0) {
            int32_t offset = (int32_t)(HOMING_SEEK_OFFSET_MM * settings.axis[i].steps_per_mm);
            if(offset >= travel)
                offset = travel / 2;            // tiny travel: park half way
            int32_t park = travel - offset;     // distance from origin to the parked spot
            if(bit_istrue(settings.homing.dir_mask.value, bit(i))) {     // homes toward negative
                if(sim_axis_pos[i] > -park)
                    sim_axis_pos[i] = -park;
            } else {                                                     // homes toward positive
                if(sim_axis_pos[i] < park)
                    sim_axis_pos[i] = park;
            }
        }
    } while(++i < N_AXIS);
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].load = 0;
    timer[STEPPER_TIMER].enable = 0;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    timer[STEPPER_TIMER].load = cycles_per_tick;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        // Advance the absolute machine model one step per stepping axis (logical direction: a set
        // dir_out bit is a negative move) so the simulated limit / probe switches can trip.
        uint_fast8_t i = 0;
        do {
            if(stepper->step_out.bits & bit(i))
                sim_axis_pos[i] += (stepper->dir_out.bits & bit(i)) ? -1 : 1;
        } while(++i < N_AXIS);
        set_step_outputs(stepper->step_out);
        sim_update_inputs();    // re-evaluate limit / probe switches now the position changed
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
//        next_step_out = stepper->step_out; // Store out_bits
//        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.value = gpio[LIMITS_PORT0].state.value;

    if (settings.limits.invert.mask)
        signals.min.mask ^= settings.limits.invert.mask;

    return signals;
}

#if SQUARING_ENABLED

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_0.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0);
    motors_1.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
static limit_signals_t limitsGetHomeState()
{
    limit_signals_t signals = {0};

    if(motors_0.mask) {

        signals.min.mask = gpio[LIMITS_PORT0].state.value;

        if (settings.limits.invert.mask)
            signals.min.mask ^= settings.limits.invert.mask;
    }

    if(motors_1.mask) {

       signals.max.mask = gpio[LIMITS_PORT1].state.value;

        if (settings.limits.invert.mask)
            signals.max.mask ^= settings.limits.invert.mask;
    }

    return signals;
}

#endif

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    // During a homing cycle (homing_cycle.mask != 0) the core asks us to suppress the hard-limit
    // pin-change interrupt for the cycle's duration - homing detects the switches by polling get_state().
    // Leaving the IRQ live makes a homing trip fire limit_interrupt_handler (mc_reset + hard-limit alarm)
    // and aborts the cycle, which surfaces as Alarm 8 (pull-off fail). The core re-enables it afterwards
    // via enable(hard_enabled, {0}) from mc_homing_cycle (motion_control.c).
    bool irq_on = on && homing_cycle.mask == 0;

    gpio[LIMITS_PORT0].irq_mask.mask = irq_on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT0].irq_state.mask = 0;

  #if SQUARING_ENABLED
    gpio[LIMITS_PORT1].irq_mask.mask = irq_on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT1].irq_state.mask = 0;

    hal.limits.get_state = homing_cycle.mask != 0 ? limitsGetHomeState : limitsGetState;
  #endif
}

// Drive the simulated limit and probe inputs from the tracked machine position. Called per step (from
// stepperPulseStart, the only time positions change) so it adds no per-tick overhead. grbl's default
// limitsGetState() reads only LIMITS_PORT0, so every axis limit is asserted there - the switch for an
// axis sits at its travel extreme on the homing side.
void sim_update_inputs (void)
{
    uint16_t trip = 0;
    uint_fast8_t i = 0;
    do {
        // travel limit in steps; settings.axis[].max_travel is stored as a negative magnitude.
        int32_t travel = (int32_t)(-settings.axis[i].max_travel * settings.axis[i].steps_per_mm);
        if(travel > 0) {
            if(bit_istrue(settings.homing.dir_mask.value, bit(i))) {     // axis homes toward negative
                if(sim_axis_pos[i] <= -travel)
                    trip |= bit(i);
            } else {                                                     // axis homes toward positive
                if(sim_axis_pos[i] >= travel)
                    trip |= bit(i);
            }
        }
    } while(++i < N_AXIS);

    // Drive the ELECTRICAL pin level pre-inverted by $5 (limit invert mask): limitsGetState() re-applies
    // the same invert, so the firmware reads the true logical "at switch" state for any switch type. Without
    // this an inverted ($5) axis reads asserted off the switch and homing's pull-off check fails (Alarm 8).
    mcu_gpio_in(&gpio[LIMITS_PORT0], trip ^ settings.limits.invert.mask, AXES_BITMASK);

    // Probe: once armed (by probeConfigureInvertMask at the start of a probe cycle) trip after the
    // probe axis has travelled PROBE_TRIP_MM from where the cycle started, in either direction.
    if(sim_probe_armed) {
        int32_t moved = sim_axis_pos[PROBE_AXIS] - sim_probe_start;
        if(moved < 0)
            moved = -moved;
        bool tripped = moved >= (int32_t)(PROBE_TRIP_MM * settings.axis[PROBE_AXIS].steps_per_mm);
        mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT | (tripped ? PROBE_BIT : 0), PROBE_MASK);
    }
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.mask = gpio[CONTROL_PORT].state.value;
	signals.limits_override = settings.control_invert.limits_override;

    if(settings.control_invert.mask)
        signals.mask ^= settings.control_invert.mask;

    return signals;
}

static void probeConfigureInvertMask (bool is_probe_away, bool probing)
{
  probe_invert = settings.probe.invert_probe_pin;

  if (is_probe_away)
      probe_invert ^= is_probe_away;

  // Arm the simulated probe at the start of a probe cycle (capture where the probe axis starts);
  // release the trigger when the cycle ends.
  if (probing) {
      if (!sim_probe_armed) {
          sim_probe_start = sim_axis_pos[PROBE_AXIS];
          sim_probe_armed = true;
      }
  } else {
      sim_probe_armed = false;
      mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_MASK);
  }
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);

    state.triggered ^= probe_invert;

    return state;
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return 0; //spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    state.value = gpio[SPINDLE_PORT].state.value ^ settings.pwm_spindle.invert.mask;

    return state;
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

	static spindle_pwm_t spindle_pwm;

	spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, 1000000);

//    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void coolantSetState (coolant_state_t mode)
{
    mcu_gpio_set(&gpio[COOLANT_PORT], mode.value ^ settings.coolant.invert.mask, COOLANT_MASK);
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.value = gpio[COOLANT_PORT].state.value ^ settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    *ptr |= bits;
//    __enable_interrupts();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
//    __enable_interrupts();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr = value;
//    __enable_interrupts();
    return prev;
}

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }

#if SQUARING_ENABLED
    hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif
}

bool driver_setup (settings_t *settings)
{
    timer[STEPPER_TIMER].prescaler = 0;
    timer[STEPPER_TIMER].irq_enable = 1;
    mcu_register_irq_handler(Stepper_IRQHandler, Timer0_IRQ);

    gpio[STEPPER_ENABLE_PORT].dir.mask = AXES_BITMASK;
    gpio[STEP_PORT0].dir.mask = AXES_BITMASK;
    gpio[DIR_PORT].dir.mask = AXES_BITMASK;

    gpio[COOLANT_PORT].dir.mask = COOLANT_MASK;
    gpio[SPINDLE_PORT].dir.mask = SPINDLE_MASK;

    gpio[LIMITS_PORT0].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT0].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits0_IRQHandler, LIMITS_IRQ0);

#if SQUARING_ENABLED
    gpio[STEP_PORT1].dir.mask = AXES_BITMASK;

    gpio[LIMITS_PORT1].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT1].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits1_IRQHandler, LIMITS_IRQ1);
#endif

    gpio[CONTROL_PORT].dir.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].rising.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].irq_mask.mask = CONTROL_MASK;
    mcu_register_irq_handler(Control_IRQHandler, CONTROL_IRQ);

    mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_CONNECTED_BIT); // default to connected

    settings_changed_flags_t changed_flags = {0};
    hal.settings_changed(settings, changed_flags);
    hal.stepper.go_idle(true);
    spindle_ptrs_t* spindle;

    if((spindle = spindle_get(0))) {
        spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
    }

    hal.coolant.set_state((coolant_state_t){0});

    return settings->version.id == 23;
}

// used to inject a sleep in grbl main loop,
// ensures hardware simulator gets some cycles in "parallel"
void sim_process_realtime (uint_fast16_t state)
{
    // One-shot, once settings are loaded: place the simulated carriage in the middle of the X/Y work
    // area and 10 mm below Z home instead of booting on the home corner, so a subsequent $H is a clearly
    // visible move in the sender's 3D view. sys.position is what grbl reports (drives the displayed
    // tool); sim_axis_pos is the sim's own switch-tripping counter - set both so they stay in step.
    // max_travel is stored negative (work envelope is [max_travel, 0]), so max_travel/2 is the centre.
    static bool start_pos_set = false;
    if(!start_pos_set && settings.axis[X_AXIS].steps_per_mm > 0.0f) {
        start_pos_set = true;
        uint_fast8_t i = 0;
        do {
            float pos_mm = i == Z_AXIS ? -10.0f : settings.axis[i].max_travel / 2.0f;
            int32_t steps = (int32_t)(pos_mm * settings.axis[i].steps_per_mm);
            sys.position[i] = steps;
            sim_axis_pos[i] = steps;
        } while(++i < N_AXIS);
    }

    //platform_sleep(0); // yield needed? or simply trust the OS's thread scheduler...
    on_execute_realtime(state);
}

uint32_t millis (void)
{
    return ticks;
}

bool driver_init ()
{
    mcu_reset();

    mcu_register_irq_handler(SysTick_Handler, Systick_IRQ);

    systick_timer.load = F_CPU / 1000 - 1;
    systick_timer.irq_enable = 1;
    systick_timer.enable = 1;

    hal.info = "Simulator";
    hal.driver_version = "260324";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = F_CPU;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = sim_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#if SQUARING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigureInvertMask;

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindle_set_speed,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "simulated PWM spindle");

    hal.control.get_state = systemGetState;
/*
    hal.show_message = showMessage;
*/

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));
    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

    hal.driver_cap.amass_level = 3;
    hal.coolant_cap.flood = On;
    hal.coolant_cap.mist = On;
    // hal.driver_cap.software_debounce = On;
    // This is required for the hal to initialize properly!
    hal.driver_cap.step_pulse_delay = On;

    hal.signals_cap.safety_door_ajar = On;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

    // Filesystem plugins ($F/$FI/$F<=, YModem, O<name> CALL macros, ATC tool change).
    // fs_stream_init() also runs fs_macros_init(), which registers the vfs on_mount hook
    // (atc_macros_attach); it must run BEFORE the littlefs mount below so that hook fires on the
    // initial mount and ATC is detected at boot when tc.macro is present. (Called directly rather
    // than via plugins_init.h, which also references the weak my_plugin_init that the archive linker
    // would not pull, and a host of plugins not built for the simulator.)
#if SDCARD_ENABLE || LITTLEFS_ENABLE == 2
    extern void fs_stream_init (void);
    fs_stream_init();
#endif

#if LITTLEFS_ENABLE
#ifndef LITTLEFS_MOUNT_DIR
#define LITTLEFS_MOUNT_DIR (LITTLEFS_ENABLE == 2 ? "/" : "/littlefs")
#endif
    fs_littlefs_mount(LITTLEFS_MOUNT_DIR, sim_littlefs_hal());
#endif

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 10;
}

// Main stepper driver
void Stepper_IRQHandler (void)
{
    hal.stepper.interrupt_callback();
}

void Control_IRQHandler (void)
{
    gpio[CONTROL_PORT].irq_state.value = ~CONTROL_MASK;
    hal.control.interrupt_callback(hal.control.get_state());
}

void Limits0_IRQHandler (void)
{
    gpio[LIMITS_PORT0].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#if SQUARING_ENABLED

void Limits1_IRQHandler (void)
{
    gpio[LIMITS_PORT1].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    ticks++;

    if(delay.ms && --delay.ms == 0) {
//        systick_timer.enable = 0;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
