/*
  simulator.c - functions to simulate how the buffer is emptied and the
                stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012-2014 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

  2020 - modified for grblHAL by Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "simulator.h"
#include "eeprom.h"
#include "mcu.h"
#include "driver.h"
#include "sim_view.h"

#include "grbl/grbl.h"

void sim_nop (void)
{
}

sim_vars_t sim = {
    .on_init = sim_nop,
    .on_tick = sim_nop,
    .on_byte = sim_nop,
    .on_shutdown = sim_nop
};

// Setup 
void init_simulator (void)
{
    sim.baud_ticks = F_CPU / 115200;

    sim.on_init();
}

// Shutdown simulator - call exit hooks, save eeprom is taken care of in atexit handler
void shutdown_simulator (void)
{
    sim.on_shutdown();
}

void simulate_hardware (bool do_serial)
{
    //do one tick
    sim.masterclock++;
    sim.sim_time = (float)sim.masterclock / (float)F_CPU;

    mcu_master_clock();

    if (do_serial)
        simulate_serial();
}

// Runs the hardware simulator at the desired rate until sim.exit is set
void sim_loop (void)
{
    // a sensible control frame length is yet to be found.
    // the currrent 100ms are a blind guess, assuming some small multiple
    // of the OS's thread scheduler's time splice makes sense.
    // smaller means less jitter in the resulting sim time.
    // too small makes it worse again because of the limited resultion of plattform_sleep(..)
    // and a simple P controller may be not enough any longer
    const uint32_t control_frame_ns = 100 * 1000 * 1000; // in real time

    int32_t sleep_time_us = 0; // sleep time per control frame
    uint32_t ticks_per_frame = F_CPU / 100; // start simulating a few ticks before entering the control loop
    uint64_t target_ticks = ticks_per_frame;
    uint32_t ns_prev = platform_ns();
    uint64_t next_byte_tick = F_CPU;   //wait 1 sec (sim time) before reading IO.

    while (sim.exit != exit_OK  ) { //don't quit until idle
        while (sim.masterclock < target_ticks) {
            // only read serial port as fast as the baud rate allows
            bool read_serial = (sim.masterclock >= next_byte_tick);

            // do low level hardware
            simulate_hardware(read_serial);

            // do app-specific per-tick processing
            sim.on_tick();

            if (read_serial) {
                // baud rate is for symbols and UART has 1 bit per symbol.
                // with a typical 8N1 serial, we need 10 symbols per byte
                next_byte_tick += sim.baud_ticks * 10;
                // do app-specific per-byte processing
                sim.on_byte();
            }

            // prevent overlong catchup with target ticks and waiting at the end
            if (sim.exit == exit_OK)
                return;
        }

        // calculate current speedup ...
        uint32_t ns_now = platform_ns();
        uint32_t ns_elapsed = (ns_now - ns_prev);
        ns_prev = ns_now;
        uint32_t rt_ticks_per_frame = F_CPU / 1e9 * ns_elapsed;
        sim.speedup = (double)ticks_per_frame / rt_ticks_per_frame;

        // ... and how many ticks to simulate next
        if (args.speedup) {
            // aim for the target speed up, i.e. approximate (scaled) real time
            float speedup_error = args.speedup - sim.speedup;
            // a negative time-error means the host was too fast
            // a positive time error means the either the host was too slow or we waited too long
            int32_t time_error_us = speedup_error * ns_elapsed / 1000;
            // for the actual error in the sleep time, we need to remove the previous wait time
            int32_t sleep_error_us = time_error_us - sleep_time_us;
            sleep_time_us = -1 * sleep_error_us; // looks like a simple P-controler is just fine
            if (sleep_time_us > 0) {
                // we've been too fast (which is good), so let's wait a bit...
                platform_sleep(sleep_time_us);
                // ... and schedule as many ticks as the desired speedup requires for the next frame
                ticks_per_frame = F_CPU / 1e9 * control_frame_ns * args.speedup;
            }
            else {
                // the host has been too slow to fullfill the desired realtime speedup.
                // so we do not wait and schedule only as many ticks for the next frame as we're
                // able to execute in the control fame's time to prevent it from getting longer and longer.
                sleep_time_us = 0;
                ticks_per_frame *= (double)control_frame_ns / ns_elapsed;
            }
        }
        else {
            // aim to maximize the simulated ticks thoughput by filling the control frame
            // completely, based on our current knowledge of the time it takes to simulate ticks.
            ticks_per_frame *= (double)control_frame_ns / ns_elapsed;
        }

        target_ticks += ticks_per_frame;
    }
}

// Print serial output to args.serial_out_file
void sim_serial_out (uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;
    static bool continuation = 0;

    buf[len++] = data;
    // print when we get to newline or run out of buffer
    if(data == '\n' || data == '\r' || len >= 127) {
        if (args.comment_char && !continuation)
            fprintf(args.serial_out_file, "%c ", args.comment_char);
        buf[len] = '\0';
        fprintf(args.serial_out_file, "%s", buf);
        // don't print comment on next line if we are just printing to avoid buffer overflow
        continuation = (len >= 128); 
        len = 0;
    }
}

// Print serial output to sim.socket_fd stream.
// Buffer bytes and emit each transmit *burst* as a single socket write, flushed by sim_socket_flush()
// once the UART TX line goes idle (see simulate_serial). The original code flushed only on a newline,
// which stalled binary protocol bursts that carry none - e.g. the YModem ACK/'C' handshake - so the
// host hung waiting for a reply that never went out. Flushing per burst (rather than per byte) keeps
// multi-byte replies like ACK+'C' together in one segment, which is what real serial hardware delivers
// and what the host's byte-at-a-time receive path expects.
static uint8_t sock_buf[1024];
static unsigned sock_len = 0;

void sim_socket_flush (void)
{
    if(sock_len == 0)
        return;
#ifdef WIN32
    if(sim.socket_fd != INVALID_SOCKET) {
        if(send(sim.socket_fd, (const char *)sock_buf, sock_len, 0) == SOCKET_ERROR)
            exit(-10);
    }
#else
    if(sim.socket_fd) {
        if(write(sim.socket_fd, sock_buf, sock_len) < 0)
            exit(-10);
    }
#endif
    sock_len = 0;
}

// --- console action log -------------------------------------------------------------------------------
// Echo socket traffic to stderr as a readable, one-line-per-command log, so the simulator's own console
// window shows what it is doing when launched (minimized) by ioSender. Only printable ASCII is kept and
// the 4 Hz status-report polling (the '?' request and "<...>" responses) is dropped, so the log shows real
// activity rather than noise.
static void sim_log_line (const char *dir, uint8_t *buf, unsigned *len, uint8_t c)
{
    if(c == '\r' || c == '\n') {
        if(*len) {
            buf[*len] = '\0';
            if(buf[0] != '<') {                 // drop status reports
                fprintf(stderr, "%s %s\n", dir, buf);
                char line[280];
                snprintf(line, sizeof(line), "%s %s", dir, buf);
                sim_view_log_append(line);      // feed the Show Log window
            }
            if(dir[0] == '>' && strncmp((char *)buf, "[MSG:", 5) == 0)   // surface [MSG:] in the 3D view
                sim_view_set_message((char *)buf);
            *len = 0;
        }
    } else if(c >= 0x20 && c < 0x7f && *len < 250)
        buf[(*len)++] = c;
}

void sim_log_rx (uint8_t c)
{
    static uint8_t buf[256];
    static unsigned len = 0;

    if(c == '?')                                // drop status-report requests
        return;

    sim_log_line("<<", buf, &len, c);
}

void sim_log_tx (uint8_t c)
{
    static uint8_t buf[256];
    static unsigned len = 0;

    sim_log_line(">>", buf, &len, c);
}

void sim_socket_out (uint8_t data)
{
    sim_log_tx(data);

    sock_buf[sock_len++] = data;
    if(sock_len >= sizeof(sock_buf))    // safety: flush a burst larger than the buffer (e.g. a file dump)
        sim_socket_flush();
}

