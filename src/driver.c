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
#include <stdio.h>

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "sim_view.h"
#include "sim_setup.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "platform.h"

#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/ngc_params.h"

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

// Simulated probe targets - real surfaces the probe trips against, so probed coordinates are physically
// meaningful (a 3D corner find yields an actual corner, a toolsetter yields a repeatable reference).
// Three solids in machine mm, positioned from the controller's own G28 / G59.3 offsets - read when each
// probe cycle arms, so they track the operator's setup the way probe_tfl / tc.macro expect:
//   - Spoilboard: the table plane near the bottom of Z travel (probed at G28, clear of the stock).
//   - Stock block: sits on the spoilboard, its TFL corner STOCK_INSET inside G28 in X and Y. probe_tfl
//     finds its top (Z down), left face (X) and front face (Y).
//   - Toolsetter puck: centred on G59.3, top PUCK_DROP below the G59.3 Z origin. M6 T8 probes its top.
// The probe trips whenever the controlled point enters any of these solids during a probe cycle.
#define STOCK_INSET   15.0f   // stock TFL corner this far inside G28 in X and Y (macro allows 10-30 mm)
#define STOCK_SIZE    60.0f   // stock footprint in X and Y
#define STOCK_THICK   19.0f   // stock thickness above the spoilboard
#define PUCK_RADIUS   10.0f   // toolsetter footprint half-width
#define PUCK_DROP     10.0f   // puck top this far below the G59.3 Z origin
#define SPOIL_LIFT     5.0f   // spoilboard this far above the bottom of Z travel

typedef struct { float min[3], max[3]; } sim_box_t;

static bool sim_probe_armed = false;
static struct {
    bool valid;
    float spoil_z;
    sim_box_t stock, puck;
} probe_geom;

static inline bool sim_in_box (const sim_box_t *b, const float *p)
{
    return p[X_AXIS] >= b->min[X_AXIS] && p[X_AXIS] <= b->max[X_AXIS] &&
           p[Y_AXIS] >= b->min[Y_AXIS] && p[Y_AXIS] <= b->max[Y_AXIS] &&
           p[Z_AXIS] >= b->min[Z_AXIS] && p[Z_AXIS] <= b->max[Z_AXIS];
}

// ----- Explicit fixture setup (-setup <file>) ------------------------------------------------------
// When a setup file is supplied the simulated fixtures are defined explicitly (in machine mm) instead of
// being derived from the Z envelope + G28/G59.3. This decouples the spoilboard height from the mechanical
// Z travel - the macros probe to fixed absolute depths, so the spoilboard must sit where they expect, not
// at the bottom of a tall envelope. The same values drive the controller's G28/G30/G59.3 offsets (written
// to NVS at boot, see sim_setup_apply_offsets) so the operator does not set them by hand: G28 = stock corner
// less a clearance (Z just above the spoilboard), G59.3 = toolsetter X/Y at Z0, G30 = tool-change X/Y at Z0.
#define G28_CORNER_CLEAR 11.0f   // G28 sits this far outside the stock corner (-X/-Y); macro wants 10-30
#define G28_Z_ABOVE_SPOIL 4.0f   // G28 Z this far above the spoilboard
static struct {
    bool active;
    bool applied;            // offsets written to NVS yet (one-shot at boot, re-armed by a Settings edit)
    char path[260];          // setup file the values came from (so a Settings edit can persist them)
    char description[64];    // free-text machine name shown in the 3D view window title (e.g. "Mega V XL")
    float spoilboard_z;
    float stock_corner_x, stock_corner_y;
    float stock_size_x, stock_size_y, stock_size_z;
    float toolsetter_x, toolsetter_y, toolsetter_height;
    float toolchange_x, toolchange_y;
} sim_setup = {0};

static bool view_geom_pushed = false;   // 3D geometry pushed to the view; cleared to force a re-push

// Parse a "-setup" file: "key = value" lines, '#' or ';' begin a comment. Returns false if it cannot be
// opened. Unknown keys are ignored; missing keys keep their zero default. Called from main() at start-up.
bool sim_setup_load (const char *path)
{
    FILE *f = fopen(path, "r");
    if(f == NULL)
        return false;

    memset(&sim_setup, 0, sizeof(sim_setup));

    char line[160], key[64];
    double val;
    while(fgets(line, sizeof(line), f)) {
        char *c;
        if((c = strchr(line, '#'))) *c = '\0';
        if((c = strchr(line, ';'))) *c = '\0';
        char strval[64];
        if(sscanf(line, " description = %63[^\r\n]", strval) == 1) {   // free-text value, not a number
            char *e = strval + strlen(strval);                        // trim trailing whitespace
            while(e > strval && (e[-1] == ' ' || e[-1] == '\t')) *--e = '\0';
            strncpy(sim_setup.description, strval, sizeof(sim_setup.description) - 1);
            continue;
        }
        if(sscanf(line, " %63[A-Za-z_] = %lf", key, &val) != 2)
            continue;
        float v = (float)val;
        if(!strcmp(key, "spoilboard_z"))           sim_setup.spoilboard_z = v;
        else if(!strcmp(key, "stock_corner_x"))    sim_setup.stock_corner_x = v;
        else if(!strcmp(key, "stock_corner_y"))    sim_setup.stock_corner_y = v;
        else if(!strcmp(key, "stock_size_x"))      sim_setup.stock_size_x = v;
        else if(!strcmp(key, "stock_size_y"))      sim_setup.stock_size_y = v;
        else if(!strcmp(key, "stock_size_z"))      sim_setup.stock_size_z = v;
        else if(!strcmp(key, "toolsetter_x"))      sim_setup.toolsetter_x = v;
        else if(!strcmp(key, "toolsetter_y"))      sim_setup.toolsetter_y = v;
        else if(!strcmp(key, "toolsetter_height")) sim_setup.toolsetter_height = v;
        else if(!strcmp(key, "toolchange_x"))      sim_setup.toolchange_x = v;
        else if(!strcmp(key, "toolchange_y"))      sim_setup.toolchange_y = v;
    }

    fclose(f);
    sim_setup.active = true;
    strncpy(sim_setup.path, path, sizeof(sim_setup.path) - 1);
    sim_view_set_title(sim_setup.description);          // window title (stored now, applied when the view opens)

    fprintf(stderr, "setup: loaded %s\n  spoilboard_z=%.3f stock_corner=(%.3f,%.3f) size=(%.1f,%.1f,%.1f)"
                    " toolsetter=(%.3f,%.3f)+%.1f toolchange=(%.3f,%.3f)\n",
            path, sim_setup.spoilboard_z, sim_setup.stock_corner_x, sim_setup.stock_corner_y,
            sim_setup.stock_size_x, sim_setup.stock_size_y, sim_setup.stock_size_z,
            sim_setup.toolsetter_x, sim_setup.toolsetter_y, sim_setup.toolsetter_height,
            sim_setup.toolchange_x, sim_setup.toolchange_y);

    return true;
}

// Write the controller's G28/G30/G59.3 coordinate offsets from the setup, so the macros (which read these
// params) and the simulated fixtures agree without the operator setting them. One-shot, called once NVS is
// ready (see sim_process_realtime).
static void sim_setup_apply_offsets (void)
{
    coord_system_data_t d;

    memset(&d, 0, sizeof(d));                                            // G28: just outside the stock corner
    d.coord.values[X_AXIS] = sim_setup.stock_corner_x - G28_CORNER_CLEAR;
    d.coord.values[Y_AXIS] = sim_setup.stock_corner_y - G28_CORNER_CLEAR;
    d.coord.values[Z_AXIS] = sim_setup.spoilboard_z + G28_Z_ABOVE_SPOIL;
    settings_write_coord_data(CoordinateSystem_G28, &d);

    memset(&d, 0, sizeof(d));                                            // G59.3: toolsetter X/Y, Z = 0 (top)
    d.coord.values[X_AXIS] = sim_setup.toolsetter_x;                     // for max tool clearance approaching it;
    d.coord.values[Y_AXIS] = sim_setup.toolsetter_y;                     // the puck SURFACE stays at spoilboard_z
    settings_write_coord_data(CoordinateSystem_G59_3, &d);              // + toolsetter_height (see probe geom).

    memset(&d, 0, sizeof(d));                                            // G30: tool-change position, Z = 0 (top)
    d.coord.values[X_AXIS] = sim_setup.toolchange_x;
    d.coord.values[Y_AXIS] = sim_setup.toolchange_y;
    settings_write_coord_data(CoordinateSystem_G30, &d);

    sim_setup.applied = true;

    fprintf(stderr, "setup: wrote offsets  G28=(%.3f,%.3f,%.3f)  G59.3=(%.3f,%.3f,0)  G30=(%.3f,%.3f,0)\n",
            sim_setup.stock_corner_x - G28_CORNER_CLEAR, sim_setup.stock_corner_y - G28_CORNER_CLEAR,
            sim_setup.spoilboard_z + G28_Z_ABOVE_SPOIL,
            sim_setup.toolsetter_x, sim_setup.toolsetter_y, sim_setup.toolchange_x, sim_setup.toolchange_y);
}

// ---- Settings dialog accessors (sim_setup.h) -----------------------------------------------------
// Expose the setup values to the 3D view's Settings dialog and accept edits back. get/set copy field by
// field so the editable struct (sim_setup_values_t) stays decoupled from the driver's internal bookkeeping.

bool sim_setup_get_values (sim_setup_values_t *out)
{
    if(!sim_setup.active)
        return false;
    strncpy(out->description, sim_setup.description, sizeof(out->description) - 1);
    out->description[sizeof(out->description) - 1] = '\0';
    out->spoilboard_z      = sim_setup.spoilboard_z;
    out->stock_corner_x    = sim_setup.stock_corner_x;
    out->stock_corner_y    = sim_setup.stock_corner_y;
    out->stock_size_x      = sim_setup.stock_size_x;
    out->stock_size_y      = sim_setup.stock_size_y;
    out->stock_size_z      = sim_setup.stock_size_z;
    out->toolsetter_x      = sim_setup.toolsetter_x;
    out->toolsetter_y      = sim_setup.toolsetter_y;
    out->toolsetter_height = sim_setup.toolsetter_height;
    out->toolchange_x      = sim_setup.toolchange_x;
    out->toolchange_y      = sim_setup.toolchange_y;
    return true;
}

// Persist the current setup values back to the .cfg file they were loaded from (key = value lines).
static void sim_setup_save (void)
{
    if(sim_setup.path[0] == '\0')
        return;
    FILE *f = fopen(sim_setup.path, "w");
    if(f == NULL) {
        fprintf(stderr, "setup: could not write %s\n", sim_setup.path);
        return;
    }
    fprintf(f, "# grblHAL simulator fixture setup - edited from the 3D view Settings dialog\n");
    if(sim_setup.description[0])
        fprintf(f, "description = %s\n", sim_setup.description);
    fprintf(f, "spoilboard_z = %.3f\n", sim_setup.spoilboard_z);
    fprintf(f, "stock_corner_x = %.3f\n", sim_setup.stock_corner_x);
    fprintf(f, "stock_corner_y = %.3f\n", sim_setup.stock_corner_y);
    fprintf(f, "stock_size_x = %.3f\n", sim_setup.stock_size_x);
    fprintf(f, "stock_size_y = %.3f\n", sim_setup.stock_size_y);
    fprintf(f, "stock_size_z = %.3f\n", sim_setup.stock_size_z);
    fprintf(f, "toolsetter_x = %.3f\n", sim_setup.toolsetter_x);
    fprintf(f, "toolsetter_y = %.3f\n", sim_setup.toolsetter_y);
    fprintf(f, "toolsetter_height = %.3f\n", sim_setup.toolsetter_height);
    fprintf(f, "toolchange_x = %.3f\n", sim_setup.toolchange_x);
    fprintf(f, "toolchange_y = %.3f\n", sim_setup.toolchange_y);
    fclose(f);
    fprintf(stderr, "setup: saved %s\n", sim_setup.path);
}

void sim_setup_set_values (const sim_setup_values_t *in)
{
    strncpy(sim_setup.description, in->description, sizeof(sim_setup.description) - 1);
    sim_setup.description[sizeof(sim_setup.description) - 1] = '\0';
    sim_view_set_title(sim_setup.description);
    sim_setup.spoilboard_z      = in->spoilboard_z;
    sim_setup.stock_corner_x    = in->stock_corner_x;
    sim_setup.stock_corner_y    = in->stock_corner_y;
    sim_setup.stock_size_x      = in->stock_size_x;
    sim_setup.stock_size_y      = in->stock_size_y;
    sim_setup.stock_size_z      = in->stock_size_z;
    sim_setup.toolsetter_x      = in->toolsetter_x;
    sim_setup.toolsetter_y      = in->toolsetter_y;
    sim_setup.toolsetter_height = in->toolsetter_height;
    sim_setup.toolchange_x      = in->toolchange_x;
    sim_setup.toolchange_y      = in->toolchange_y;

    sim_setup_save();

    // Re-arm the one-shots so the grbl-thread realtime loop re-applies the G28/G30/G59.3 offsets and
    // re-pushes the 3D geometry on its next tick (benign cross-thread bool writes).
    sim_setup.applied = false;
    view_geom_pushed = false;
}

// ----- Tool table (for the material-removal carve) -------------------------------------------------
// grblHAL's tool_data_t has no cutter diameter/shape we can rely on (radius is "currently unsupported"
// and N_TOOLS is 0 here), so the cutter geometry is supplied by the gcode itself as comment lines the CAM
// post emits near the top of the program:  (TOOL T=1 D=6.35 TYPE=FLAT)  /  (TOOL T=3 D=12.7 TYPE=VBIT A=60)
// These are parsed via grbl.on_gcode_comment; real controllers ignore them. The active tool is tracked via
// the tool-change hooks, and its diameter/shape are pushed to the 3D view which carves the stock heightmap.
#define SIM_MAX_TOOL 64
typedef struct {
    bool  defined;
    float diameter;             // mm
    int   shape;                // sim_tool_shape_t
    float vangle;               // V-bit included angle (deg)
} sim_tool_t;
static sim_tool_t sim_tools[SIM_MAX_TOOL + 1];
static volatile int sim_active_tool = -1;

static on_gcode_message_ptr on_gcode_comment;   // chained (on_gcode_comment shares the message ptr type)
static on_tool_changed_ptr  on_tool_changed;    // chained (core assigns this)
static on_tool_selected_ptr on_tool_selected;   // chained

// Push the active tool's diameter/shape to the 3D view (so subsequent moves carve with the right cutter).
static void sim_push_tool_geometry (void)
{
    if(sim_active_tool >= 0 && sim_active_tool <= SIM_MAX_TOOL && sim_tools[sim_active_tool].defined) {
        sim_tool_t *t = &sim_tools[sim_active_tool];
        sim_view_set_tool_geometry(t->diameter, t->shape, t->vangle);
    } else
        sim_view_set_tool_geometry(0.0f, SIM_TOOL_FLAT, 0.0f);   // unknown tool -> no carving
}

// Geometry parsed from a tool comment with no T= (an end-of-line comment on the M6 line); it is applied
// to the next tool selected. Lets the post tag the cutter on the tool-change line, e.g. T1 M6 (TOOL D=6.35
// TYPE=FLAT), as well as in a top-of-file (TOOL T=1 ...) block.
static sim_tool_t pending_tool = {0};

// Parse a "(TOOL ...)" comment. Receives the comment body without the parentheses, e.g. "TOOL T=1 D=6.35
// TYPE=FLAT" (top-of-file table) or "TOOL D=6.35 TYPE=FLAT" (no T= -> the tool being changed to). Unknown
// keys are ignored; case-insensitive.
static status_code_t sim_on_gcode_comment (char *comment)
{
    char buf[120];
    strncpy(buf, comment, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tok = strtok(buf, " \t");
    if(tok && !strcasecmp(tok, "TOOL")) {
        int   id = -1;
        sim_tool_t t = { .defined = true, .diameter = 0.0f, .shape = SIM_TOOL_FLAT, .vangle = 0.0f };
        while((tok = strtok(NULL, " \t")) != NULL) {
            char *eq = strchr(tok, '=');
            if(eq == NULL)
                continue;
            *eq++ = '\0';
            if(!strcasecmp(tok, "T"))         id = atoi(eq);
            else if(!strcasecmp(tok, "D"))    t.diameter = (float)atof(eq);
            else if(!strcasecmp(tok, "A"))    t.vangle = (float)atof(eq);
            else if(!strcasecmp(tok, "TYPE")) {
                if(!strcasecmp(eq, "BALL"))      t.shape = SIM_TOOL_BALL;
                else if(!strcasecmp(eq, "VBIT")) t.shape = SIM_TOOL_VBIT;
                else                             t.shape = SIM_TOOL_FLAT;   // FLAT / ENDMILL / unknown
            }
        }
        if(t.diameter <= 0.0f) {
            // A "TOOL" comment with no diameter is not a tool definition - e.g. the post's header line
            // "(Tool table - generated by ...)" whose first word "Tool" matches case-insensitively. Ignoring
            // it is essential: otherwise it would become a zero-diameter pending tool and overwrite the real
            // tool on the next M6, leaving the carve with cut_dia=0.
        } else if(id >= 0 && id <= SIM_MAX_TOOL) {           // explicit tool number -> store directly
            sim_tools[id] = t;
            fprintf(stderr, "tool: T%d D=%.3f TYPE=%d A=%.1f\n", id, t.diameter, t.shape, t.vangle);
            if(id == sim_active_tool)
                sim_push_tool_geometry();
        } else {                                            // no T= -> apply to the next tool selected
            pending_tool = t;
            fprintf(stderr, "tool: pending D=%.3f TYPE=%d A=%.1f (next M6)\n", t.diameter, t.shape, t.vangle);
        }
    }

    return on_gcode_comment ? on_gcode_comment(comment) : Status_OK;
}

// Apply any pending (no-T) tool comment to the tool now being selected, then make it the active cutter.
static void sim_set_active_tool (int id)
{
    if(pending_tool.defined && id >= 0 && id <= SIM_MAX_TOOL) {
        sim_tools[id] = pending_tool;
        pending_tool.defined = false;
        fprintf(stderr, "tool: T%d D=%.3f TYPE=%d (from M6-line comment)\n",
                id, sim_tools[id].diameter, sim_tools[id].shape);
    }
    sim_active_tool = id;
    sim_push_tool_geometry();
}

static void sim_on_tool_selected (tool_data_t *tool)
{
    sim_set_active_tool((int)tool->tool_id);
    if(on_tool_selected)
        on_tool_selected(tool);
}

static void sim_on_tool_changed (tool_data_t *tool)
{
    sim_set_active_tool((int)tool->tool_id);
    fprintf(stderr, "tool: active T%d%s\n", sim_active_tool,
            (sim_active_tool >= 0 && sim_active_tool <= SIM_MAX_TOOL && sim_tools[sim_active_tool].defined)
                ? "" : " (no geometry - not carving)");
    if(on_tool_changed)
        on_tool_changed(tool);
}

// Build the 3D view's static geometry (machine envelope + spoilboard/stock/puck) from the live settings
// and the -setup fixtures, and hand it to sim_view. Called once settings are loaded (only when -view is on).
static void sim_view_push_geometry (void)
{
    sim_view_geometry_t g;
    memset(&g, 0, sizeof(g));

    uint_fast8_t i = 0;
    do {
        float travel = -settings.axis[i].max_travel;            // max_travel is stored negative
        if(bit_istrue(settings.homing.dir_mask.value, bit(i))) {  // homes to the negative end -> extends +
            g.env_min[i] = 0.0f; g.env_max[i] = travel;
        } else {                                                  // -> extends negative
            g.env_min[i] = -travel; g.env_max[i] = 0.0f;
        }
    } while(++i < 3);

    if(sim_setup.active) {
        g.have_fixtures = true;
        g.spoil_z = sim_setup.spoilboard_z;
        g.stock_min[X_AXIS] = sim_setup.stock_corner_x;
        g.stock_min[Y_AXIS] = sim_setup.stock_corner_y;
        g.stock_min[Z_AXIS] = sim_setup.spoilboard_z;
        g.stock_max[X_AXIS] = sim_setup.stock_corner_x + sim_setup.stock_size_x;
        g.stock_max[Y_AXIS] = sim_setup.stock_corner_y + sim_setup.stock_size_y;
        g.stock_max[Z_AXIS] = sim_setup.spoilboard_z + sim_setup.stock_size_z;
        g.puck_min[X_AXIS] = sim_setup.toolsetter_x - PUCK_RADIUS;
        g.puck_min[Y_AXIS] = sim_setup.toolsetter_y - PUCK_RADIUS;
        g.puck_min[Z_AXIS] = sim_setup.spoilboard_z;
        g.puck_max[X_AXIS] = sim_setup.toolsetter_x + PUCK_RADIUS;
        g.puck_max[Y_AXIS] = sim_setup.toolsetter_y + PUCK_RADIUS;
        g.puck_max[Z_AXIS] = sim_setup.spoilboard_z + sim_setup.toolsetter_height;
    } else
        g.spoil_z = settings.axis[Z_AXIS].max_travel + SPOIL_LIFT;

    sim_view_set_geometry(&g);
}

// Build the probe targets. With a -setup file the fixtures are taken verbatim from it; otherwise they are
// derived from the live G28 (#5161/#5162) and G59.3 (#5381-#5383) offsets plus the Z envelope. Called when
// a probe cycle arms so the surfaces always match the current setup.
static void sim_compute_probe_geom (void)
{
    if(sim_setup.active) {

        probe_geom.valid = true;
        probe_geom.spoil_z = sim_setup.spoilboard_z;

        probe_geom.stock.min[X_AXIS] = sim_setup.stock_corner_x;
        probe_geom.stock.min[Y_AXIS] = sim_setup.stock_corner_y;
        probe_geom.stock.min[Z_AXIS] = sim_setup.spoilboard_z;
        probe_geom.stock.max[X_AXIS] = sim_setup.stock_corner_x + sim_setup.stock_size_x;
        probe_geom.stock.max[Y_AXIS] = sim_setup.stock_corner_y + sim_setup.stock_size_y;
        probe_geom.stock.max[Z_AXIS] = sim_setup.spoilboard_z + sim_setup.stock_size_z;

        probe_geom.puck.min[X_AXIS] = sim_setup.toolsetter_x - PUCK_RADIUS;
        probe_geom.puck.min[Y_AXIS] = sim_setup.toolsetter_y - PUCK_RADIUS;
        probe_geom.puck.min[Z_AXIS] = sim_setup.spoilboard_z;
        probe_geom.puck.max[X_AXIS] = sim_setup.toolsetter_x + PUCK_RADIUS;
        probe_geom.puck.max[Y_AXIS] = sim_setup.toolsetter_y + PUCK_RADIUS;
        probe_geom.puck.max[Z_AXIS] = sim_setup.spoilboard_z + sim_setup.toolsetter_height;

        return;
    }

    float g28x, g28y, g59x, g59y, g59z;

    probe_geom.valid = ngc_param_get(5161, &g28x) && ngc_param_get(5162, &g28y) &&
                       ngc_param_get(5381, &g59x) && ngc_param_get(5382, &g59y) && ngc_param_get(5383, &g59z);
    if(!probe_geom.valid)
        return;

    probe_geom.spoil_z = settings.axis[Z_AXIS].max_travel + SPOIL_LIFT;   // max_travel is negative (envelope bottom)

    probe_geom.stock.min[X_AXIS] = g28x + STOCK_INSET;
    probe_geom.stock.min[Y_AXIS] = g28y + STOCK_INSET;
    probe_geom.stock.min[Z_AXIS] = probe_geom.spoil_z;
    probe_geom.stock.max[X_AXIS] = probe_geom.stock.min[X_AXIS] + STOCK_SIZE;
    probe_geom.stock.max[Y_AXIS] = probe_geom.stock.min[Y_AXIS] + STOCK_SIZE;
    probe_geom.stock.max[Z_AXIS] = probe_geom.spoil_z + STOCK_THICK;

    probe_geom.puck.min[X_AXIS] = g59x - PUCK_RADIUS;
    probe_geom.puck.min[Y_AXIS] = g59y - PUCK_RADIUS;
    probe_geom.puck.min[Z_AXIS] = probe_geom.spoil_z;
    probe_geom.puck.max[X_AXIS] = g59x + PUCK_RADIUS;
    probe_geom.puck.max[Y_AXIS] = g59y + PUCK_RADIUS;
    probe_geom.puck.max[Z_AXIS] = g59z - PUCK_DROP;
}

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

    // Probe: once armed (by probeConfigureInvertMask) trip when the controlled point enters a target
    // solid - the stock block (top/left/front faces), the toolsetter puck, or the spoilboard plane.
    // This reproduces a real touch: each face stops the probe at its actual position, so X/Y/Z corner
    // finds and the Z toolsetter all work and yield meaningful coordinates.
    if(sim_probe_armed && probe_geom.valid) {
        // Use the controller's real machine position (the same coordinate the PRB report and the G28 /
        // G59.3 offsets are in), NOT sim_axis_pos - that is an independent carriage tracker for limit
        // simulation and is offset from machine coordinates after homing.
        float p[N_AXIS];
        system_convert_array_steps_to_mpos(p, sys.position);
        bool tripped = sim_in_box(&probe_geom.stock, p) || sim_in_box(&probe_geom.puck, p) ||
                       p[Z_AXIS] <= probe_geom.spoil_z;   // spoilboard: everywhere not over an object
        // Drive the ELECTRICAL pin pre-inverted by $6 (probe invert), exactly as the limit inputs above are
        // pre-inverted by $5: probeGetState() re-applies probe_invert, so grbl reads the true logical
        // "triggered" = in-contact state for any probe polarity. Without this an inverted ($6=1) probe reads
        // triggered while off-contact, so a probe cycle trips Alarm 4 the instant it arms.
        mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT | ((tripped ^ probe_invert) ? PROBE_BIT : 0), PROBE_MASK);
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

  // Arm the simulated probe at the start of a probe cycle: snapshot the target geometry from the live
  // G28 / G59.3 offsets so the surfaces match the current setup. Release the trigger when it ends.
  if (probing) {
      if (!sim_probe_armed) {
          sim_compute_probe_geom();
          sim_probe_armed = true;
      }
  } else {
      sim_probe_armed = false;
      // Disarmed: present "not in contact", pre-inverted by $6 so probeGetState() reports not-triggered
      // (an inverted probe must read deasserted at idle, else Pn:P shows and the next probe cycle Alarm 4s).
      mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT | (probe_invert ? PROBE_BIT : 0), PROBE_MASK);
  }
}

// Active probe input. The simulated probe trips on geometry (the controlled point entering a target solid)
// rather than a physical input, so the selection is cosmetic - it just rides along in the status report.
static probe_id_t sim_probe_id = Probe_Default;

// Select the active probe (main / toolsetter / probe2). Required for the built-in G65 P5 Q<n> probe-select
// macro the ATC tool-change macro uses; without it G65 P5 Q1 returns error 39 (Status_GcodeValueOutOfRange).
static bool probeSelect (probe_id_t probe_id)
{
    sim_probe_id = probe_id;
    return true;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);

    state.triggered ^= probe_invert;
    state.probe_id = sim_probe_id;

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
    // One-shot, once settings are loaded: park the carriage over the centre of the spoilboard at Z = 0
    // (machine top) instead of booting on the home corner, so the tool is visible in the 3D view straight
    // away (before homing) and a subsequent $H is a clearly visible move. sys.position is what grbl reports
    // (drives the displayed tool); sim_axis_pos is the sim's own switch-tripping counter - set both so they
    // stay in step. The work envelope is [0, |max_travel|] for an axis that homes toward its negative end
    // (extends +) and [-|max_travel|, 0] otherwise, so the centre depends on the homing direction - using
    // max_travel/2 (always negative) put X/Y outside the envelope and off-screen until homing corrected it.
    static bool start_pos_set = false;
    if(!start_pos_set && settings.axis[X_AXIS].steps_per_mm > 0.0f) {
        start_pos_set = true;
        uint_fast8_t i = 0;
        do {
            float pos_mm;
            if(i == Z_AXIS)
                pos_mm = 0.0f;                                      // machine top - tool hovers above the stock
            else {
                float travel = -settings.axis[i].max_travel;       // positive magnitude
                pos_mm = bit_istrue(settings.homing.dir_mask.value, bit(i)) ? travel * 0.5f : travel * -0.5f;
            }
            int32_t steps = (int32_t)(pos_mm * settings.axis[i].steps_per_mm);
            sys.position[i] = steps;
            sim_axis_pos[i] = steps;
        } while(++i < N_AXIS);
    }

    // One-shot, once settings/NVS are live: write the G28/G30/G59.3 offsets from the -setup file so the
    // macros find them without the operator setting them by hand.
    if(sim_setup.active && !sim_setup.applied && settings.axis[X_AXIS].steps_per_mm > 0.0f)
        sim_setup_apply_offsets();

    // Feed the optional 3D view (-view): push the static geometry once settings are live, then the live
    // tool position every tick. Skipped entirely when -view is off (sim_view_active() == false).
    if(sim_view_active()) {
        if(!view_geom_pushed && settings.axis[X_AXIS].steps_per_mm > 0.0f) {
            sim_view_push_geometry();
            view_geom_pushed = true;
        }
        float p[N_AXIS];
        system_convert_array_steps_to_mpos(p, sys.position);
        sim_view_set_tool(p[X_AXIS], p[Y_AXIS], p[Z_AXIS]);
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

    // Tool table comments + active-tool tracking feed the 3D view's material-removal carve. Chain the
    // tool-change hooks (the core assigns on_tool_changed) so existing behaviour is preserved.
    on_gcode_comment = grbl.on_gcode_comment;
    grbl.on_gcode_comment = sim_on_gcode_comment;
    on_tool_selected = grbl.on_tool_selected;
    grbl.on_tool_selected = sim_on_tool_selected;
    on_tool_changed = grbl.on_tool_changed;
    grbl.on_tool_changed = sim_on_tool_changed;

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
    hal.probe.select = probeSelect;
    hal.driver_cap.toolsetter = On;             // advertise a toolsetter input so G65 P5 Q1 selects it

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
// Mount at /littlefs (as a real board does) even in the LITTLEFS_ENABLE==2 build, so ioSender's
// absolute-path ATC provisioning (/littlefs/<name>.macro over YModem) and the named-sub resolver
// (which searches /littlefs) line up. atc_macros_attach() keys tc_path off this mount path, so M6
// and ATC detection follow automatically.
#define LITTLEFS_MOUNT_DIR "/littlefs"
#endif
    fs_littlefs_mount(LITTLEFS_MOUNT_DIR, sim_littlefs_hal());
#endif

#if SDCARD_ENABLE || LITTLEFS_ENABLE
    // Advertise persistent storage (matches a real board that reports SD in $I NEWOPT). Without this
    // grbllib.c clears settings.macro_atc_flags.error_on_no_macro ($675 bit 1) on every boot - it gates
    // that flag on hal.driver_cap.sd_card only - so the ATC-macro flow ($675=2 -> ATC=0 when tc.macro is
    // missing) could never be set/persisted on this littlefs-only build, blocking ATC testing.
    hal.driver_cap.sd_card = On;
#endif

    // Enable $REBOOT (system.c reboot_system returns error:3 when hal.reboot is NULL). sim_reboot
    // re-execs the process - a fresh boot that re-mounts littlefs, so an uploaded tc.macro brings ATC
    // online without a manual restart.
    {
        extern void sim_reboot (void);
        hal.reboot = sim_reboot;
    }

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
