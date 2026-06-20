/*
  sim_view.h - optional 3D machine view for the grblHAL simulator (-view).

  A self-contained Win32 + OpenGL window (no external dependencies) that draws the
  machine envelope, the spoilboard / stock / toolsetter fixtures (from the -setup
  file) and a cone at the live tool position, all in true machine coordinates.
  No-ops on non-Windows builds.
*/

#pragma once

#include <stdbool.h>

typedef struct {
    bool have_fixtures;                 // stock/puck/spoilboard defined (a -setup file is active)
    float env_min[3], env_max[3];       // machine envelope (mm)
    float spoil_z;                      // spoilboard plane Z (mm)
    float stock_min[3], stock_max[3];   // stock block (mm)
    float puck_min[3], puck_max[3];     // toolsetter puck bounding box (mm)
} sim_view_geometry_t;

// Create the view window + render thread. Safe to call once; ignored if already running.
void sim_view_start (void);

// True once the window/render thread is up (so callers can skip work when -view is off).
bool sim_view_active (void);

// Publish the static fixture/envelope geometry (call once settings + setup are live).
void sim_view_set_geometry (const sim_view_geometry_t *g);

// Publish the live tool tip position in machine coordinates (call each realtime tick). When the stock
// heightmap is active this also carves material the cutter has passed through (see sim_view_set_tool_geometry).
void sim_view_set_tool (float x, float y, float z);

// Cutter shape for the material-removal carve.
typedef enum { SIM_TOOL_FLAT = 0, SIM_TOOL_BALL = 1, SIM_TOOL_VBIT = 2 } sim_tool_shape_t;

// Publish the active cutter geometry (diameter mm, shape, V-bit included angle deg) used to carve the stock.
void sim_view_set_tool_geometry (float diameter, int shape, float vangle);

// Reset the stock heightmap back to an uncut block (e.g. before re-running a job).
void sim_view_reset_stock (void);

// Set the 3D view window title (the setup "description", e.g. "Mega V XL").
void sim_view_set_title (const char *s);

// Publish the latest controller [MSG:...] line for the status line at the bottom of the view.
void sim_view_set_message (const char *s);

// Append one line to the action log shown by the Show Log window.
void sim_view_log_append (const char *s);
