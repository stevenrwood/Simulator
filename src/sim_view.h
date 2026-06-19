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

// Publish the live tool tip position in machine coordinates (call each realtime tick).
void sim_view_set_tool (float x, float y, float z);

// Publish the latest controller [MSG:...] line for the status line at the bottom of the view.
void sim_view_set_message (const char *s);
