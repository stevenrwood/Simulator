/*
  sim_setup.h - editable fixture/setup values shared between the driver (which stores them, applies the
  G28/G30/G59.3 offsets and drives the simulated fixtures) and the 3D view's Settings dialog (which edits
  them live). All distances are in machine millimetres.
*/

#pragma once

#include <stdbool.h>

typedef struct {
    char  description[64];      // free-text machine name shown in the 3D view window title
    float spoilboard_z;
    float stock_corner_x, stock_corner_y;
    float stock_size_x, stock_size_y, stock_size_z;
    float toolsetter_x, toolsetter_y, toolsetter_height;
    float toolchange_x, toolchange_y;
    float resolution_mm;        // material-removal cell size (mm); smaller = finer (0 = default)
} sim_setup_values_t;

// Fetch the current setup values for editing. Returns false if no -setup is active.
bool sim_setup_get_values (sim_setup_values_t *out);

// Apply edited values: update the live setup, re-apply the controller offsets + re-push the 3D geometry
// (on the next realtime tick) and persist them back to the setup .cfg file. Safe to call from any thread.
void sim_setup_set_values (const sim_setup_values_t *in);
