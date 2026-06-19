/*
  littlefs_hal.h - littlefs block device HAL for the grblHAL simulator

  Part of grblHAL

  Backs the VFS littlefs mount with a RAM buffer (optionally persisted to a host
  file) so the PC simulator can exercise the SD/littlefs, macro and ATC features.
*/

#pragma once

#include "grbl/vfs.h"
#include "littlefs/lfs.h"

struct lfs_config *sim_littlefs_hal (void);

// Request a one-shot format: the next sim_littlefs_hal() discards any persisted littlefs.img and starts
// from a freshly erased device, so the mount reformats it. Wired to the simulator's -format option.
void sim_littlefs_format_on_boot (void);
