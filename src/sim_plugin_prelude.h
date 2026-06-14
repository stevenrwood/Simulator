/*
  sim_plugin_prelude.h - force-included (-include) before the SD card plugin sources in the simulator.

  Two jobs, order matters:

   1. Pull in the grbl configuration so FS_ENABLE / LITTLEFS_ENABLE are computed. The plugin sources
      gate on FS_ENABLE before including any grbl header, and the simulator's driver.h - unlike a real
      target's - does not include the config. driver_opts.h must come FIRST: the simulator's grbl
      headers have an include-order sensitivity ("On" redefinition) if <string.h> et al precede them.

   2. Work around a host-libc clash. MinGW defines errno as a macro, but the plugin uses 'errno' as a
      local variable name (valid on bare-metal targets). Include the libc headers that reference errno
      while the macro is still defined, then strip it so the plugin's later local 'errno' is a plain
      variable; the headers' include guards suppress the re-definition when the plugin includes them.
*/

#pragma once

#include "grbl/driver_opts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <time.h>
#include <limits.h>
#include <errno.h>
#undef errno
