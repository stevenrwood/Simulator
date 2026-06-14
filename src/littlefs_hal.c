/*
  littlefs_hal.c - littlefs block device HAL for the grblHAL simulator

  Part of grblHAL

  On real hardware littlefs is backed by a region of MCU flash. On the PC
  simulator there is no flash, so this backs it with a RAM buffer that is
  loaded from / synced to a host file (littlefs.img) - giving persistence
  across simulator restarts, just like real flash. This lets the simulator
  exercise the filesystem ($F/$FI/$F<=/YModem), the O<name> CALL macros and
  the ATC tool-change flow without target hardware.
*/

#include "driver.h"

#if LITTLEFS_ENABLE

#include <stdio.h>
#include <string.h>

#include "littlefs_hal.h"

#ifndef LFS_SIZE_KB
#define LFS_SIZE_KB 512                 // mirror typical target littlefs size
#endif
#define FS_SIZE     (LFS_SIZE_KB * 1024)
#define SECTOR_SIZE 4096
#define IMG_FILE    "littlefs.img"      // host-side persistence (created next to the working dir)

static uint8_t fs[FS_SIZE];

static int sim_hal_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size)
{
    memcpy(buffer, fs + block * c->block_size + offset, size);

    return LFS_ERR_OK;
}

static int sim_hal_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    memcpy(fs + block * c->block_size + offset, buffer, size);

    return LFS_ERR_OK;
}

static int sim_hal_erase (const struct lfs_config *c, lfs_block_t block)
{
    memset(fs + block * c->block_size, 0xFF, c->block_size);

    return LFS_ERR_OK;
}

static int sim_hal_sync (const struct lfs_config *c)
{
    (void)c;

    FILE *f = fopen(IMG_FILE, "wb");
    if (f) {
        fwrite(fs, 1, FS_SIZE, f);
        fclose(f);
    }

    return LFS_ERR_OK;
}

struct lfs_config *sim_littlefs_hal (void)
{
    static struct lfs_config cfg = {
        // block device operations
        .read = sim_hal_read,
        .prog = sim_hal_prog,
        .erase = sim_hal_erase,
        .sync = sim_hal_sync,
        // block device configuration
        .read_size = 128,
        .prog_size = 128,
        .block_size = SECTOR_SIZE,
        .block_count = FS_SIZE / SECTOR_SIZE,
        .cache_size = 128,
        .lookahead_size = 128,
        .block_cycles = 500
    };

    // Start from the erased state, then overlay any persisted image so files
    // (uploaded macros, atc.sum, ...) survive simulator restarts.
    memset(fs, 0xFF, FS_SIZE);

    FILE *f = fopen(IMG_FILE, "rb");
    if (f) {
        if (fread(fs, 1, FS_SIZE, f) != FS_SIZE)
            memset(fs, 0xFF, FS_SIZE);   // partial/short image - treat as fresh
        fclose(f);
    }

    return &cfg;
}

#endif // LITTLEFS_ENABLE
