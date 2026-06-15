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
static FILE *img = NULL;   // backing file kept open for the process lifetime (see sim_littlefs_hal)

static int sim_hal_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size)
{
    memcpy(buffer, fs + block * c->block_size + offset, size);

    return LFS_ERR_OK;
}

// Persist only the touched region to the backing file. The previous implementation rewrote the whole
// 512 KB image (fopen/fwrite/fclose) on every sync, and littlefs syncs many times per file operation -
// on Windows each close also triggers an AV scan of the image, so a small upload took minutes. Keeping
// the file open and writing just the changed bytes here makes prog/erase cheap and sync a plain flush.
static int sim_hal_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    size_t pos = block * c->block_size + offset;

    memcpy(fs + pos, buffer, size);

    if (img) {
        fseek(img, (long)pos, SEEK_SET);
        fwrite(fs + pos, 1, size, img);
    }

    return LFS_ERR_OK;
}

static int sim_hal_erase (const struct lfs_config *c, lfs_block_t block)
{
    size_t pos = block * c->block_size;

    memset(fs + pos, 0xFF, c->block_size);

    if (img) {
        fseek(img, (long)pos, SEEK_SET);
        fwrite(fs + pos, 1, c->block_size, img);
    }

    return LFS_ERR_OK;
}

static int sim_hal_sync (const struct lfs_config *c)
{
    (void)c;

    if (img)
        fflush(img);

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

    // Start from the erased state, then overlay any persisted image so files (uploaded macros, atc.sum,
    // ...) survive simulator restarts. The backing file is kept open ("r+b") for the rest of the run so
    // prog/erase can write just the changed bytes; on first run it is created and pre-filled with the
    // erased image.
    memset(fs, 0xFF, FS_SIZE);

    if ((img = fopen(IMG_FILE, "r+b"))) {
        if (fread(fs, 1, FS_SIZE, img) != FS_SIZE) {
            memset(fs, 0xFF, FS_SIZE);   // partial/short image - treat as fresh and rewrite it
            fseek(img, 0, SEEK_SET);
            fwrite(fs, 1, FS_SIZE, img);
            fflush(img);
        }
    } else if ((img = fopen(IMG_FILE, "w+b"))) {
        fwrite(fs, 1, FS_SIZE, img);   // fresh image
        fflush(img);
    }

    return &cfg;
}

#endif // LITTLEFS_ENABLE
