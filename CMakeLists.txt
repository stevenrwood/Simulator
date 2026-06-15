cmake_minimum_required(VERSION 3.10)
project(grblHAL-sim C)

add_compile_definitions(F_CPU=16000000)
#add_compile_definitions(SQUARING_ENABLED=1)

# Filesystem support: littlefs at root (=2 also enables YModem and fs_stream_init).
# SDCARD_ENABLE=0 is required - the sim has no SD hardware, and a non-zero default would both
# downgrade LITTLEFS_ENABLE 2->1 and pull in FatFs/SD code that cannot build here.
# NGC_EXPRESSIONS_ENABLE=1 for O<name> CALL macros + the ATC tool-change flow.
add_compile_definitions(LITTLEFS_ENABLE=2 SDCARD_ENABLE=0 NGC_EXPRESSIONS_ENABLE=1)

# Case-insensitive littlefs name matching (vendored lfs.c) so O<name> CALL macros resolve regardless
# of the g-code parser upper-casing the label - matching an SD card FatFs, which the VFS treats the same.
add_compile_definitions(LFS_CASE_INSENSITIVE)

# src/ so sources find "driver.h" / "grbl/..."; the sdcard plugin reaches littlefs via "../" itself.
include_directories(src)

include(src/grbl/CMakeLists.txt)

# SD card plugin (fs_stream/fs_littlefs/macros/ymodem). Built as its own static lib with grbl/hal.h
# force-included: these sources gate on FS_ENABLE before including any grbl header, so the config must
# already be in scope. Force-including here (instead of via driver.h) keeps the grbl headers ahead of
# <string.h> et al - avoiding a sim header-order conflict - and confines it to the plugin. fs_fatfs.c
# and sdcard.c are omitted (SDCARD_ENABLE=0: no SD hardware, and they need FatFs headers).
add_library(sdcard STATIC
    src/sdcard/fs_stream.c
    src/sdcard/fs_littlefs.c
    src/sdcard/macros.c
    src/sdcard/ymodem.c
)
target_include_directories(sdcard PRIVATE src src/sdcard src/littlefs)
target_compile_options(sdcard PRIVATE -include sim_plugin_prelude.h)

if (WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)

    set(platform_SRC
        src/platform_windows.h
        src/platform_windows.c
    )

    set(platform_LIB
        ws2_32
    )
endif(WIN32)

if(UNIX)
    add_compile_definitions(PLATFORM_LINUX)

    set(platform_SRC
        src/platform_linux.h
        src/platform_linux.c
    )

    if(APPLE)
        set(platform_LIB
            pthread
        )
    else(APPLE)
        set(platform_LIB
            rt
            pthread
        )
    endif(APPLE)
endif(UNIX)

add_executable(grblHAL_sim
    src/main.c
    src/simulator.c
    src/driver.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/mcu.c
    src/serial.c
    src/grbl_interface.c
    src/littlefs_hal.c
    src/littlefs/lfs.c
    src/littlefs/lfs_util.c
    ${platform_SRC}
)

target_link_libraries(grblHAL_sim PRIVATE
    m
    grbl
    sdcard
    ${platform_LIB}
)

add_executable(grblHAL_validator 
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/validator.c 
    src/validator_driver.c 
    ${platform_SRC}
)

target_link_libraries(grblHAL_validator PRIVATE 
    m
    grbl
    ${platform_LIB}
)
