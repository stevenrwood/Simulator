cmake_minimum_required(VERSION 3.10)
project(grblHAL-sim C)

add_compile_definitions(F_CPU=16000000)
#add_compile_definitions(SQUARING_ENABLED=1)

# Filesystem support: littlefs at root (=2 also enables YModem and fs_stream_init).
# SDCARD_ENABLE=0 is required - the sim has no SD hardware, and a non-zero default would both
# downgrade LITTLEFS_ENABLE 2->1 and pull in FatFs/SD code that cannot build here.
# NGC_EXPRESSIONS_ENABLE=1 for O<name> CALL macros + the ATC tool-change flow.
add_compile_definitions(LITTLEFS_ENABLE=2 SDCARD_ENABLE=0 NGC_EXPRESSIONS_ENABLE=1)

# N_TOOLS left at 0 (no native tool table): the ATC flow uses toolsetter probing + dynamic G43.1 TLO and
# the carve reads cutter geometry from (TOOL ...) gcode comments, so the native tool table is unused. The
# tool-change hooks (grbl.on_tool_changed) that drive the carve geometry fire regardless of N_TOOLS.

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
    src/sim_view.c
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

# The optional 3D machine view (-view) uses Win32 + OpenGL (system libraries only).
if(WIN32)
    target_link_libraries(grblHAL_sim PRIVATE opengl32 glu32 gdi32 advapi32)
endif()

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

# --- Auto-deploy to a sibling ioSender checkout --------------------------------------------------------
# So a freshly built simulator is immediately the one ioSender loads, with no manual copy. The exe is
# copied into ioSender's source simulator/ folder (the single canonical home its csproj CopySimulator
# target pulls from) AND straight into the existing bin\...\simulator\ output folders so a running/next
# debug session picks it up without rebuilding ioSender. Entirely skipped when there is no sibling
# ioSender checkout next to this repo (e.g. on CI), so it never affects the release build.
set(_iosender "${CMAKE_SOURCE_DIR}/../ioSender")
if(EXISTS "${_iosender}/simulator")
    set(_sim_dests
        "${_iosender}/simulator"
        "${_iosender}/ioSender XL/ioSender XL/bin/Debug/simulator"
        "${_iosender}/ioSender XL/ioSender XL/bin/Release/simulator"
        "${_iosender}/ioSender/ioSender/bin/Debug/simulator")
    foreach(_d ${_sim_dests})
        if(EXISTS "${_d}")
            add_custom_command(TARGET grblHAL_sim POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:grblHAL_sim>" "${_d}/grblHAL_sim.exe"
                COMMENT "Deploy grblHAL_sim.exe -> ${_d}")
        endif()
    endforeach()
endif()
