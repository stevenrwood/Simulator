# Building grblHAL_sim on macOS

**Handoff note.** Written from a Windows session that could *not* compile or test on a Mac.
A Mac-side session (or a person on a Mac) should pick up here: verify the base build, then
do the Phase-2 3D-view port. Branch: `pr/macos-build` (off `integration`).

## TL;DR — the base sim should already build

The groundwork is in place, so this should *just work* for the virtual controller (homing,
jogging, g-code, probing, littlefs) — only the 3D carve view is Windows-only for now.

```bash
brew install cmake
xcode-select --install                 # clang + make (skip if already installed)
cd Simulator                           # this repo, on branch pr/macos-build
cmake -S . -B build
cmake --build build --parallel
./build/grblHAL_sim -p 23              # listens on TCP 23; point a sender at 127.0.0.1:23
```

Why it should already work:
- **CMake is platform-aware.** `if(UNIX)` (covers macOS) selects `platform_linux.c` + `PLATFORM_LINUX`; `if(APPLE)` already links only `pthread` (not Linux's `rt`).
- **The platform layer is POSIX.** `platform_linux.c` uses `clock_gettime(CLOCK_MONOTONIC)` (macOS 10.12+), pthreads and non-blocking stdin — all present on macOS. Upstream grblHAL/Simulator already targets Linux; macOS is the same POSIX surface.
- **The 3D view is already guarded.** `sim_view.c` is `#ifdef _WIN32` for the full Win32 + OpenGL implementation, with **no-op stubs** otherwise — so it compiles everywhere; the 3D window just doesn't open off-Windows yet.

This branch only adds this note plus one fix: the auto-deploy `POST_BUILD` step (which copies
`grblHAL_sim.exe` into a sibling ioSender checkout) is now guarded to `WIN32`, so it won't run
on macOS.

## If the build breaks — likely culprits (fix from the real compiler output)

These are guesses from a non-Mac box:
- **`pthread_cancel`** (`platform_linux.c`) — exists on macOS but cancellation semantics differ. If thread teardown misbehaves, switch to a flag/`pthread_kill`-based stop (the thread already polls an `exit` flag).
- **`-lm`** — harmless on macOS (libm is in libSystem). If CMake objects, drop `m` from the link on `APPLE`.
- **`clock_gettime`** — needs the macOS 10.12+ SDK (any modern Mac).
- **Warnings** — no `-Werror` is set, so a stricter clang diagnostic is cosmetic.

Run it: `./build/grblHAL_sim -p 23`, then connect ioSender (in a Windows VM) or any grbl sender
to `127.0.0.1:23`. The validator (`grblHAL_validator`) is already platform-clean and should also build.

## Phase 2 — the 3D carve view on macOS

The `-view` window is Windows-only today (`sim_view.c` under `#ifdef _WIN32`: a Win32 window +
WGL context + legacy OpenGL `gl.h`/`glu.h`, linking `opengl32 glu32 gdi32`). To bring the carve
to macOS without disturbing the Windows path:

1. **Window + GL context → a portable backend.** Add `src/sim_view_glfw.c` (or SDL2) that creates
   the window, GL context and input behind the *same* `sim_view.h` API. `brew install glfw`. The
   carve **logic** (heightmap, cutter sweep, all the `sim_view_set_*` calls, the status/log windows)
   is platform-independent — only the windowing/context/input is Win32. Factor the shared drawing out
   of the `#ifdef _WIN32` block so both backends call it.
2. **Legacy OpenGL on macOS.** macOS supports OpenGL ≤ 2.1 via a *compatibility* context (deprecated
   but functional) — enough for the immediate-mode draws here. `glu` is absent on macOS; reimplement
   the few `glu*` helpers used (`gluPerspective`, `gluLookAt` — a few lines each). Alternatively move
   the whole view to SDL2 + minimal modern GL.
3. **CMake.** Add a non-Windows branch: compile `sim_view_glfw.c` instead of the Win32 path; link
   `glfw` + the system OpenGL framework (`find_package(OpenGL REQUIRED)`; on macOS that's the
   `OpenGL` framework). Keep the Win32 path under `if(WIN32)`.

It's a contained windowing port — the carve already works on Windows, so there's no new rendering
logic to invent, just a cross-platform window/context/input layer.

## Status at handoff
- **Base sim + validator:** expected to build & run on macOS unchanged — **verify first.**
- **3D view:** no-ops on macOS (Phase 2 above).
- **Firmware (separate repo):** builds on macOS with no changes — `brew install platformio`, then
  `pio run -e teensy41` in `iMXRT1062/grblHAL_Teensy4` (PlatformIO is host-OS-agnostic).
- **ioSender:** WPF, Windows-only — run it in a Windows VM, or (large effort) port the UI to Avalonia.
  See `Overview.html` in the ioSender repo.
