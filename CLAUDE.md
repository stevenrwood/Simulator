# Claude Code guide — stevenrwood/Simulator fork

A **fork of [grblHAL/Simulator](https://github.com/grblHAL/Simulator)** — the PC grblHAL simulator that
ioSender's Connect dialog can launch. It adds a 3D material-removal view, modeled probing, littlefs,
settings persistence + `$REBOOT`, and a YModem-over-socket fix, as clean `pr/*` branches.

## Read first
- **`ProposedPRs.html` / `ProposedPRs.pdf`** — every PR (features, file/line counts, cross-fork deps).
- Cross-fork **`Overview.pdf`** lives in the [ioSender fork](https://github.com/stevenrwood/ioSender).

## Build (cross-platform CMake + C)
The base sim builds everywhere; the 3D view (`-view`) is Windows-only today.

- **Windows (MinGW):** `cmake -S . -B build -G "MinGW Makefiles"` then
  `cmake --build build --clean-first --parallel` → `build/grblHAL_sim.exe` (+ `grblHAL_validator.exe`).
- **macOS / Linux:** `brew install cmake` (+ `xcode-select --install` on Mac), `cmake -S . -B build`,
  `cmake --build build`. See **`BUILD-MACOS.md`** — the base sim should build unchanged; the 3D view is a
  Phase-2 GLFW/SDL2 port tracked on branch `pr/macos-build`.

Run: `./build/grblHAL_sim -p 23` (listens on TCP 23); point a sender at `127.0.0.1:23`.

## Branch model & all-or-nothing
- **`master`** = pristine upstream. **`integration`** (default) = all PRs merged — what you actually run.
- **`pr/<name>`** = one focused change each, off `master`, for upstream review / cherry-pick. Two
  companion fixes live in submodule forks (`core`, `Plugin_SD_card`) and ride with `integration`.
- The features interlock, so **for *using* the sim, take `integration`** (clone with
  `--recurse-submodules`). Subset composition is a dev/review aid only.

## Compose (dev/review only) — from the ioSender repo
`python tools/apply-prs.py sim-build --fork sim 1 2 3 4 5` — resolves deps, CMake-builds, verifies.

## Pairing
Pair with **ioSender at `integration`** for the full virtual-CNC experience: the 3D carve renders from
`(STOCK)`/`(TOOL)` geometry that ioSender's Fusion add-in produces and forwards — see the ioSender
`Overview`. Line endings: LF.
