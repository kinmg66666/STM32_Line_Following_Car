# AGENTS.md

## Project Shape
- This is a STM32CubeMX/Keil MDK project for `STM32F103C8Tx`; the root `.ioc` is the CubeMX source of truth and `MDK-ARM/STM32_Line_Following_Car01.uvprojx` is the build source of truth.
- There is no Makefile, CMake, CI, or test runner in this repo; verify firmware changes with Keil uVision/MDK-ARM V5 unless you add a build system intentionally.
- `Core/` and `Drivers/` are CubeMX/HAL-generated areas. Put hand-written code inside CubeMX `USER CODE BEGIN/END` blocks or outside generated files to avoid regeneration loss.

## Keil Build Gotchas
- The current Keil project compiles only `Core/Src/*`, startup assembly, and selected HAL/CMSIS files listed in `MDK-ARM/STM32_Line_Following_Car01.uvprojx`.
- `Code/App` and `Code/Algorithm` are not currently listed in the `.uvprojx`, and their include paths are not in the Keil include path. Add both source files and include paths to the project when making those modules part of firmware.
- Active defines from the Keil project are `USE_HAL_DRIVER,STM32F103xB`; keep them in sync if creating another compiler path.
- Keil output artifacts belong under `MDK-ARM/STM32_Line_Following_Car01/` or ignored build directories; do not commit generated `.hex`, `.axf`, `.map`, `.lst`, or local `.uvguix.*` files.

## Current Code Notes
- `Core/Src/main.c` only initializes HAL, configures the 72 MHz HSE/PLL clock, initializes GPIO, then idles; no app-level entrypoint is called yet.
- If `PID_t` is reported as unknown in `Code/Algorithm/alg_pid.c`, first check line 1: it currently uses smart quotes `#include “alg_pid.h”`; Keil needs ASCII quotes: `#include "alg_pid.h"`.
- `alg_pid.h` declares `PID_t` but no `PID_Init` prototype; keep declarations and definitions together when adding PID APIs.
