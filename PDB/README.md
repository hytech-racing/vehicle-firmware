# Converting a CubeMX project into our PlatformIO (STM32duino) setup

When you configure a board in STM32CubeMX and generate code, you get a full
standalone STM32CubeIDE project. We **don't** use that project directly — we
build with PlatformIO + the Arduino (STM32duino) framework, which already
provides all the generic chip support (HAL, CMSIS, startup, system init).

So the job is: **take only the parts of the generated project that describe
*your board's specific configuration*, and drop everything generic** (STM32duino
supplies it). If you copy the generic parts in too, you'll get duplicate-symbol
linker errors.

Below is what to keep, what to extract from, and what to ignore.

---

## ❌ Ignore completely — do NOT copy these in

STM32duino already provides its own copy of all of these. Copying them causes
duplicate definitions / linker conflicts.

| Path | Why ignore |
|---|---|
| `Drivers/STM32H7xx_HAL_Driver/` | Full HAL source — STM32duino bundles its own |
| `Drivers/CMSIS/` | CMSIS core + device headers — STM32duino has its own |
| `Core/Startup/startup_*.s` | Startup assembly (reset vector, stack) — framework provides it |
| `Core/Src/system_stm32*.c` | System init — framework provides it |
| `Core/Src/syscalls.c` | newlib syscall stubs — framework provides them |
| `Core/Src/sysmem.c` | Heap/`_sbrk` stubs — framework provides them |
| `.cproject`, `.project`, `.settings/` | STM32CubeIDE metadata — irrelevant to PlatformIO |
| `.mxproject` | CubeMX internal bookkeeping |
| `*_RAM.ld` | RAM-execution linker script — we flash to FLASH |

---

## ✂️ Extract from — copy specific pieces, not the whole file

These generated files contain your board-specific configuration. Pull the
relevant parts into the board's PlatformIO folder (`include/`, `src/`).

| Source file | What to take |
|---|---|
| `Core/Src/main.c` | `SystemClock_Config()`, every `MX_*_Init()` function, and `Error_Handler()` (unless the framework already defines one — see note) |
| `Core/Src/stm32h7xx_hal_msp.c` | The `HAL_*_MspInit()` functions for the peripherals you use. **Easy to forget** — this is where each peripheral's pins get their alternate-function mode and where the peripheral clocks are enabled. Without it, your peripherals' pins/clocks aren't wired up. |
| `Core/Inc/main.h` | The block of pin `#define`s (`XXX_Pin` / `XXX_GPIO_Port`). Put these in a `pins.h`. |
| `Core/Inc/stm32h7xx_hal_conf.h` | **Don't copy the file.** Use it as a checklist: every uncommented `#define HAL_XXX_MODULE_ENABLED` becomes a `-D HAL_XXX_MODULE_ENABLED` build flag in `platformio.ini`. |

---

## 📁 Keep as-is — copy the whole file

| File | Where it goes | Referenced by |
|---|---|---|
| `STM32H750VBTX_FLASH.ld` (the FLASH one) | board folder | `board_build.ldscript` in `platformio.ini` |
| `<Board>.ioc` | board folder | the CubeMX source of truth — commit it so anyone can reopen and regenerate |