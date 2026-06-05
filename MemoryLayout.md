# XIAO nRF52840 Bootloader Memory Layout

This project currently builds as an Adafruit nRF52 bootloader variant for
`BOARD=xiao_nrf52840_ble` with S140 SoftDevice `7.3.0`.

## Working Build Commands

Use the existing Make workflow for the XIAO board:

```sh
make BOARD=xiao_nrf52840_ble DEBUG=1 compile
```

The `compile` target builds the ELF, bootloader HEX, no-SoftDevice HEX, UF2
self-update file, and merged SoftDevice+bootloader HEX. It intentionally skips
the final DFU ZIP packaging step, which currently depends on a broken local
`adafruit-nrfutil` Python environment.

Existing one-letter helper scripts:

```sh
./c    # clean DEBUG=1 XIAO Make build
./m    # full DEBUG=1 XIAO Make build, including DFU ZIP packaging
./f    # OpenOCD flash.cfg
./a    # OpenOCD flashapp.cfg
./d    # OpenOCD RTT server config
./n    # connect to RTT server with nc
./t    # connect to OpenOCD telnet
```

CMake diagnostics also work after adding XIAO `board.cmake` files:

```sh
cmake -S . -B build-codex -G Ninja -DBOARD=xiao_nrf52840_ble -DCMAKE_BUILD_TYPE=Debug
cmake --build build-codex --target bootloader
```

## Environment Status

Present:

- `arm-none-eabi-gcc` 12.3.1
- `cmake` 3.28.1
- `ninja` 1.11.1
- Python `intelhex`
- OpenOCD 0.12.0

Needs fixing for full upstream-style tooling:

- `adafruit-nrfutil` imports `serial` from the wrong package. Install/fix
  `pyserial` and remove the conflicting `serial` package in the Python
  environment used by `/usr/local/bin/adafruit-nrfutil`.
- `nrfjprog` is installed but cannot load `libjlinkarm.dylib`. Reinstall or
  repair SEGGER J-Link if Nordic CLI flashing is needed.

## Current nRF52840 Regions

Hardware flash and active application layout:

| Region | Start | End exclusive | Size |
| --- | ---: | ---: | ---: |
| MBR | `0x00000000` | `0x00001000` | 4 KB |
| S140 7.3.0 SoftDevice | `0x00001000` | `0x00027000` | 152 KB |
| Main application / DFU bank 0 | `0x00027000` | `0x00088000` | `0x61000` / 388 KB |
| OTA staging area | `0x00088000` | `0x000E9000` | `0x61000` / 388 KB |
| OTA metadata page | `0x000E9000` | `0x000EA000` | 4 KB |

The main application layout is defined by
`/Users/steve/Dropbox/PC/Documents/S2J/fdr_display/software/Devicesfirmware/ld/app_bank0_grok.ld`:

```ld
FLASH (rx)  : ORIGIN = 0x27000, LENGTH = 0x61000
RAM   (rwx) : ORIGIN = 0x20006000, LENGTH = 0x3A000
```

The current Adafruit/Nordic DFU path should continue writing application updates
to `DFU_BANK_0_REGION_START`, which resolves to the main application start
`0x00027000`. That is correct for the existing DFU behavior.

Release linker script, `linker/nrf52840.ld`:

| Region | Start | End exclusive | Size |
| --- | ---: | ---: | ---: |
| Bootloader code | `0x000F4000` | `0x000FE000 - 2K` | 38 KB |
| Bootloader config | `0x000FE000 - 2K` | `0x000FE000` | 2 KB |
| MBR params page | `0x000FE000` | `0x000FF000` | 4 KB |
| Bootloader settings | `0x000FF000` | `0x00100000` | 4 KB |

Debug build uses `BOOTLOADER_REGION_START=0x000EA000` from the Makefile, while
the linker uses `linker/nrf52840_debug.ld`. That creates more room for debug
code and makes `src/main.c` place the custom bank config page at `0x000E9000`.
That debug layout matches the application linker script and leaves two equal
`0x61000` flash regions before the bootloader.

## Current Custom Bank Code

`src/main.c` currently has hard-coded bank constants:

| Name | Address/size |
| --- | ---: |
| `APP_BANK0_ADDR` | `0x00027000` |
| `APP_BANK1_ADDR` | `0x00088000` |
| `APP_BANK_SIZE` | `0x00061000` |
| `BANK_CFG_PAGE_ADDR` | `0x000E9000` in DEBUG build |

This matches the app linker layout where the main app is fixed in bank 0 and
the OTA staging area starts immediately after it.

The XIAO board headers now use the same layout:

| Name | Address/size |
| --- | ---: |
| `BANK0_ADDR` | `0x00027000` |
| `BANK_SIZE` | `0x00061000` |
| `BANK1_ADDR` | `0x00088000` |
| `BANK_SETTINGS_ADDR` | `0x000E9000` |

These constants are for the new DiffBus serial OTA path and related bootloader
metadata; they do not change the existing DFU write target.

## Required Dual-Bank OTA Design

The intended behavior is fixed main application plus OTA staging:

1. Normal boot jumps to the validated main application at `0x00027000`.
2. Existing DFU continues writing to bank 0 via `DFU_BANK_0_REGION_START`.
3. New serial OTA over DiffBus writes the incoming firmware to the OTA staging
   area at `0x00088000`.
4. The bootloader stores metadata for the pending image.
5. On reboot, the bootloader validates the pending image.
6. If validation passes, it erases/programs the main application bank from the
   storage bank, verifies the copy, updates metadata, and boots the new app.
7. If validation fails or copy is interrupted, it keeps or restores the last
   valid app.

Metadata should be stored in a dedicated flash page outside both banks and must
include at least:

- magic/version
- installed app address, normally `0x00027000`
- pending image address
- image size
- full image CRC
- page size and page count
- per-page CRC table or pointer to a CRC table
- state: empty, receiving, received, validated, copying, active, failed
- monotonically increasing sequence number or generation counter

CRC requirements:

- Per-page CRC validates individual 4 KB pages as they are received and before
  copying them into the main app area.
- Full-image CRC validates the entire candidate image after receive and validates
  the installed app before boot.

## Code Areas To Change Next

Primary files:

- `lib/sdk11/components/libraries/bootloader_dfu/dfu_types.h`
- `lib/sdk11/components/libraries/bootloader_dfu/dfu_single_bank.c`
- `lib/sdk11/components/libraries/bootloader_dfu/bootloader.c`
- `lib/sdk11/components/libraries/bootloader_dfu/bootloader_types.h`
- `src/main.c`
- `src/boards/xiao_nrf52840_ble/board.h`
- `src/boards/xiao_nrf52840_ble_sense/board.h`

Current blocker:

- The new DiffBus serial OTA writer does not exist in this bootloader yet. It
  should write the received image to `BANK1_ADDR` / `0x00088000`, not through
  the existing DFU data path.

Secondary blocker:

- `bootloader_app_is_valid()` validates only `DFU_BANK_0_REGION_START`, which is
  correct for the fixed main app. The new OTA install path should validate the
  staged image separately, copy it into bank 0, then update the existing bank-0
  validity metadata.
- `src/main.c` currently has experimental active-bank selection code that can
  try to boot `APP_BANK1_ADDR`. For the fixed-main/staging design, bank 1 should
  be treated as staging only unless a later design explicitly supports executing
  from either bank.

Open decision:

- BLE OTA may later be routed to the same staging area, but the existing DFU
  flow should remain bank-0 based until that is explicitly designed.
