# UF2 Bootloader Specification (MX25UW25645GXDI00_STM32H7S3Z8T)

Last updated: 2026-08-17

## Overview

- Target project: `MX25UW25645GXDI00_STM32H7S3Z8T`
- The current system uses a simple configuration that starts an application from external flash using execute-in-place (XIP).
- The UF2 bootloader adds firmware updates over USB Mass Storage Class (MSC).

## Confirmed Requirements

- Enter bootloader mode: power on the device by connecting USB while holding SW3
- USB mode: High Speed (HS)
- Image validation: CRC32 only
- Application location: fixed offset in external flash
- Volume label: `UF2BOOT`
- USB VID/PID: `0x31BF` / `0x0100`

## Purpose

- Receive a UF2 file over USB MSC and write it to the application region in external flash.
- Validate the application using CRC32, then start it using XIP.

## Startup Flow

1. Perform low-level initialization for the clock, MPU, and caches.
2. Initialize XSPI and ExtMem Manager.
3. Select the boot mode.
   - Enter UF2 mode when SW3 is held and USB is connected.
   - Otherwise, start the application.
4. In UF2 mode:
   - Start the USB MSC device.
   - Receive UF2 blocks and write them to external flash.
   - Update the metadata after successful CRC32 validation.
5. In normal mode:
   - Jump to the XIP application.

## Memory Map

- Internal flash
  - Bootloader
- External flash (32 MB = `0x0200_0000`)
  - META region: `0x9000_0000` - `0x9000_FFFF` (64 KB)
  - APP region: `0x9001_0000` - `0x9040_FFFF` (4 MB)
  - Reserved: `0x9041_0000` - `0x91FF_FFFF` (remaining space)

### Fixed Offsets

- `APP_OFFSET = 0x0001_0000` (64 KB)
- `APP_BASE = 0x9000_0000 + APP_OFFSET = 0x9001_0000`
- Set `EXTMEM_XIP_IMAGE_OFFSET` equal to `APP_OFFSET`.
- Set `EXTMEM_HEADER_OFFSET` to `0`.

The META region could be as small as 4 KB, but 64 KB is reserved to align it with an erase-block boundary.

## Erase and Write Strategy

- The device supports 4 KB sector erase and 64 KB block erase operations.
- Erase the APP region in 64 KB blocks to reduce update time.
- Allow 4 KB sector erase operations for small regions such as META.
- When receiving a UF2 file, pre-erase only the 64 KB blocks that correspond to the target address range.

## UF2 Processing

- Receive 512-byte UF2 blocks over USB MSC.
- Map each `targetAddr` to the APP region in external flash.
- Write each block as it is received.
- Calculate CRC32 after all blocks have been received.
- Update META after successful CRC32 validation.

## UF2 MSC Virtual Disk Layout

- Purpose: provide enough capacity to transfer a UF2 file of approximately 8 MB for a 4 MB application
- Disk size: 16 MB (16,384 KB)
- Sector size: 512 bytes
- Cluster size: 1 sector
- File system: FAT16
- Root directory entries: 64
- Volume label: `UF2BOOT`

### Generated Read-only Files

- `INFO_UF2.TXT`
  - Board name, build information, supported address range, and related information
- `INDEX.HTM`
  - Optional link to the product page or update instructions
- `CURRENT.UF2` (optional)
  - Used to read back the current application; omit it if this feature is not required

### Write Acceptance Rules

- Accept only blocks with valid UF2 magic values.
- Write only blocks whose `targetAddr` is within `APP_BASE .. APP_BASE + APP_SIZE - 1`.
- Ignore addresses outside this range and ordinary file-system writes.
- On the first write to an address, erase only the corresponding 64 KB block.
- Use `numBlocks` and `blockNo` to determine when reception is complete, then perform CRC32 validation.

## UF2 Generation and Installation

### Prerequisites

- Application link address: `0x90010000` (`APP_BASE`)
- Use `uf2conv.py` for UF2 conversion.
- Place `uf2families.json` in the same directory as `uf2conv.py`.
- Family ID: `STM32H7RS = 0x6db66083`

### 1. Convert ELF to BIN
```
arm-none-eabi-objcopy -O binary app.elf app.bin
```

### 2. Convert BIN to UF2
```
python uf2conv.py -c -b 0x90010000 -f STM32H7RS -o app.uf2 app.bin
```

### 3. Install the Firmware

1. Hold SW3 while powering on the device by connecting USB.
2. Wait for the `UF2BOOT` volume to mount on the host computer.
3. Drag and drop `app.uf2` onto `UF2BOOT`.
4. After the update completes, the bootloader automatically jumps to the application.

## CRC32 Metadata Specification

### Location

- META base address: `0x9000_0000`
- The first 256 bytes are used for metadata.

### Format (Little-endian)
```
typedef struct {
  uint32_t magic;       // 0x4D544131 ('MTA1')
  uint16_t version;     // 0x0001
  uint16_t header_size; // sizeof(MetaHeader) = 64
  uint32_t flags;       // bit0: valid
  uint32_t app_base;    // 0x90010000
  uint32_t app_size;    // max 0x00400000
  uint32_t app_crc32;   // CRC32 of the entire application
  uint32_t build_id;    // Optional build number or short Git hash
  uint32_t image_size;  // Number of bytes actually written
  uint32_t reserved[6]; // 0
} MetaHeader;
```

### Update Timing

- Calculate CRC32 after the complete UF2 image has been received.
- Update META with `flags.valid = 1` only if CRC32 validation succeeds.
- To update META, erase its 4 KB sector and write the first 256 bytes.

### Validation at Startup

- Check `magic`, `version`, and `header_size` at startup.
- If `flags.valid == 1` and `app_size` is within the maximum size, validate CRC32.
- Start the application using XIP when validation succeeds; enter UF2 mode when validation fails.

## LED Status Patterns

- Normal startup (jump to XIP application): turn on LED0 for 1 second, then turn it off.
- Waiting for UF2 (update mode entered with SW3): blink LED0 slowly at 1 Hz.
- Writing UF2: blink LED1 quickly at 4 Hz.
- Validating CRC32: breathe LED2 at 0.5 Hz.
- Validation successful / preparing to start: turn on LED0, LED1, and LED2 together for 200 ms, then turn them off.
- Validation failed / error: blink LED2 twice for 100 ms each, repeating every second.
- SW3 held without a USB connection (invalid): blink LED1 briefly three times.

## XIP Startup Strategy
- `VTOR = APP_BASE`
- `MSP = *(uint32_t*)APP_BASE`
- `PC  = *(uint32_t*)(APP_BASE + 4)`
- Clean and invalidate the caches before jumping to the application.

## Current Boot Implementation Review (2026-02-12)

- `BOOT_Application()` is implemented in `stm32_boot_xip.c`.
- `MapMemory()` calls `EXTMEM_MemoryMappedMode()` for every memory in `extmem_list_config`.
- If `EXTMEM_MEMORY_BOOTXIP` is unsupported, the bootloader reports `BOOT_ERROR_INCOMPATIBLEMEMORY`.
- `JumpToApplication()` performs the following operations in order:
  - Calls `EXTMEM_GetMapAddress(EXTMEM_MEMORY_BOOTXIP, &Application_vector)`.
  - Adds `EXTMEM_XIP_IMAGE_OFFSET` and `EXTMEM_HEADER_OFFSET`; both are currently undefined and therefore default to `0`.
  - Stops SysTick, I-Cache, and D-Cache.
  - Sets `VTOR`, sets `MSP`, and jumps to `PC`.
- `Boot/Core/Src/main.c` initializes the following components in order:
  - `MX_XSPI1_Init()`
  - `MX_EXTMEM_MANAGER_Init()`
  - `BOOT_Application()`
- The MPU configures the 32 MB region at `0x90000000` as executable and cacheable.
- The linker reserves 64 KB of internal flash starting at `0x08000000` for the bootloader.
- `extmem_list_config` is initialized for NOR SFDP, XSPI1, and 8-line mode.

## Remaining Work

1. Complete the boot-mode selection, USB MSC, UF2 writing, META update, and LED control implementation.

## Implementation Status (2026-02-12)

- Implemented the basic structure for boot-mode selection, UF2 parsing, external flash writes, META updates, and LED control.
- USB MSC operates in High Speed mode using TinyUSB.
- MSC presents a minimal 16 MB FAT16 virtual disk with read-only generated files.
- Confirmed CRC32 validation and the automatic jump to the application after a UF2 update.
