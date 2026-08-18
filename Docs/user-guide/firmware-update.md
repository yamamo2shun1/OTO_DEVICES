# Firmware Update

JUMBLEQ includes a built-in UF2 bootloader, so application firmware can be updated without a dedicated programming tool.

## Enter UF2 Bootloader Mode

### Starting from Power Off

1. Make sure JUMBLEQ is powered off.
2. Hold down SW3.
3. While continuing to hold SW3, connect the USB cable.
4. Release SW3 after JUMBLEQ appears as a USB flash drive.

### While JUMBLEQ Is Running

1. Hold down SW3.
2. While continuing to hold SW3, press and release the RESET button.
3. Release SW3 after JUMBLEQ appears as a USB flash drive.

## Install Firmware

Drag and drop the JUMBLEQ `.uf2` firmware file onto the USB drive. The firmware update completes when the file copy finishes.

For information about generating UF2 files from an STM32CubeIDE project, see [Build and UF2 generation](../development/build-and-uf2.md).
