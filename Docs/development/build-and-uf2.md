# Build and UF2 Generation

JUMBLEQ application projects convert their ELF output to BIN and UF2 files as a post-build step.

## STM32CubeIDE Post-build Command

When creating a new STM32CubeIDE application project, open:

`Project > Properties > C/C++ Build > Settings > Build Steps`

Enter the following command under `Post-build steps > Command`:

```sh
arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && python "../../../MX25UW25645GXDI00_STM32H7S3Z8T/uf2conv.py" -c -b 0x90010000 -f STM32H7RS -o "${BuildArtifactFileBaseName}.uf2" "${BuildArtifactFileBaseName}.bin"
```

This relative path assumes that the new project is located at the same directory depth as the existing application projects. Adjust the path to `uf2conv.py` if the project is placed elsewhere.

The generated `.bin` and `.uf2` files are written to the active build directory.

<img width="600" alt="STM32CubeIDE post-build command configuration" src="https://github.com/user-attachments/assets/52bb0bfe-e50b-4182-b115-3d1fe1585bd2" />

For instructions on installing the generated firmware, see [Firmware update](../user-guide/firmware-update.md).
