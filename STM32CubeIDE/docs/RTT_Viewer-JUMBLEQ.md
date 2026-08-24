# J-Link RTT Viewer Setup for `JUMBLEQ`

This guide describes the J-Link RTT Viewer settings used to monitor debug messages from `JUMBLEQ_Appli`.

## Prerequisites

- Build and program `JUMBLEQ_Appli` with `JUMBLEQ_Appli Debug (JLINK)`.
- Keep the target powered and allow the application to run.
- Install SEGGER J-Link Software and Documentation Pack, including J-Link RTT Viewer.

## Connection Settings

Open J-Link RTT Viewer and use the following configuration.

<img width="360" alt="J-Link RTT Viewer configuration for JUMBLEQ" src="https://github.com/user-attachments/assets/a6d018f1-82e3-4e46-82cc-02af67bd21b2" />

| Setting | Value |
| --- | --- |
| Connection to J-Link | `USB` |
| SN / Nickname | Disabled unless selecting a specific probe |
| Target Device | `STM32H7S3Z8` |
| Force go on connect | Disabled |
| Script file | Leave empty |
| Target Interface | `SWD` |
| Speed | `4000 kHz` |
| RTT Control Block | `Address` |
| RTT Control Block address | `0x20004200` |

Click **OK** to connect. Channel 0 displays firmware diagnostics while the application is running. The exact messages depend on the connected hardware and runtime activity. For example:

```text
EEPROM CAT24C512 detected on I2C2 (0x50)
[USB] loop=... ready=... idle=... lag=... max_gap=...
```

## Control Block Address

For the current Debug build, select **Address** and enter:

```text
0x20004200
```

In the current Debug build, `_SEGGER_RTT` occupies `0x4B8` bytes in the `.rtt` section starting at `0x20004200`. The linker places this section in DTCM, which is not affected by the Cortex-M7 data cache.

Unlike the fixed address used by `LED_BLINK_APP_JLINK`, the `JUMBLEQ_Appli` address follows the heap and stack allocation and can move when the linker layout changes. After such a change, use **Auto Detection** or confirm `_SEGGER_RTT` in `Debug/JUMBLEQ_Appli.map` and update the address.

## Troubleshooting

- If no output appears, resume the application in STM32CubeIDE; RTT output stops while the CPU is halted.
- Confirm that the selected device is `STM32H7S3Z8` and the interface is `SWD`.
- If the configured address no longer works after a layout change, try **Auto Detection** and check the map file.
- If more than one J-Link is connected, enable **SN / Nickname** and select the intended probe.
- Rebuild the application after changing its linker script, heap size, stack size, or RTT configuration.
