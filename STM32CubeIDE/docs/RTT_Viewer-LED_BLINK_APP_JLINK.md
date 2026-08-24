# J-Link RTT Viewer Setup for `LED_BLINK_APP_JLINK`

This guide describes the J-Link RTT Viewer settings used to monitor the LED state messages from `LED_BLINK_APP_JLINK_Appli`.

## Prerequisites

- Build and program `LED_BLINK_APP_JLINK_Appli` with its J-Link Debug configuration.
- Keep the target powered and allow the application to run.
- Install SEGGER J-Link Software and Documentation Pack, including J-Link RTT Viewer.

## Connection Settings

Open J-Link RTT Viewer and use the following configuration.

<img width="360" alt="J-Link RTT Viewer configuration for LED_BLINK_APP_JLINK" src="https://github.com/user-attachments/assets/f49be479-0906-4995-a2e1-f010ded93999" />


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
| RTT Control Block address | `0x24071C00` |

Click **OK** to connect. Channel 0 displays messages similar to the following while the application is running:

```text
LED blink started
[0 ms] LED1 ON
[200 ms] LED2 ON
[400 ms] LED1 OFF
[600 ms] LED2 OFF
```

## Why a Fixed Address Is Used

The RTT control block and buffers are linked into the `RW_NONCACHEABLE` region beginning at `0x24071C00`. The application configures this 1 KiB region as non-cacheable before enabling the Cortex-M7 data cache, allowing both the target and J-Link to observe coherent RTT data.

If the RAM layout or linker script changes, confirm the address of `_SEGGER_RTT` in `Debug/LED_BLINK_APP_JLINK_Appli.map` before reconnecting RTT Viewer.

## Troubleshooting

- If no output appears, resume the application in STM32CubeIDE; RTT output stops while the CPU is halted.
- Confirm that the selected device is `STM32H7S3Z8`, the interface is `SWD`, and the control block address is exactly `0x24071C00`.
- If more than one J-Link is connected, enable **SN / Nickname** and select the intended probe.
- Rebuild the application after changing the linker script or RTT configuration.
