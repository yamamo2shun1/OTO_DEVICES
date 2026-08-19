# MIDI Receive Specification

This document describes the MIDI receive specification based on the `JUMBLEQ/Appli/Core` implementation as of August 2026.

## 1. Overview

- Incoming MIDI messages are processed over USB MIDI.
- Incoming messages used to change UI settings are accepted only on **MIDI Ch. 15** (1-based channel numbering).

## 2. Program Change (Host -> Device)

### 2.1 Acceptance Conditions

- Program Change messages are accepted only on MIDI Ch. 15.
- Program Change messages on all other channels are ignored.

### 2.2 Program Change Map

| PC Number | Function |
|---:|---|
| 0/1 | Set the Ch. 1 input type (Line / Phono). |
| 2/3 | Set the Ch. 2 input type (Line / Phono). |
| 4/5/6/7 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to XFader A. |
| 8/9/10/11 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to XFader B. |
| 12/13/14/15 | Assign an input source (Ch. 1 / Ch. 2 / USB[1/2] / USB[3/4]) to Post XFader. |
| 16/17 | Disable or enable DVS mode for Ch. 1. |
| 18/19 | Disable or enable DVS mode for Ch. 2. |
| 20/21 | Select the Return source (USB[1/2] / USB[3/4]). |
| 22/23/24/25 | Select the headphone output source (XF_A / XF_B / THRU / MASTER). |
| 26/27 | Assign magnetic switch 2 to the auxiliary fade-down function for XFader A or B. |
| 28/29 | Assign magnetic switch 3 to the auxiliary fade-down function for XFader A or B. |
| 120/121 | Disable or enable crossfader cut-margin edit mode. |
| 122/123 | Select the output mode (CC / Note) for magnetic controls and XFADER sensors. |
| 126 | Transmit a dump of the parameters currently configured on the device. |
| 127 | Save the current settings to EEPROM. |

Notes:

- Auxiliary fade-down assignments for magnetic switches 2 and 3 take effect immediately when the Program Change message is received.
- The default assignments are XFader A for magnetic switch 2 and XFader B for magnetic switch 3.
- `PC127` saves the complete device configuration listed in Section 5 to EEPROM, including the DVS enable states, Return and headphone source assignments, magnetic output mode, auxiliary fade-down assignments, and XFader cut margins.
- `PC126` transmits the **current operating state (parameters eligible for saving)** rather than reading and transmitting the EEPROM contents.

## 3. Control Change (Host -> Device)

### 3.1 Acceptance Conditions

Control Change messages are accepted only when all of the following conditions are met:

- Crossfader cut-margin edit mode is enabled.
- The MIDI channel is Ch. 15.
- The CC number is supported (`20` or `21`).

### 3.2 Control Change Map

| CC Number | Parameter | Value Range |
|---:|---|---|
| 20 | `XFADE_CUT_MARGIN_A` | 0.02 to 0.60 |
| 21 | `XFADE_CUT_MARGIN_B` | 0.02 to 0.60 |

Conversion:

- CC values use inverted linear mapping: `0` maps to a cut margin of `0.60`, and `127` maps to `0.02`.
- The conversion is `margin = 0.60 + (0.02 - 0.60) * (CC / 127)`.
- A larger margin produces a wider, more gradual transition. A smaller margin produces a narrower, sharper cut.
- Default values are `0.60` for XFader A and `0.16` for XFader B.

## 4. `curve_edit_mode` (Cut-margin Edit Mode) Behavior

- Use Program Change messages to enable or disable the mode (`PC121` ON / `PC120` OFF).
- While enabled, the main OLED shows the cut-margin edit display.
- While disabled, the main OLED uses its normal display and shows the gain values for each input and output channel.
- There is no automatic timeout.

## 5. EEPROM Storage and Startup Restore

- Save trigger: `PC127` on Ch. 15
- Saved parameters:
  - CH1/CH2 input types
  - XF A/B/POST assignments
  - CH1/CH2 DVS enable states
  - Return source assignment
  - Headphone output source
  - Magnetic output mode (CC / Note)
  - Auxiliary fade-down assignments for magnetic switches 2 and 3
  - XFader cut margins A/B
- EEPROM record version: `0x0006`
- When loading at startup, XFader cut-margin values outside the valid range are clamped to `0.02..0.60` before being applied.

## 6. Implementation Notes

- Channel values in the implementation are zero-based (for example, `14` = MIDI Ch. 15).
- The Ch. 15 restrictions for incoming Program Change and Control Change messages are checked in their respective dispatch functions.
