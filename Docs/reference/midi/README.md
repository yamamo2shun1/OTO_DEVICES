# MIDI Reference

JUMBLEQ provides USB MIDI input and output for remote control, configuration, and status reporting.

## Specifications

- [Receive specification](./receive.md) — Program Change and Control Change messages accepted by JUMBLEQ
- [Transmit specification](./transmit.md) — live control output and configuration-dump responses sent by JUMBLEQ

## JUMBLEQ Configurator

The [JUMBLEQ Configurator](../../user-guide/configurator.md) uses these messages to read and change device settings. It is developed with [Max 9](https://cycling74.com/products/max-9). Project files and packaged versions are available in the [`Max` directory](../../../Max/).

### Max Version

The Max version is available for Windows and macOS. Max 9 is required to open the source patch.

<img width="400" alt="JUMBLEQ Configurator for Max" src="https://github.com/user-attachments/assets/d568f29f-4c74-4a79-a0e7-26b0f5e07027" />

### Max for Live Version

The [Max for Live device](../../../Max/JUMBLEQ_Configurator.amxd) allows JUMBLEQ settings to be changed directly from Ableton Live.

<img width="600" alt="JUMBLEQ Configurator for Max for Live" src="https://github.com/user-attachments/assets/4fddf9b7-5925-4448-b21a-bae903f854b2" />
