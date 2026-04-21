## Overview
<img width="517" height="126" alt="JUMBLEQ-logo" src="https://github.com/user-attachments/assets/0fec933e-3c29-45de-b90b-8d6eba264ed7" />  

###
“JUMBLEQ” is short for “jumbled equipment”.

JUMBLEQ is a one-of-a-kind stereo 2-channel digital audio mixer that removes the traditional input faders and crossfader found on DJ mixers, replacing them with multiple magnetic switches.

Although it is faderless, the magnetic switches behave like faders. JUMBLEQ also features a crossfader assign function, allowing a total of four stereo channels—two analog stereo inputs and two USB stereo inputs—to be freely assigned to A/B. In addition, the USB audio I/O can be used like a send/return path, enabling a USB host device to function as an external effects processor.

As a USB audio interface, JUMBLEQ supports UAC2 and provides 4-in/4-out audio I/O with 24-bit / 48 kHz / 96 kHz formats. Since it is USB class-compliant, no driver installation is required. It can be used immediately not only with Windows and macOS, but also with iPhone, iPad, and Android devices simply by connecting it.

## Gallery
- [Prototype4](./Gallery/prototype4.md)
- [Prototype3](./Gallery/prototype3.md)
- [Prototype2](./Gallery/prototype2.md)
- [Prototype1](./Gallery/prototype1.md)

## PCB Modules
- Original  
  - [STM32H7S3 Development Board](https://github.com/YamamotoWorksDev/STM32H7S3_Dev_Board)  
  - [AK4619 Audio Module](https://github.com/YamamotoWorksDev/AK4619_Module)  
  - [TAD5142 HP Module](https://github.com/YamamotoWorksDev/TAD5142_Module)
  - [MagneticSwitch & POT Module](https://github.com/YamamotoWorksDev/MagneticSwitch_and_POT_Module)
  - [JST PH2 Bus Module](https://github.com/YamamotoWorksDev/JST_PH2_BUS)

- Third party  
  - [0.96インチ 128×64ドット有機ELディスプレイ(OLED) 白色](https://akizukidenshi.com/catalog/g/g112031/)  
  - [SparkFun Qwiic OLED Display (0.91 in., 128x32)](https://www.sparkfun.com/sparkfun-qwiic-oled-display-0-91-in-128x32-lcd-24606.html)  
  - [SparkFun Qwiic EEPROM Breakout - 512Kbit](https://www.sparkfun.com/sparkfun-qwiic-eeprom-breakout-512kbit.html)

## Block Diagram
<img width="811" height="701" alt="JUMBLEQ drawio" src="https://github.com/user-attachments/assets/966f1396-498a-4aa9-8add-3a8f13c7d79a" />

## Audio Routing and Signal Flow Diagram
<img width="720" alt="audio_routing_dark(1)" src="https://github.com/user-attachments/assets/7b672a5b-e1c1-442d-823f-1a2c49538c81" />

## Audio Spec.
- Analog: 4in/4out+2out(Headphone)
- USB: 4in/4out(USB Audio Class 2.0) and MIDI in/out
- Format: 24bit, 48kHz/96kHz

## Electronic Spec.
- MCU: [STM32H7S3Z8](https://www.st.com/en/microcontrollers-microprocessors/stm32h7s3z8.html)
- DSP: [ADAU1466](https://www.analog.com/jp/products/adau1466.html)
- CODEC: [AK4619](https://www.akm.com/jp/ja/products/audio/audio-codec/ak4619vn/)
- IO: [Gateron Low Profile Magnet Jade Pro](https://www.gateron.com/products/gateron-full-pom-low-profile-magnetic-jade-pro-switch-set?srsltid=AfmBOopFkdCLZGMJNqQ-K2jzlEZM16yoIFbTWwkXe4nwWVk6TughK0Un&VariantsId=10870) x10 and 10 k POT x12

## Knob Assign
<img width="640" alt="image" src="https://github.com/user-attachments/assets/23123ed1-6aa1-4a9c-919b-3d1c9a353c36" />

## Magnetic SW Assign
<img width="640" alt="image" src="https://github.com/user-attachments/assets/2c029f36-cb75-4e7c-aaaf-ac73f4f2e775" />

## MIDI Spec.
- [RX](./Docs/MIDI_RX_SPEC.md)
- [TX](./Docs/MIDI_TX_SPEC.md)

JUMBLEQ also provides a configurator for changing settings via MIDI: [JUMBLEQ Configurator](https://github.com/yamamo2shun1/JUMBLEQ/tree/main/Max).  
JUMBLEQ Configurator is built with [Max 9](https://cycling74.com/products/max). The standalone version is available for Windows and macOS, but requires Max 9 to be installed. A [Max for Live device version](./Max/JUMBLEQ_Configurator.amxd) is also included, allowing JUMBLEQ settings to be changed directly from within Ableton Live.

<img width="600" alt="image" src="https://github.com/user-attachments/assets/12bdb115-7f15-489f-8334-a758076ccd83" />

## for Windows User
I’m developing JUMBLEQ as a solo project. I can’t realistically develop both an ASIO driver and the necessary firmware support on my own. If you’re using this device on Windows, please use [VB-Audio Matrix](https://vb-audio.com/Matrix/) or [ASIO4ALL](https://asio4all.org/)—even if you’d prefer a dedicated ASIO driver.

## Firmware Update
JUMBLEQ uses a UF2 bootloader.
When you connect the USB cable while holding down SW3, it is recognized as a USB flash drive, so you can update the firmware simply by copying app.uf2—created using the procedure below—to the drive.
```
> arm-none-eabi-objcopy -O binary JUMBLEQ_Appli.elf app.bin
> python uf2conv.py -c -b 0x90010000 -f STM32H7RS -o app.uf2 app.bin
```

## Mitigation for Errata 2.2.15
For the STM32H7S3xx erratum:

- [Errata 2.2.15 "I/O compensation could alter duty-cycle of high-frequency output signal"](https://www.st.com/resource/en/errata_sheet/es0596-stm32h7rxx7sxx-device-errata-stmicroelectronics.pdf)

the workaround has already been implemented in the boot-side code: [sbs.c](https://github.com/yamamo2shun1/JUMBLEQ/blob/main/STM32CubeIDE/MX25UW25645GXDI00_STM32H7S3Z8T/Boot/Core/Src/sbs.c)
