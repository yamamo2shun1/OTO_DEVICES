# JUMBLEQ USB Audio 開発メモ

## プロジェクト概要
- **MCU**: STM32H7S3Z8TX (Cortex-M7)
- **USB Stack**: TinyUSB (UAC1/UAC2)
- **Audio Path**: STM32 ↔ ADAU1466 (DSP) ↔ AK4619 (Codec)
- **サンプルレート**: 48kHz / 96kHz
- **ビット深度**: 16bit / 24bit

---

## 完了した作業

### 1. USB Audio 4ch拡張 (2in/2out → 4in/4out)
- `tusb_config.h`: `CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX/RX = 4`
- `usb_descriptors.h`: UAC1/UAC2の4ch対応ディスクリプタ
- `audio_control.h`: `SAI_TX_BUF_SIZE = 2048` (4ch用に倍増)

### 2. SAI TX 4ch拡張
- CubeMXで SAI A (TX): Frame Length 128bits, Number of Slots 4
- `copybuf_usb2ring()`: USB 4ch → リングバッファ（直接コピー）

### 3. SAI RX 2ch → USB 4ch変換
- `copybuf_ring2usb_and_send()`: SAI 2ch → USB 4ch (ch3/4は無音)
- 16bit/24bit両対応

### 4. スタックサイズ増加
- `STM32H7S3Z8TX_ROMxspi1.ld`: `_Min_Stack_Size = 0x4000` (4KB → 16KB)

---

## HardFault修正履歴

### 2024-12: 入出力同時使用時のHardFault (PRECISERR)

**症状**: Ableton Liveで入出力同時処理時にHardFault発生

**原因**: `tud_audio_tx_done_isr` (USB TX完了ISR) 内で `tud_audio_write` を呼び出すと、
同時進行中のUSB RX処理 (`tud_audio_read`) とTinyUSB内部FIFOで競合が発生。
FIFOの `wrap_count` が異常値 (65280) になり、`memcpy` でアクセス違反。

**コールスタック**:
```
HardFault_Handler
 └── memcpy
      └── _ff_push_n (tusb_fifo.c)
           └── tu_fifo_write_n_access_mode
                └── tud_audio_n_write
                     └── tud_audio_write
                          └── copybuf_ring2usb_and_send
                               └── tud_audio_tx_done_isr
```

**解決策**: ISRではフラグのみ設定し、タスクコンテキストからUSB送信を行う

```c
// audio_control.c
static volatile bool usb_tx_pending = false;

// ISRではフラグを立てるだけ
bool tud_audio_tx_done_isr(...)
{
    usb_tx_pending = true;
    return true;
}

// audio_task() 内でUSB送信
if (usb_tx_pending || s_streaming_in)
{
    usb_tx_pending = false;
    copybuf_ring2usb_and_send();
}
```

**結果**: 6時間連続動作でHardFault発生なし ✅

---

## 動作フロー（現在）

```
[FreeRTOSタスク - audio_task 1ms周期]
  tud_audio_read → copybuf_usb2ring → copybuf_ring2sai → SAI TX
  copybuf_sai2ring
  copybuf_ring2usb_and_send (tud_audio_write) → USB IN

[USB ISR - tud_audio_tx_done_isr]
  usb_tx_pending = true;  ← フラグのみ
```

---

## バッファ構成
```
SAI_RNG_BUF_SIZE = 8192  (リングバッファ)
SAI_TX_BUF_SIZE  = 2048  (SAI TX DMAバッファ, 4ch)
SAI_RX_BUF_SIZE  = 1024  (SAI RX DMAバッファ, 2ch)
```

---

## 重要な注意点
1. **ASIO4ALLはノイズ発生** - FlexASIOを推奨
2. **SAI RXデータは24bit左詰め** (下位8bitは0x00)
3. **USB READ/WRITEは同一タスクコンテキストから行う** (FIFO競合回避)
4. **TinyUSBのFIFOは完全にスレッドセーフではない**

---

## デバッグ用カウンタ
- `dbg_tx_half_count`, `dbg_tx_cplt_count`: SAI TX DMA割り込み回数
- `dbg_fill_tx_count`, `dbg_fill_underrun`, `dbg_fill_copied`: TX処理状態
- `dbg_ret_not_mounted`, `dbg_ret_not_streaming`, `dbg_ret_underrun`, `dbg_ret_written_zero`: 早期リターン原因

---

## 主な変更ファイル
1. `Appli/Core/Inc/tusb_config.h`
2. `Appli/Core/Inc/usb_descriptors.h`
3. `Appli/Core/Inc/audio_control.h`
4. `Appli/Core/Src/audio_control.c`
5. `Appli/STM32H7S3Z8TX_ROMxspi1.ld`
