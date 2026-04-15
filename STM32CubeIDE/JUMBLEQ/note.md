# JUMBLEQ USB Audio 開発メモ

## プロジェクト概要
- **MCU**: STM32H7S3Z8TX (Cortex-M7)
- **USB Stack**: TinyUSB (UAC2)
- **Audio Path**: STM32 ↔ ADAU1466 (DSP) ↔ AK4619 (Codec)
- **サンプルレート**: 48kHz / 96kHz
- **ビット深度**: 24bit

---

# オーディオレイテンシー見積もりメモ (2026-04)

## 対象範囲

この見積もりは、**STM32内部のUSB Audio FIFO <-> バッファ群 <-> SAI** の遅延を対象とする。

含まないもの:
- PC/DAW/ドライバ/ASIOバッファ遅延
- AK4619のADC/DAC群遅延
- ADAU1466のDSPアルゴリズム遅延
- USBホスト側スケジューリングの追加揺らぎ

---

## 見積もり時点の実装条件

- `SAI_RNG_BUF_SIZE = 8192`
- `SAI_TX_BUF_SIZE = 256`
- `SAI_RX_BUF_SIZE = 256`
- `SAI_TX_TARGET_LEVEL_WORDS = 96`
- `audioTask` は `ulTaskNotifyTake(..., 1ms)` で通知待ちしつつ、通知がなくても 1ms ごとに起床
- USB Audio は High-Speed, 4ch, 32bit slot, 48kHz / 96kHz
- USB IN は `audio_frames_per_ms()` により **1ms分まとめて送信**
- USB OUT/IN の TinyUSB SW FIFO はどちらも **約1ms分** (`8 * HS EP size`)
- USB OUT feedback は **FIFO中央水位**を狙う

参考コード:
- `Appli/Core/Inc/audio_control.h`
- `Appli/Core/Src/audio_control.c`
- `Appli/Core/Src/freertos.c`
- `Appli/Core/Inc/tusb_config.h`

---

## 現状の概算レイテンシー

### 1. 再生側 (USB OUT -> SAI OUT)

平均の目安:
- **48kHz: 約 2.3ms**
- **96kHz: 約 1.4ms**

主な内訳:
- USB OUT FIFO目標滞留: 約 `0.5ms`
- TXリング滞留平均:
  - 48kHz: 約 `0.83ms`
  - 96kHz: 約 `0.42ms`
- DMAに積まれてから実際に出るまでの平均:
  - 48kHz: 約 `1.0ms`
  - 96kHz: 約 `0.5ms`

### 2. 録音側 (SAI IN -> USB IN)

平均の目安:
- **48kHz: 約 1.3ms**
- **96kHz: 約 1.0ms**

主な内訳:
- SAI RX から USB送信塊が揃うまで:
  - 48kHz: 約 `0.84ms`
  - 96kHz: 約 `0.51ms`
- USB IN FIFO に 1ms塊を書いた後、ホストへ出るまでの平均: 約 `0.5ms`

### 3. STM32内部の往復 (概算)

- **48kHz: 約 3.7ms**
- **96kHz: 約 2.4ms**

備考:
- 現状は `audioTask` に 1ms タイムアウト起床があるため、平均値とは別に**最大ほぼ1msの追加待ち/ジッタ**が乗る可能性がある。
- したがって、体感差に効くのは平均値よりも**最悪値とジッタ**の方が大きい可能性がある。

---

## 改善余地の見積もり

### A. `SAI_TX_BUF_SIZE` / `SAI_RX_BUF_SIZE` を `256 -> 128` にする

期待できる効果:
- 再生側で約
  - **48kHz: 約 0.67ms短縮**
  - **96kHz: 約 0.33ms短縮**

補足:
- 録音側は現状 `1msまとめ送り` が支配的なため、これ単独では改善幅は比較的小さい。

### B. `SAI_TX_TARGET_LEVEL_WORDS` を下げる

例:
- `96 -> 64 words`
  - 48kHz: 約 `0.17ms` 短縮
  - 96kHz: 約 `0.08ms` 短縮
- `96 -> 32 words`
  - 48kHz: 約 `0.33ms` 短縮
  - 96kHz: 約 `0.17ms` 短縮

補足:
- 再生側専用の改善。
- 下げすぎると underrun 耐性が落ちる。

### C. `audioTask` を完全通知駆動にする

期待できる効果:
- **平均値の短縮は限定的**
- ただし **最大ほぼ1msの余計な待ち** を削れる可能性があり、**ジッタ/最悪値改善**には有効

補足:
- 単体で劇的に短くなるというより、DMA縮小や目標水位低減を安全に進めるための前提として重要。

### D. USB IN を「1msまとめ送り」から細粒度送信にする

期待できる効果:
- 録音側で約
  - **0.3ms から 0.5ms** 短縮見込み

補足:
- 往復でもその分だけ効きやすい。
- 現状の録音側はここが支配的。

### E. USB OUT FIFO目標水位を中央より低くする

期待できる効果:
- 再生側でさらに **約0.25ms前後** 短縮の余地あり

補足:
- 安定性悪化リスクあり。
- 実運用では慎重に詰める必要がある。

---

## 優先度付きの改善順

実装負荷と安全性を考えると、試す順番は以下が妥当。

1. `audioTask` を完全通知駆動に寄せる
2. `SAI_TX_BUF_SIZE` / `SAI_RX_BUF_SIZE` を `128` に下げる
3. `SAI_TX_TARGET_LEVEL_WORDS` を `64` まで下げて評価する
4. USB IN の `1msまとめ送り` をやめて、DMA half/full に近い単位で送る

---

## 現時点での結論

- STM32内部だけを見ると、レイテンシーは**楽器用途として致命的に大きい値ではない**
- 体感遅延により効きやすいのは、STM32内部最適化よりも
  - **PC側のオーディオバッファ設定**
  - **ドライバ/ホストアプリ設定**
  - **DSPアルゴリズム遅延**
  の方である可能性が高い

したがって、システム全体の改善優先度としては、
**まずホスト側バッファ設定を見直し、その後必要ならSTM32内部を詰める**
のが合理的。
```c
volatile uint32_t dbg_usb_isr_count = 0;       // USB ISR呼び出し回数
volatile uint32_t dbg_usb_isr_msp_min = 0xFFFFFFFF;  // ISR中のMSP最小値
volatile uint32_t dbg_usb_isr_msp_start = 0;   // ISR開始時のMSP
```

**freertos.c (vApplicationIdleHook):**
```c
extern size_t dbg_min_free_heap;  // FreeRTOSヒープ空き容量最小値
```
