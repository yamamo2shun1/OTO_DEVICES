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

---

# HardFault修正履歴 #2 - DCache/FreeRTOSヒープ問題 (2025/01)

## 問題の症状

USB Audio再生中に**ランダムなHardFault**が発生。

- **発生タイミング**: 7分〜2時間以上と不規則
- **エラー種別**: UNDEFINSTR, INVSTATE, UNALIGNED など多様
- **特徴的なパターン**:
  - `stacked_pc`が無効なアドレス（0xFFFFFFF1 = EXC_RETURN）
  - レジスタに`0xA5A5A5A5`（FreeRTOSスタック初期化パターン）
  - IdleTask内でクラッシュ

## 調査経緯

### 1. 初期仮説 - DCache問題
DWC2 DMAとDCacheのコヒーレンシ問題を疑い、以下を実施：
- MPU Region 3（0x24040000-0x24080000）がNoncacheableであることを確認
- `xfer_status`, `_dcd_data`を`CFG_TUD_MEM_SECTION`でNoncacheable領域に配置
- → **改善せず**

### 2. DMA vs Slaveモード
- Slaveモード（`CFG_TUD_DWC2_DMA_ENABLE=0`）でテスト
- → **同様にクラッシュ**

### 3. DCache完全無効化テスト
- `SCB_DisableDCache()`で無効化
- → **まだクラッシュ** → DCacheは直接の原因ではない

### 4. 決定的な発見
クラッシュ時のレジスタに`0xA5A5A5A5`（FreeRTOSのスタック初期化パターン）が出現。
これは**FreeRTOSのTCB（Task Control Block）またはスタックが破壊されている**ことを示す。

## 根本原因

**FreeRTOSヒープ（ucHeap）がキャッシュ可能なRAM領域にあった。**

USB DMAがNoncacheable領域にアクセスする際、同じキャッシュラインを共有する
FreeRTOSヒープのデータが巻き込まれ、DCache eviction/refillのタイミングで破壊された。

### メモリレイアウト（修正前）
```
0x24000000 - 0x24040000: 通常RAM（キャッシュ可能）← ucHeap がここにあった
0x24040000 - 0x24080000: Noncacheable（MPU Region 3）← USB DMAバッファ
```

破壊パターン：
1. DMAがNoncacheable領域に書き込み
2. CPUがキャッシュラインをevict/refill
3. 隣接するucHeapのデータが古い値で上書き
4. FreeRTOSのTCB/スタックが破壊
5. タスクスケジューリング時にHardFault

## 解決策

### FreeRTOSヒープをNoncacheable領域に配置

**FreeRTOSConfig.h:**
```c
#define configAPPLICATION_ALLOCATED_HEAP         1
```

**freertos.c:**
```c
__attribute__((section("noncacheable_buffer"), aligned(8)))
uint8_t ucHeap[configTOTAL_HEAP_SIZE];
```

### TinyUSBバッファもNoncacheable領域に配置

**tusb_config.h:**
```c
#define CFG_TUD_MEM_SECTION   __attribute__((section("noncacheable_buffer"), aligned(32)))
#define CFG_TUD_MEM_ALIGN     TU_ATTR_ALIGNED(32)
#define IN_SW_BUF_MEM_ATTR    CFG_TUD_MEM_SECTION CFG_TUD_MEM_ALIGN
#define OUT_SW_BUF_MEM_ATTR   CFG_TUD_MEM_SECTION CFG_TUD_MEM_ALIGN
```

**audio_device.c:** (TinyUSBソース修正)
```c
#ifndef IN_SW_BUF_MEM_ATTR
  #if !CFG_TUD_EDPT_DEDICATED_HWFIFO
    #define IN_SW_BUF_MEM_ATTR TU_ATTR_ALIGNED(4)
  #else
    #define IN_SW_BUF_MEM_ATTR CFG_TUD_MEM_SECTION CFG_TUD_MEM_ALIGN
  #endif
#endif
// OUT_SW_BUF_MEM_ATTRも同様
```

**dcd_dwc2.c:** (TinyUSBソース修正)
```c
CFG_TUD_MEM_SECTION static xfer_ctl_t xfer_status[DWC2_EP_MAX][2];
CFG_TUD_MEM_SECTION static dcd_data_t _dcd_data;
```

## Noncacheable領域の最終配置

mapファイルから確認：
```
アドレス         サイズ    内容
0x24040000                noncacheable_buffer開始
0x24056420      32KB      ucHeap (FreeRTOSヒープ)
0x2405E640      16KB      audio_device.o (ep_in/out_sw_buf, lin_buf_in/out)
0x24062700      1KB       midi_device.o
0x24062B00      64B       usbd_control.o
0x24062B40      352B      dcd_dwc2.o (xfer_status, _dcd_data)
0x24062CA0                __NONCACHEABLEBUFFER_END

使用量: 約140KB / 256KB
```

## デバッグ用モニタリング変数

**stm32h7rsxx_it.c:**
```c
volatile uint32_t dbg_usb_isr_count = 0;       // USB ISR呼び出し回数
volatile uint32_t dbg_usb_isr_msp_min = 0xFFFFFFFF;  // ISR中のMSP最小値
volatile uint32_t dbg_usb_isr_msp_start = 0;   // ISR開始時のMSP
```

**freertos.c (vApplicationIdleHook):**
```c
extern size_t dbg_min_free_heap;  // FreeRTOSヒープ空き容量最小値
```

## 検証結果

- **12時間以上の連続動作でHardFault発生なし** ✅
- ヒープ使用量安定（約8KB使用、24KB空き）
- ISRスタック使用量安定（約1KB使用）

## 学んだ教訓

1. **DCache問題は間接的に他の領域を破壊しうる**
   - DMAバッファだけでなく、同じキャッシュラインを共有する可能性のある重要なデータ（ヒープ、TCB）もNoncacheable領域に置く

2. **0xA5A5A5A5パターンはFreeRTOS破壊の証拠**
   - FreeRTOSはスタックを`0xA5`で初期化する（`configCHECK_FOR_STACK_OVERFLOW`有効時）
   - このパターンがレジスタに現れたら、TCB/スタック破壊を疑う

3. **DCacheを無効化してもDMA問題は残る**
   - DCache無効化はキャッシュコヒーレンシ問題の一部を解消するが、DMAと CPUの同時アクセス競合は別問題

## 修正ファイル一覧

1. `Appli/Core/Inc/FreeRTOSConfig.h` - `configAPPLICATION_ALLOCATED_HEAP=1`追加
2. `Appli/Core/Src/freertos.c` - `ucHeap`をNoncacheable領域に定義
3. `Appli/Core/Inc/tusb_config.h` - バッファ属性マクロ追加
4. `Appli/tinyusb/src/class/audio/audio_device.c` - `#ifndef`ガード追加
5. `Appli/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c` - `CFG_TUD_MEM_SECTION`追加
6. `Appli/Core/Src/stm32h7rsxx_it.c` - ISRモニタリング追加、HardFaultInfo拡張
7. `Appli/Core/Src/usb_otg.c` - USB割り込み優先度を6→5に変更
