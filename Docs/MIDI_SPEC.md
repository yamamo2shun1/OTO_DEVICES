# JUMBLEQ Firmware MIDI仕様

本ドキュメントは、`JUMBLEQ/Appli/Core` 実装に基づくMIDI仕様（2026-03時点）をまとめたものです。

## 1. 基本方針

- 受信MIDIは USB-MIDI 経由で処理する
- UI設定変更に使う受信MIDIは **MIDI Ch.15 固定**（1始まり表記）

## 2. 受信仕様（Host -> Device）

### 2.1 Program Change（Ch.15のみ有効）

Ch.15以外のPCは無視される。

| PC番号 | 機能 |
|---:|---|
| 0-19 | 既存の入力切替/アサイン/DVS制御（`adau1466.h` 定義） |
| 120 | `curve_edit_mode` OFF |
| 121 | `curve_edit_mode` ON |
| 127 | 現在設定をEEPROMへ保存 |

備考:
- `PC127` はルーティング設定に加え、XFADEカーブ指数 `A/B` も保存する。

### 2.2 Control Change（Ch.15かつ編集モードON時のみ有効）

以下をすべて満たした場合のみ受理する。

- `curve_edit_mode == ON`
- MIDIチャンネルが Ch.15
- CC番号が対象（20 or 21）

| CC番号 | パラメータ | 値範囲 |
|---:|---|---|
| 20 | `XFADE_CURVE_EXP_A` | 0.5 ～ 50.0 |
| 21 | `XFADE_CURVE_EXP_B` | 0.5 ～ 50.0 |

変換仕様:
- CC値 `0..127` を指数 `0.5..50.0` へ **対数マッピング**する
- 低い指数側/高い指数側の操作感を均すため、線形ではなく対数で変換する

## 3. curve_edit_modeの挙動

- ON/OFFはPCで切り替える（`PC121` ON / `PC120` OFF）
- ON中はメインOLEDにカーブ編集表示を出す
- OFF中は従来のメインOLED表示（各入出力Chのゲイン値表示）
- 自動タイムアウトは無し

## 4. EEPROM保存・起動時復元

- 保存トリガ: `PC127`（Ch.15）
- 保存対象:
  - CH1/CH2入力タイプ
  - XF A/B/POSTアサイン
  - CH1/CH2 DVS有効状態
  - `XFADE_CURVE_EXP_A/B`
- EEPROMレコードバージョン: `0x0003`
- 起動時読込時、`XFADE_CURVE_EXP_A/B` が範囲外なら `0.5..50.0` にクランプして適用する

## 5. 送信仕様（Device -> Host）

### 5.1 送信Control Change（MIDI Ch.1）

以下はデバイス側から送信される（実装上チャンネル値0 = Ch.1）。

- POT ch0-3: CC番号 `0..3`
- 磁気XFADER生値: CC番号 `10..15`（センサ変化量が閾値を超えたときのみ送信）

備考:
- 送信チャンネルは受信制御チャンネル（Ch.15）とは独立している。

## 6. 実装メモ

- 実装内チャンネル値は 0始まり（例: `14` = MIDI Ch.15）
- 受信PC/CCのCh.15制限はそれぞれのdispatch関数内で判定している
