# JUMBLEQ Firmware MIDI受信仕様

本ドキュメントは、`JUMBLEQ/Appli/Core` 実装に基づくMIDI受信仕様（2026-03時点）をまとめたものです。

## 1. 基本方針

- 受信MIDIは USB-MIDI 経由で処理する
- UI設定変更に使う受信MIDIは **MIDI Ch.15 固定**（1始まり表記）

## 2. Program Change（Host -> Device）

### 2.1 受理条件

- MIDIチャンネルが Ch.15 のときのみ受理する
- Ch.15以外のPCは無視する

### 2.2 PCマップ

| PC番号 | 機能 |
|---:|---|
| 0 | Ch.1をLine入力に切り替え |
| 1 | Ch.1をPhono入力に切り替え |
| 2 | Ch.2をLine入力に切り替え |
| 3 | Ch.2をPhono入力に切り替え |
| 4 | XFader AにCh.1を割り当て |
| 5 | XFader AにCh.2を割り当て |
| 6 | XFader AにUSB[1/2]を割り当て |
| 7 | XFader AにUSB[3/4]を割り当て |
| 8 | XFader BにCh.1を割り当て |
| 9 | XFader BにCh.2を割り当て |
| 10 | XFader BにUSB[1/2]を割り当て |
| 11 | XFader BにUSB[3/4]を割り当て |
| 12 | Post XFaderにCh.1を割り当て |
| 13 | Post XFaderにCh.2を割り当て |
| 14 | Post XFaderにUSB[1/2]を割り当て |
| 15 | Post XFaderにUSB[3/4]を割り当て |
| 16 | Ch.1のDVSモードを無効化 |
| 17 | Ch.1のDVSモードを有効化 |
| 18 | Ch.2のDVSモードを無効化 |
| 19 | Ch.2のDVSモードを有効化 |
| 120 | フェーダー・カーブ調整モード オフ |
| 121 | フェーダー・カーブ調整モード オン |
| 127 | 現在設定をEEPROMへ保存 |

備考:
- `PC127` はルーティング設定に加え、XFADEカーブ指数 `A/B` も保存する。

## 3. Control Change（Host -> Device）

### 3.1 受理条件

以下をすべて満たした場合のみ受理する。

- フェーダー・カーブ調整モードオン中
- MIDIチャンネルが Ch.15
- CC番号が対象（20 or 21）

### 3.2 CCマップ

| CC番号 | パラメータ | 値範囲 |
|---:|---|---|
| 20 | `XFADE_CURVE_EXP_A` | 0.5 ～ 50.0 |
| 21 | `XFADE_CURVE_EXP_B` | 0.5 ～ 50.0 |

変換仕様:
- CC値 `0..127` を指数 `0.5..50.0` へ **対数マッピング**する
- 低い指数側/高い指数側の操作感を均すため、線形ではなく対数で変換する

## 4. curve_edit_modeの挙動

- ON/OFFはPCで切り替える（`PC121` ON / `PC120` OFF）
- ON中はメインOLEDにカーブ編集表示を出す
- OFF中は従来のメインOLED表示（各入出力Chのゲイン値表示）
- 自動タイムアウトは無し

## 5. EEPROM保存・起動時復元

- 保存トリガ: `PC127`（Ch.15）
- 保存対象:
  - CH1/CH2入力タイプ
  - XF A/B/POSTアサイン
  - CH1/CH2 DVS有効状態
  - `XFADE_CURVE_EXP_A/B`
- EEPROMレコードバージョン: `0x0003`
- 起動時読込時、`XFADE_CURVE_EXP_A/B` が範囲外なら `0.5..50.0` にクランプして適用する

## 6. 実装メモ

- 実装内チャンネル値は 0始まり（例: `14` = MIDI Ch.15）
- 受信PC/CCのCh.15制限はそれぞれのdispatch関数内で判定している
