# JUMBLEQ Firmware MIDI送信仕様

本ドキュメントは、`JUMBLEQ/Appli/Core` 実装に基づくMIDI送信仕様（2026-03時点）をまとめたものです。

## 1. 基本方針

- 送信MIDIは USB-MIDI 経由でHostへ出力する
- 送信チャンネルは受信制御チャンネル（Ch.15）とは独立している

## 2. 送信Control Change（Device -> Host）

以下はデバイス側から送信される（実装上チャンネル値0 = Ch.1）。

| 種別 | CC番号 | 備考 |
|---|---|---|
| POT ch0-3 | `0..3` | ノブ操作に応じて送信 |
| 磁気XFADER生値 | `10..15` | センサ変化量が閾値を超えたときのみ送信 |

## 3. 実装メモ

- 実装内チャンネル値は 0始まり（例: `0` = MIDI Ch.1）
- XFADER系CCは更新閾値を持ち、トラフィック/ジッタ抑制のため差分が小さい場合は送信しない

## 4. 設定ダンプ応答（PC126受信時）

`Host -> Device` で `PC126 (Ch.15)` を受信したとき、Deviceは現在設定中の保存対象パラメータを `Device -> Host` で送信する。

- 送信チャンネル: **MIDI Ch.15**（実装値14）
- 送信種別: Program Change + Control Change

### 4.1 Program Change（Device -> Host, Ch.15）

| PC値 | 意味 |
|---:|---|
| 0/1 | Ch.1入力タイプ（Line / Phono） |
| 2/3 | Ch.2入力タイプ（Line / Phono） |
| 4/5/6/7 | XFader Aソース（Ch.1 / Ch.2 / USB[1/2] / USB[3/4]） |
| 8/9/10/11 | XFader Bソース（Ch.1 / Ch.2 / USB[1/2] / USB[3/4]） |
| 12/13/14/15 | Post XFaderソース（Ch.1 / Ch.2 / USB[1/2] / USB[3/4]） |
| 16/17 | Ch.1 DVS（Off / On） |
| 18/19 | Ch.2 DVS（Off / On） |
| 20/21 | Returnソース（USB[1/2] / USB[3/4]） |
| 22/23/24/25 | HP出力ソース（XF_A / XF_B / THRU / MASTER） |
| 122/123 | 磁気スイッチ送信MIDI（CC / Note） |

### 4.2 Control Change（Device -> Host, Ch.15）

| CC値 | 意味 |
|---:|---|
| 20 | `XFADE_CURVE_EXP_A`（0.5..50.0 を 0..127 へ変換） |
| 21 | `XFADE_CURVE_EXP_B`（0.5..50.0 を 0..127 へ変換） |
