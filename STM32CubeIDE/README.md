# STM32CubeIDE projects

JUMBLEQハードウェア向けのファームウェアを管理するSTM32CubeIDEワークスペースです。
いずれもカスタム基板上の **STM32H7S3Z8T6** を対象としています。

## プロジェクト一覧

### `JUMBLEQ`

JUMBLEQ本体のメインファームウェアです。

- FreeRTOS上でUSB、オーディオ、LED、ADC、OLEDの処理を実行
- TinyUSBを使用したUSB Audio Class 2.0およびUSB MIDI
- AK4619オーディオコーデックとADAU1466 DSPの制御
- OLED、LED、各種入力によるユーザーインターフェース
- EEPROMへの設定保存と復元

ビルド対象のSTM32CubeIDEプロジェクトは `JUMBLEQ/Appli` にあります。
アプリケーションは外部フラッシュの `0x90010000` から実行する構成で、ビルド後にBINファイルとUF2ファイルを生成します。

### `LED_BLINK_APP`

基板の基本動作と外部フラッシュからのアプリ起動を確認するための最小テストアプリです。
LED1とLED2を500 ms間隔で順番に点灯・消灯します。

ビルド対象のSTM32CubeIDEプロジェクトは `LED_BLINK_APP/Appli` にあります。
`JUMBLEQ` と同じく外部フラッシュの `0x90010000` から実行する構成で、ビルド後にUF2ファイルを生成します。

### `MX25UW25645GXDI00_STM32H7S3Z8T`

外部フラッシュ **MX25UW25645GXDI00** を扱うためのブートローダおよび外部メモリローダです。

- `Boot`: USB MSC経由でUF2ファイルを書き込むブートローダ
  - SW3を押しながら起動するとUF2書き込みモードに入る
  - 通常起動時は外部フラッシュ上のアプリケーションへジャンプする
- `ExtMemLoader`: STM32CubeProgrammerやSTM32CubeIDEから外部フラッシュへアクセスするためのローダ
- `uf2conv.py`: BINファイルをUF2形式へ変換するツール

ブートローダの詳細は [`docs/UF2_BOOTLOADER_SPEC.md`](MX25UW25645GXDI00_STM32H7S3Z8T/docs/UF2_BOOTLOADER_SPEC.md) を参照してください。

## プロジェクト間の関係

`MX25UW25645GXDI00_STM32H7S3Z8T/Boot` が起動およびファームウェア更新を担当し、外部フラッシュに書き込まれた `JUMBLEQ/Appli` または `LED_BLINK_APP/Appli` を実行します。

