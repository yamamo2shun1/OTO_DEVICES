# JUMBLEQ Configurator

JUMBLEQのルーティング、クロスフェーダー、DVS、磁気スイッチ設定を行う
Next.js Webアプリです。ロリポップ！デプロイナウでの公開を前提にしています。

## Prerequisites

- Node.js `>=22.13.0`

## Quick Start

```bash
npm install
npm run dev
npm run build
npm start
```

開発画面は `http://localhost:3000` で開きます。

## Validation

```bash
npm test
```

lintとデプロイ用の本番ビルドをまとめて確認します。

## Deploy Now

`next.config.ts`では、デプロイナウが必要とする`output: "standalone"`を設定済みです。
`postbuild`で公開ファイルとNext.jsの静的アセットをstandalone出力へ同梱します。
リポジトリ直下から公開する場合、アプリのルートは`React`を指定します。

公開時はロリポップ！公式の手順に従ってCLIへログインし、Next.jsとしてデプロイします。
