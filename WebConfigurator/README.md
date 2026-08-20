# JUMBLEQ Configurator

This Next.js Web app configures JUMBLEQ routing, crossfader, DVS, and magnetic-switch settings. It is designed for deployment with LOLIPOP! Deploy Now.

## Prerequisites

- Node.js `>=22.13.0`

## Quick Start

```bash
npm install
npm run dev
npm run build
npm start
```

The development server is available at `http://localhost:3000`.

## Validation

```bash
npm test
npm run test:e2e
```

`npm test` runs the MIDI protocol and preset-processing unit tests, linting, and a production build for deployment.

`npm run test:e2e` uses Chromium through Playwright and emulates a Web MIDI device to verify:

- Connection to JUMBLEQ and initial synchronization
- Setting changes, curve editing, and the EEPROM save command
- Automatic reconnection after USB disconnection
- Preset import and export

Install the browser used for E2E testing before running the tests for the first time:

```bash
npx playwright install chromium
```

Use `npm run test:all` to run all validation steps in sequence.

## Deploy Now

`next.config.ts` sets `output: "standalone"`, as required by Deploy Now. The `postbuild` script includes the public files and Next.js static assets in the standalone output. When deploying from the repository root, set the application root to `WebConfigurator`.

For deployment, follow the official LOLIPOP! instructions to log in through the CLI and deploy the project as a Next.js application.
