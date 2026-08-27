# JUMBLEQ Public Website

This directory contains the Astro-based public website for JUMBLEQ.

## Development

Requires Node.js 24 and pnpm 11.

```bash
pnpm install
pnpm dev
pnpm build
```

The production build is written to `dist/`. The repository Pages workflow uses Astro's official GitHub Action to build and publish the site.

`astro.config.mjs` configures the GitHub Pages origin and the `/JUMBLEQ` repository base path. Remove `base` and update `site` if the website moves to a custom domain.
