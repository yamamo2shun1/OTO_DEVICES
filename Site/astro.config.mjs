import { defineConfig } from "astro/config";
import sitemap from "@astrojs/sitemap";

export default defineConfig({
  site: "https://jumbleq.io",
  base: "/",
  output: "static",
  integrations: [sitemap()],
  redirects: {
    "/en": "/",
  },
});
