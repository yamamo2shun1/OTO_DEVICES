import { defineConfig } from "astro/config";

export default defineConfig({
  site: "https://jumbleq.io",
  base: "/",
  output: "static",
  redirects: {
    "/en": "/",
  },
});
