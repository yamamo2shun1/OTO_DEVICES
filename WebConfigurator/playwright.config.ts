import { defineConfig, devices } from "@playwright/test";

const port = 3000;
const host = "localhost";

export default defineConfig({
  testDir: "./e2e",
  fullyParallel: true,
  forbidOnly: Boolean(process.env.CI),
  retries: process.env.CI ? 2 : 0,
  workers: process.env.CI ? 1 : undefined,
  reporter: process.env.CI ? "github" : "list",
  use: {
    baseURL: `http://${host}:${port}`,
    trace: "on-first-retry",
    screenshot: "only-on-failure",
  },
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
  webServer: {
    command: `npm run dev -- --hostname ${host} --port ${port}`,
    url: `http://${host}:${port}`,
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
});
