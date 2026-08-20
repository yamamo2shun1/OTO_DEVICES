import { readFile } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import {
  clearMidiMessages,
  disconnectMockDevice,
  installWebMidiMock,
  midiMessages,
  reconnectMockDevice,
} from "./web-midi-mock";

const importedPreset = {
  ch1Type: "PHONO",
  ch2Type: "LINE",
  assignA: "USB 1/2",
  assignB: "USB 3/4",
  assignPost: "CH 2",
  dvs1: true,
  dvs2: false,
  returnSource: "USB 1/2",
  headphoneSource: "Thru",
  sensor2: "B",
  sensor3: "A",
  magMode: "NOTE",
  curveA: 35,
  curveB: 65,
};

test.beforeEach(async ({ page }) => {
  await installWebMidiMock(page);
  await page.goto("/", { waitUntil: "networkidle" });
});

test("connects to JUMBLEQ and reflects the complete initial sync", async ({ page }) => {
  await page.getByRole("button", { name: "Connect device" }).click();

  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();
  await expect(page.getByText("Current settings loaded from JUMBLEQ")).toBeVisible();
  await expect(page.getByText("14/14 synced")).toBeVisible();
  await expect(page.getByRole("group", { name: "Channel 2 input type" }).getByRole("button", { name: "PHONO" })).toHaveClass(/active/);
  await expect(page.getByRole("combobox", { name: "Fader A", exact: true })).toHaveValue("USB 3/4");
  await expect(page.getByLabel("Headphone monitor source")).toHaveValue("Fader B");
  await expect(page.getByLabel("USB return input")).toHaveValue("USB 1/2");
  await expect(page.getByRole("switch").first()).toHaveAttribute("aria-checked", "true");
  await expect(page.getByRole("button", { name: "MIDI note" })).toHaveClass(/active/);
  await expect(page.getByLabel("Fader A curve sharpness")).toHaveValue("25");
  await expect(page.getByLabel("Fader B curve sharpness")).toHaveValue("75");
  expect(await midiMessages(page)).toContainEqual([0xce, 126]);
});

test("sends setting, curve edit, and EEPROM save messages", async ({ page }) => {
  await page.getByRole("button", { name: "Connect device" }).click();
  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();
  await clearMidiMessages(page);

  await page.getByRole("group", { name: "Channel 1 input type" }).getByRole("button", { name: "PHONO" }).click();
  await page.getByRole("combobox", { name: "Fader A", exact: true }).selectOption("USB 3/4");
  await page.getByRole("switch").first().click();
  const curveA = page.getByLabel("Fader A curve sharpness");
  await curveA.fill("80");
  await curveA.blur();
  await page.getByRole("button", { name: "Save to device" }).click();

  await expect(page.getByText("Save command sent to JUMBLEQ")).toBeVisible();
  expect(await midiMessages(page)).toEqual(expect.arrayContaining([
    [0xce, 1],
    [0xce, 7],
    [0xce, 16],
    [0xce, 121],
    [0xbe, 20, 102],
    [0xce, 120],
    [0xce, 127],
  ]));
});

test("automatically reconnects and synchronizes after a USB interruption", async ({ page }) => {
  await page.getByRole("button", { name: "Connect device" }).click();
  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();
  await clearMidiMessages(page);

  await disconnectMockDevice(page);
  await expect(page.getByRole("button", { name: "Reconnecting…" })).toBeVisible();
  await reconnectMockDevice(page);

  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();
  await expect(page.getByText("14/14 synced")).toBeVisible();
  expect(await midiMessages(page)).toContainEqual([0xce, 126]);
});

test("imports a validated preset and exports the same settings", async ({ page }) => {
  await page.getByLabel("Import preset file").setInputFiles({
    name: "test-preset.json",
    mimeType: "application/json",
    buffer: Buffer.from(JSON.stringify(importedPreset)),
  });

  await expect(page.getByText("Preset imported into the preview.")).toBeVisible();
  await expect(page.getByRole("group", { name: "Channel 1 input type" }).getByRole("button", { name: "PHONO" })).toHaveClass(/active/);
  await expect(page.getByRole("combobox", { name: "Fader A", exact: true })).toHaveValue("USB 1/2");
  await expect(page.getByLabel("Headphone monitor source")).toHaveValue("Thru");
  await expect(page.getByLabel("Fader A curve sharpness")).toHaveValue("35");
  await expect(page.getByLabel("Fader B curve sharpness")).toHaveValue("65");

  const downloadPromise = page.waitForEvent("download");
  await page.getByRole("button", { name: "Export preset" }).click();
  const download = await downloadPromise;
  expect(download.suggestedFilename()).toBe("jumbleq-preset.json");
  const path = await download.path();
  expect(path).not.toBeNull();
  expect(JSON.parse(await readFile(path!, "utf8"))).toEqual(importedPreset);
});
