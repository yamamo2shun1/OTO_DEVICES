import { readFile } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import {
  clearMidiMessages,
  disconnectMockDevice,
  emitMockMidiMessage,
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

test("shows the verified iPad MIDIWeb Browser guidance", async ({ page }) => {
  await page.getByRole("button", { name: "Open help" }).click();

  const midiWebBrowserLink = page.getByRole("link", { name: /MIDIWeb Browser/ });
  await expect(midiWebBrowserLink).toHaveAttribute(
    "href",
    "https://apps.apple.com/jp/app/midiweb-browser/id6757226617?l=en-US",
  );
  await expect(page.getByText("Connection and MIDI communication are verified on iPadOS 26.5.")).toBeVisible();
});

test("connects to JUMBLEQ and reflects the complete initial sync", async ({ page }) => {
  await page.getByRole("button", { name: "Connect device" }).click();

  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();
  await expect(page.getByText("Current settings loaded from JUMBLEQ")).toBeVisible();
  await expect(page.getByText("14/14 synced")).toBeVisible();
  await expect(page.getByRole("group", { name: "Channel 2 input type" }).getByRole("button", { name: "PHONO" })).toHaveClass(/active/);
  await expect(page.getByRole("article", { name: "Channel 1 input" }).getByRole("switch", { name: "Channel 1 DVS" })).toHaveAttribute("aria-checked", "true");
  await expect(page.getByRole("combobox", { name: "Fader A", exact: true })).toHaveValue("USB 3/4");
  await expect(page.getByLabel("Headphone monitor source")).toHaveValue("Fader B");
  await expect(page.getByLabel("USB return input")).toHaveValue("USB 1/2");
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
  await page.getByRole("switch", { name: "Channel 1 DVS" }).click();
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

test("visualizes magnetic CC values and Note velocities on their hardware keys", async ({ page }) => {
  await page.getByRole("button", { name: "Connect device" }).click();
  await expect(page.getByRole("button", { name: "JUMBLEQ connected" })).toBeVisible();

  const topLeftMidi = page.getByTestId("magnetic-midi-0");
  await emitMockMidiMessage(page, [0x90, 68, 104]);
  await expect(topLeftMidi).toHaveClass(/is-live/);
  await expect(topLeftMidi.locator("small")).toHaveText("VEL 104");
  expect(await topLeftMidi.evaluate((element) => (
    Number.parseFloat((element as HTMLElement).style.getPropertyValue("--midi-level"))
  ))).toBeGreaterThan(80);

  await emitMockMidiMessage(page, [0x80, 68, 0]);
  await expect(topLeftMidi).not.toHaveClass(/is-live/);
  await expect(topLeftMidi.locator("small")).toHaveText("VEL 104");
  await expect(topLeftMidi.locator("small")).toHaveText("NOTE", { timeout: 2_000 });

  const bottomLeftMidi = page.locator(".key-bottom-midi-a");
  await emitMockMidiMessage(page, [0x90, 69, 80]);
  await expect(bottomLeftMidi).toHaveClass(/is-live/);
  await expect(bottomLeftMidi.locator("small")).toHaveText("VEL 080");
  await emitMockMidiMessage(page, [0x80, 69, 0]);

  const topRightMidi = page.locator(".key-top-midi-b");
  await emitMockMidiMessage(page, [0x90, 70, 72]);
  await expect(topRightMidi).toHaveClass(/is-live/);
  await expect(topRightMidi.locator("small")).toHaveText("VEL 072");
  await emitMockMidiMessage(page, [0x80, 70, 0]);

  await page.getByRole("button", { name: "Control change" }).click();
  await emitMockMidiMessage(page, [0xb0, 13, 80]);
  await expect(bottomLeftMidi).toHaveClass(/is-live/);
  await expect(bottomLeftMidi.locator("small")).toHaveText("CC 080");
  await emitMockMidiMessage(page, [0xb0, 13, 0]);

  await emitMockMidiMessage(page, [0xb0, 14, 72]);
  await expect(topRightMidi).toHaveClass(/is-live/);
  await expect(topRightMidi.locator("small")).toHaveText("CC 072");
  await emitMockMidiMessage(page, [0xb0, 14, 0]);

  const bottomRightMidi = page.getByTestId("magnetic-midi-3");
  await emitMockMidiMessage(page, [0xb0, 15, 96]);
  await expect(bottomRightMidi).toHaveClass(/is-live/);
  await expect(bottomRightMidi.locator("small")).toHaveText("CC 096");

  await emitMockMidiMessage(page, [0xb0, 15, 0]);
  await expect(bottomRightMidi).not.toHaveClass(/is-live/);
  await expect(bottomRightMidi.locator("small")).toHaveText("CC 000");
  await expect(bottomRightMidi.locator("small")).toHaveText("CC", { timeout: 2_000 });
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
