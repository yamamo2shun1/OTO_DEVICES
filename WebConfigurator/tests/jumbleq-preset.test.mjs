import assert from "node:assert/strict";
import test from "node:test";

import { RESTORE_DEFAULT_CONFIG } from "../app/midi/jumbleq-midi.ts";
import { parseJumbleqPreset, serializeJumbleqPreset } from "../app/presets/jumbleq-preset.ts";

const validPreset = {
  ch1Type: "PHONO",
  ch2Type: "LINE",
  assignA: "USB 1/2",
  assignB: "CH 2",
  assignPost: "USB 3/4",
  dvs1: true,
  dvs2: false,
  returnSource: "USB 1/2",
  headphoneSource: "Fader B",
  sensor2: "B",
  sensor3: "A",
  magMode: "NOTE",
  curveA: 0,
  curveB: 100,
};

test("current flat preset format serializes and parses without data loss", () => {
  assert.deepEqual(parseJumbleqPreset(serializeJumbleqPreset(validPreset)), validPreset);
  assert.deepEqual(parseJumbleqPreset(serializeJumbleqPreset(RESTORE_DEFAULT_CONFIG)), RESTORE_DEFAULT_CONFIG);
});

test("parser returns only supported fields", () => {
  const imported = parseJumbleqPreset(JSON.stringify({
    ...validPreset,
    futureField: "ignored",
    __proto__: "ignored",
  }));

  assert.deepEqual(imported, validPreset);
  assert.equal(Object.hasOwn(imported, "futureField"), false);
});

test("parser rejects malformed JSON and non-object roots", () => {
  assert.throws(() => parseJumbleqPreset("{"), /not valid JSON/);
  assert.throws(() => parseJumbleqPreset("null"), /must be a JSON object/);
  assert.throws(() => parseJumbleqPreset("[]"), /must be a JSON object/);
  assert.throws(() => parseJumbleqPreset('"preset"'), /must be a JSON object/);
});

test("parser rejects every missing required field", () => {
  for (const field of Object.keys(validPreset)) {
    const incomplete = { ...validPreset };
    delete incomplete[field];
    assert.throws(() => parseJumbleqPreset(JSON.stringify(incomplete)), new RegExp(field), field);
  }
});

test("parser rejects unsupported enum values", () => {
  const invalidValues = {
    ch1Type: "MIC",
    ch2Type: "MIC",
    assignA: "Bluetooth",
    assignB: "Bluetooth",
    assignPost: "Bluetooth",
    returnSource: "USB 5/6",
    headphoneSource: "Return",
    sensor2: "C",
    sensor3: "C",
    magMode: "POLY",
  };

  for (const [field, value] of Object.entries(invalidValues)) {
    assert.throws(
      () => parseJumbleqPreset(JSON.stringify({ ...validPreset, [field]: value })),
      new RegExp(field),
      field,
    );
  }
});

test("parser requires real boolean DVS values", () => {
  assert.throws(() => parseJumbleqPreset(JSON.stringify({ ...validPreset, dvs1: 1 })), /dvs1/);
  assert.throws(() => parseJumbleqPreset(JSON.stringify({ ...validPreset, dvs2: "false" })), /dvs2/);
});

test("parser accepts only integer curve percentages from 0 through 100", () => {
  for (const value of [-1, 100.1, 101, "50", null]) {
    assert.throws(
      () => parseJumbleqPreset(JSON.stringify({ ...validPreset, curveA: value })),
      /curveA/,
      String(value),
    );
  }

  assert.equal(parseJumbleqPreset(JSON.stringify({ ...validPreset, curveA: 0 })).curveA, 0);
  assert.equal(parseJumbleqPreset(JSON.stringify({ ...validPreset, curveB: 100 })).curveB, 100);
});
