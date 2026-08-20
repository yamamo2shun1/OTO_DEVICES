import assert from "node:assert/strict";
import test from "node:test";

import {
  CURVE_EDIT_OFF,
  CURVE_EDIT_ON,
  decodeConfigMessage,
  encodeCurveSetting,
  encodeProgramSetting,
  REQUEST_CURRENT_CONFIG,
  RESTORE_DEFAULT_CONFIG,
  SAVE_CURRENT_CONFIG,
  SYNC_FIELD_COUNT,
  SYNC_FIELDS,
  curvePercentToMidiCC,
} from "../app/midi/jumbleq-midi.ts";

const PROGRAM_CHANGE_CH_15 = 0xce;
const CONTROL_CHANGE_CH_15 = 0xbe;

function bytes(message) {
  return Array.from(message);
}

test("control commands use MIDI channel 15 and the documented programs", () => {
  assert.deepEqual(bytes(CURVE_EDIT_OFF), [PROGRAM_CHANGE_CH_15, 120]);
  assert.deepEqual(bytes(CURVE_EDIT_ON), [PROGRAM_CHANGE_CH_15, 121]);
  assert.deepEqual(bytes(REQUEST_CURRENT_CONFIG), [PROGRAM_CHANGE_CH_15, 126]);
  assert.deepEqual(bytes(SAVE_CURRENT_CONFIG), [PROGRAM_CHANGE_CH_15, 127]);
});

test("restore defaults contain one value for every synchronized field", () => {
  assert.deepEqual(SYNC_FIELDS, [
    "ch1Type",
    "ch2Type",
    "assignA",
    "assignB",
    "assignPost",
    "dvs1",
    "dvs2",
    "returnSource",
    "headphoneSource",
    "sensor2",
    "sensor3",
    "magMode",
    "curveA",
    "curveB",
  ]);
  assert.equal(SYNC_FIELD_COUNT, 14);
  assert.deepEqual(Object.keys(RESTORE_DEFAULT_CONFIG).sort(), [...SYNC_FIELDS].sort());
  assert.deepEqual(RESTORE_DEFAULT_CONFIG, {
    ch1Type: "LINE",
    ch2Type: "LINE",
    assignA: "CH 1",
    assignB: "CH 2",
    assignPost: "USB 1/2",
    dvs1: false,
    dvs2: false,
    returnSource: "USB 3/4",
    headphoneSource: "Master",
    sensor2: "A",
    sensor3: "B",
    magMode: "CC",
    curveA: 50,
    curveB: 50,
  });
});

const programSettingCases = [
  ["ch1Type", "LINE", 0],
  ["ch1Type", "PHONO", 1],
  ["ch2Type", "LINE", 2],
  ["ch2Type", "PHONO", 3],
  ["assignA", "CH 1", 4],
  ["assignA", "CH 2", 5],
  ["assignA", "USB 1/2", 6],
  ["assignA", "USB 3/4", 7],
  ["assignB", "CH 1", 8],
  ["assignB", "CH 2", 9],
  ["assignB", "USB 1/2", 10],
  ["assignB", "USB 3/4", 11],
  ["assignPost", "CH 1", 12],
  ["assignPost", "CH 2", 13],
  ["assignPost", "USB 1/2", 14],
  ["assignPost", "USB 3/4", 15],
  ["dvs1", false, 16],
  ["dvs1", true, 17],
  ["dvs2", false, 18],
  ["dvs2", true, 19],
  ["returnSource", "USB 1/2", 20],
  ["returnSource", "USB 3/4", 21],
  ["headphoneSource", "Fader A", 22],
  ["headphoneSource", "Fader B", 23],
  ["headphoneSource", "Thru", 24],
  ["headphoneSource", "Master", 25],
  ["sensor2", "A", 26],
  ["sensor2", "B", 27],
  ["sensor3", "A", 28],
  ["sensor3", "B", 29],
  ["magMode", "CC", 122],
  ["magMode", "NOTE", 123],
];

test("every program setting follows the documented PC map and decodes back", () => {
  for (const [field, value, program] of programSettingCases) {
    const encoded = encodeProgramSetting(field, value);
    assert.deepEqual(bytes(encoded), [PROGRAM_CHANGE_CH_15, program], `${field}=${String(value)}`);
    assert.deepEqual(decodeConfigMessage(encoded), { field, value }, `decode PC${program}`);
  }
});

test("program encoder rejects values that could otherwise select another setting", () => {
  assert.throws(() => encodeProgramSetting("ch1Type", "MIC"), /Invalid value for ch1Type/);
  assert.throws(() => encodeProgramSetting("assignA", "INVALID"), /Invalid value for assignA/);
  assert.throws(() => encodeProgramSetting("dvs1", 1), /Invalid value for dvs1/);
  assert.throws(() => encodeProgramSetting("magMode", "POLY"), /Invalid value for magMode/);
  assert.throws(() => encodeProgramSetting("unknown", "LINE"), /Unknown setting field/);
});

test("curve percentages map to the full MIDI CC range", () => {
  assert.equal(curvePercentToMidiCC(-20), 0);
  assert.equal(curvePercentToMidiCC(0), 0);
  assert.equal(curvePercentToMidiCC(25), 32);
  assert.equal(curvePercentToMidiCC(50), 64);
  assert.equal(curvePercentToMidiCC(75), 95);
  assert.equal(curvePercentToMidiCC(100), 127);
  assert.equal(curvePercentToMidiCC(120), 127);
  assert.throws(() => curvePercentToMidiCC(Number.NaN), /finite number/);
  assert.throws(() => curvePercentToMidiCC(Number.POSITIVE_INFINITY), /finite number/);
});

test("curve messages use CC20/21 on MIDI channel 15 and round-trip 0-100 percent", () => {
  assert.deepEqual(bytes(encodeCurveSetting("curveA", 50)), [CONTROL_CHANGE_CH_15, 20, 64]);
  assert.deepEqual(bytes(encodeCurveSetting("curveB", 50)), [CONTROL_CHANGE_CH_15, 21, 64]);
  assert.throws(() => encodeCurveSetting("curveC", 50), /Unknown curve field/);

  for (let percent = 0; percent <= 100; percent += 1) {
    assert.deepEqual(decodeConfigMessage(encodeCurveSetting("curveA", percent)), { field: "curveA", value: percent });
    assert.deepEqual(decodeConfigMessage(encodeCurveSetting("curveB", percent)), { field: "curveB", value: percent });
  }
});

test("decoder converts every raw curve CC value to a normalized percentage", () => {
  for (let value = 0; value <= 127; value += 1) {
    const expected = Math.round((value * 100) / 127);
    assert.deepEqual(
      decodeConfigMessage(new Uint8Array([CONTROL_CHANGE_CH_15, 20, value])),
      { field: "curveA", value: expected },
    );
    assert.deepEqual(
      decodeConfigMessage(new Uint8Array([CONTROL_CHANGE_CH_15, 21, value])),
      { field: "curveB", value: expected },
    );
  }
});

test("decoder ignores incomplete, unrelated, and wrong-channel MIDI messages", () => {
  const ignoredMessages = [
    [],
    [PROGRAM_CHANGE_CH_15],
    [0xc0, 0],
    [PROGRAM_CHANGE_CH_15, 30],
    [CONTROL_CHANGE_CH_15, 20],
    [CONTROL_CHANGE_CH_15, 22, 64],
    [0xb0, 20, 64],
    [0x9e, 60, 127],
  ];

  for (const message of ignoredMessages) {
    assert.equal(decodeConfigMessage(new Uint8Array(message)), null, bytes(new Uint8Array(message)).join(","));
  }
});
