import type { JumbleqConfig } from "../midi/jumbleq-midi";

const inputTypes = ["LINE", "PHONO"] as const;
const sources = ["CH 1", "CH 2", "USB 1/2", "USB 3/4"] as const;
const returnSources = ["USB 1/2", "USB 3/4"] as const;
const headphoneSources = ["Fader A", "Fader B", "Thru", "Master"] as const;
const auxiliarySides = ["A", "B"] as const;
const magneticModes = ["CC", "NOTE"] as const;

function presetObject(value: unknown): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    throw new Error("Preset must be a JSON object.");
  }
  return value as Record<string, unknown>;
}

function enumValue<const Value extends string>(
  preset: Record<string, unknown>,
  field: string,
  allowedValues: readonly Value[],
): Value {
  const value = preset[field];
  if (!allowedValues.includes(value as Value)) {
    throw new Error(`${field} must be one of: ${allowedValues.join(", ")}.`);
  }
  return value as Value;
}

function booleanValue(preset: Record<string, unknown>, field: string): boolean {
  const value = preset[field];
  if (typeof value !== "boolean") throw new Error(`${field} must be true or false.`);
  return value;
}

function curveValue(preset: Record<string, unknown>, field: string): number {
  const value = preset[field];
  if (typeof value !== "number" || !Number.isInteger(value) || value < 0 || value > 100) {
    throw new Error(`${field} must be an integer from 0 to 100.`);
  }
  return value;
}

export function parseJumbleqPreset(text: string): JumbleqConfig {
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error("Preset file is not valid JSON.");
  }

  const preset = presetObject(parsed);
  return {
    ch1Type: enumValue(preset, "ch1Type", inputTypes),
    ch2Type: enumValue(preset, "ch2Type", inputTypes),
    assignA: enumValue(preset, "assignA", sources),
    assignB: enumValue(preset, "assignB", sources),
    assignPost: enumValue(preset, "assignPost", sources),
    dvs1: booleanValue(preset, "dvs1"),
    dvs2: booleanValue(preset, "dvs2"),
    returnSource: enumValue(preset, "returnSource", returnSources),
    headphoneSource: enumValue(preset, "headphoneSource", headphoneSources),
    sensor2: enumValue(preset, "sensor2", auxiliarySides),
    sensor3: enumValue(preset, "sensor3", auxiliarySides),
    magMode: enumValue(preset, "magMode", magneticModes),
    curveA: curveValue(preset, "curveA"),
    curveB: curveValue(preset, "curveB"),
  };
}

export function serializeJumbleqPreset(config: JumbleqConfig) {
  return JSON.stringify(config, null, 2);
}
