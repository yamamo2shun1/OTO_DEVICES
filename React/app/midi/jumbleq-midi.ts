export type Source = "CH 1" | "CH 2" | "USB 1/2" | "USB 3/4";
export type InputType = "LINE" | "PHONO";
export type HeadphoneSource = "Fader A" | "Fader B" | "Thru" | "Master";
export type MagneticMode = "CC" | "NOTE";
export type AuxiliarySide = "A" | "B";

export type MagneticLiveMessage = {
  index: 0 | 1 | 2 | 3;
  mode: MagneticMode;
  value: number;
  active: boolean;
};

export type JumbleqConfig = {
  ch1Type: InputType;
  ch2Type: InputType;
  assignA: Source;
  assignB: Source;
  assignPost: Source;
  dvs1: boolean;
  dvs2: boolean;
  returnSource: "USB 1/2" | "USB 3/4";
  headphoneSource: HeadphoneSource;
  sensor2: AuxiliarySide;
  sensor3: AuxiliarySide;
  magMode: MagneticMode;
  curveA: number;
  curveB: number;
};

export const RESTORE_DEFAULT_CONFIG: JumbleqConfig = {
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
};

export type SyncField = keyof JumbleqConfig;
export type ProgramSettingField = Exclude<SyncField, "curveA" | "curveB">;

export const SYNC_FIELDS: readonly SyncField[] = [
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
];

export const SYNC_FIELD_COUNT = SYNC_FIELDS.length;

const MIDI_CHANNEL_15 = 14;
const PROGRAM_CHANGE = 0xc0;
const CONTROL_CHANGE = 0xb0;
const inputTypes: readonly InputType[] = ["LINE", "PHONO"];
const sources: readonly Source[] = ["CH 1", "CH 2", "USB 1/2", "USB 3/4"];
const headphoneSources: readonly HeadphoneSource[] = ["Fader A", "Fader B", "Thru", "Master"];
const auxiliarySides: readonly AuxiliarySide[] = ["A", "B"];
const magneticModes: readonly MagneticMode[] = ["CC", "NOTE"];

export const REQUEST_CURRENT_CONFIG = new Uint8Array([
  PROGRAM_CHANGE | MIDI_CHANNEL_15,
  126,
]);

function programChange(program: number) {
  return new Uint8Array([PROGRAM_CHANGE | MIDI_CHANNEL_15, program]);
}

function settingIndex<Value extends string>(
  field: ProgramSettingField,
  value: unknown,
  allowedValues: readonly Value[],
) {
  const index = allowedValues.indexOf(value as Value);
  if (index < 0) throw new Error(`Invalid value for ${field}.`);
  return index;
}

function booleanIndex(field: ProgramSettingField, value: unknown) {
  if (typeof value !== "boolean") throw new Error(`Invalid value for ${field}.`);
  return value ? 1 : 0;
}

export const CURVE_EDIT_OFF = programChange(120);
export const CURVE_EDIT_ON = programChange(121);
export const SAVE_CURRENT_CONFIG = programChange(127);

export type DecodedConfigValue = {
  [Key in SyncField]: { field: Key; value: JumbleqConfig[Key] };
}[SyncField];

function curveMidiCCToPercent(value: number) {
  return Math.round((value * 100) / 127);
}

export function curvePercentToMidiCC(percent: number) {
  if (!Number.isFinite(percent)) throw new Error("Curve percentage must be a finite number.");
  return Math.round(Math.min(100, Math.max(0, percent)) * 127 / 100);
}

export function encodeProgramSetting(
  field: ProgramSettingField,
  value: JumbleqConfig[ProgramSettingField],
) {
  let program: number;

  switch (field) {
    case "ch1Type": program = settingIndex(field, value, inputTypes); break;
    case "ch2Type": program = 2 + settingIndex(field, value, inputTypes); break;
    case "assignA": program = 4 + settingIndex(field, value, sources); break;
    case "assignB": program = 8 + settingIndex(field, value, sources); break;
    case "assignPost": program = 12 + settingIndex(field, value, sources); break;
    case "dvs1": program = 16 + booleanIndex(field, value); break;
    case "dvs2": program = 18 + booleanIndex(field, value); break;
    case "returnSource": program = 20 + settingIndex(field, value, ["USB 1/2", "USB 3/4"] as const); break;
    case "headphoneSource": program = 22 + settingIndex(field, value, headphoneSources); break;
    case "sensor2": program = 26 + settingIndex(field, value, auxiliarySides); break;
    case "sensor3": program = 28 + settingIndex(field, value, auxiliarySides); break;
    case "magMode": program = 122 + settingIndex(field, value, magneticModes); break;
    default: throw new Error(`Unknown setting field: ${String(field)}.`);
  }

  return programChange(program);
}

export function encodeCurveSetting(field: "curveA" | "curveB", percent: number) {
  if (field !== "curveA" && field !== "curveB") throw new Error(`Unknown curve field: ${String(field)}.`);
  return new Uint8Array([
    CONTROL_CHANGE | MIDI_CHANNEL_15,
    field === "curveA" ? 20 : 21,
    curvePercentToMidiCC(percent),
  ]);
}

function decodeProgramChange(program: number): DecodedConfigValue | null {
  if (program <= 1) return { field: "ch1Type", value: program === 0 ? "LINE" : "PHONO" };
  if (program <= 3) return { field: "ch2Type", value: program === 2 ? "LINE" : "PHONO" };
  if (program <= 7) return { field: "assignA", value: sources[program - 4] };
  if (program <= 11) return { field: "assignB", value: sources[program - 8] };
  if (program <= 15) return { field: "assignPost", value: sources[program - 12] };
  if (program <= 17) return { field: "dvs1", value: program === 17 };
  if (program <= 19) return { field: "dvs2", value: program === 19 };
  if (program <= 21) return { field: "returnSource", value: program === 20 ? "USB 1/2" : "USB 3/4" };
  if (program <= 25) return { field: "headphoneSource", value: headphoneSources[program - 22] };
  if (program <= 27) return { field: "sensor2", value: program === 26 ? "A" : "B" };
  if (program <= 29) return { field: "sensor3", value: program === 28 ? "A" : "B" };
  if (program === 122 || program === 123) return { field: "magMode", value: program === 122 ? "CC" : "NOTE" };
  return null;
}

export function decodeConfigMessage(data: Uint8Array): DecodedConfigValue | null {
  if (data.length < 2) return null;

  const status = data[0];
  const messageType = status & 0xf0;
  const channel = status & 0x0f;
  if (channel !== MIDI_CHANNEL_15) return null;

  if (messageType === PROGRAM_CHANGE) return decodeProgramChange(data[1]);

  if (messageType === CONTROL_CHANGE && data.length >= 3) {
    const controller = data[1];
    const value = curveMidiCCToPercent(data[2]);
    if (controller === 20) return { field: "curveA", value };
    if (controller === 21) return { field: "curveB", value };
  }

  return null;
}

export function decodeMagneticLiveMessage(data: Uint8Array): MagneticLiveMessage | null {
  if (data.length < 3) return null;

  const status = data[0];
  const messageType = status & 0xf0;
  const channel = status & 0x0f;
  if (channel !== 0) return null;

  if (messageType === CONTROL_CHANGE) {
    const ccNumber = data[1];
    if (ccNumber < 12 || ccNumber > 15) return null;
    const value = data[2];
    return {
      index: (ccNumber - 12) as MagneticLiveMessage["index"],
      mode: "CC",
      value,
      active: value > 0,
    };
  }

  if (messageType === 0x90 || messageType === 0x80) {
    const noteNumber = data[1];
    if (noteNumber < 68 || noteNumber > 71) return null;
    const velocity = data[2];
    const active = messageType === 0x90 && velocity > 0;
    return {
      index: (noteNumber - 68) as MagneticLiveMessage["index"],
      mode: "NOTE",
      value: active ? velocity : 0,
      active,
    };
  }

  return null;
}
