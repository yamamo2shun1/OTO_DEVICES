export type Source = "CH 1" | "CH 2" | "USB 1/2" | "USB 3/4";
export type InputType = "LINE" | "PHONO";
export type HeadphoneSource = "Fader A" | "Fader B" | "Thru" | "Master";
export type MagneticMode = "CC" | "NOTE";
export type AuxiliarySide = "A" | "B";

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

export type SyncField = keyof JumbleqConfig;

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
const sources: readonly Source[] = ["CH 1", "CH 2", "USB 1/2", "USB 3/4"];
const headphoneSources: readonly HeadphoneSource[] = ["Fader A", "Fader B", "Thru", "Master"];

export const REQUEST_CURRENT_CONFIG = new Uint8Array([
  PROGRAM_CHANGE | MIDI_CHANNEL_15,
  126,
]);

export type DecodedConfigValue = {
  [Key in SyncField]: { field: Key; value: JumbleqConfig[Key] };
}[SyncField];

function curveMidiCCToPercent(value: number) {
  return Math.round((value * 100) / 127);
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
