import assert from "node:assert/strict";
import test from "node:test";

import {
  midiPortIdentity,
  selectInitialMidiPort,
  selectReconnectMidiPort,
} from "../app/midi/midi-port-selection.ts";

const jumbleqInput = {
  id: "jumbleq-input-1",
  manufacturer: "Microsoft Corporation",
  name: "JUMBLEQ MIDI",
};

test("initial connection prefers a JUMBLEQ port", () => {
  const generic = { id: "generic", manufacturer: "Other", name: "MIDI keyboard" };
  assert.equal(selectInitialMidiPort([generic, jumbleqInput]), jumbleqInput);
  assert.equal(selectInitialMidiPort([generic]), generic);
  assert.equal(selectInitialMidiPort([generic, { ...generic, id: "generic-2" }]), null);
});

test("reconnection first matches the original stable port id", () => {
  const target = midiPortIdentity(jumbleqInput);
  const sameIdRenamed = { ...jumbleqInput, name: "Renamed port" };
  assert.equal(selectReconnectMidiPort([sameIdRenamed], target), sameIdRenamed);
});

test("reconnection survives a changed id when device name and manufacturer match", () => {
  const target = midiPortIdentity(jumbleqInput);
  const reconnected = { ...jumbleqInput, id: "jumbleq-input-2", name: "  jumbleq midi  " };
  assert.equal(selectReconnectMidiPort([reconnected], target), reconnected);
});

test("reconnection accepts one same-name port when manufacturer metadata changes", () => {
  const target = midiPortIdentity(jumbleqInput);
  const reconnected = { id: "jumbleq-input-2", manufacturer: "", name: "JUMBLEQ MIDI" };
  assert.equal(selectReconnectMidiPort([reconnected], target), reconnected);
});

test("reconnection never substitutes an unrelated MIDI device", () => {
  const target = midiPortIdentity(jumbleqInput);
  const unrelated = { id: "generic", manufacturer: "Other", name: "MIDI keyboard" };
  assert.equal(selectReconnectMidiPort([unrelated], target), null);
  assert.equal(selectReconnectMidiPort([], target), null);
});

test("reconnection waits when duplicate same-name ports cannot be identified", () => {
  const target = midiPortIdentity({ ...jumbleqInput, manufacturer: "Unknown" });
  const duplicateA = { ...jumbleqInput, id: "a", manufacturer: "Vendor A" };
  const duplicateB = { ...jumbleqInput, id: "b", manufacturer: "Vendor B" };
  assert.equal(selectReconnectMidiPort([duplicateA, duplicateB], target), null);
});
