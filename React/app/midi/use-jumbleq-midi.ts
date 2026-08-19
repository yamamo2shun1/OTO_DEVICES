"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import {
  CURVE_EDIT_OFF,
  CURVE_EDIT_ON,
  decodeConfigMessage,
  encodeCurveSetting,
  encodeProgramSetting,
  JumbleqConfig,
  ProgramSettingField,
  REQUEST_CURRENT_CONFIG,
  SAVE_CURRENT_CONFIG,
  SYNC_FIELD_COUNT,
} from "./jumbleq-midi";

type MidiPortState = "connected" | "disconnected";

type WebMidiPort = {
  id: string;
  manufacturer?: string;
  name?: string;
  state: MidiPortState;
  open: () => Promise<WebMidiPort>;
  close: () => Promise<WebMidiPort>;
};

type WebMidiInput = WebMidiPort & {
  onmidimessage: ((event: { data: Uint8Array }) => void) | null;
};

type WebMidiOutput = WebMidiPort & {
  send: (data: Uint8Array | number[]) => void;
};

type WebMidiAccess = {
  inputs: Map<string, WebMidiInput>;
  outputs: Map<string, WebMidiOutput>;
  addEventListener: (type: "statechange", listener: () => void) => void;
  removeEventListener: (type: "statechange", listener: () => void) => void;
};

type MidiNavigator = Navigator & {
  requestMIDIAccess?: () => Promise<WebMidiAccess>;
};

export type MidiStatus =
  | "idle"
  | "unsupported"
  | "requesting"
  | "selecting"
  | "connecting"
  | "syncing"
  | "ready"
  | "error";

export type MidiPortOption = {
  id: string;
  name: string;
  manufacturer: string;
};

const SYNC_TIMEOUT_MS = 2500;

function portOption(port: WebMidiPort): MidiPortOption {
  return {
    id: port.id,
    name: port.name?.trim() || "Unnamed MIDI port",
    manufacturer: port.manufacturer?.trim() || "",
  };
}

function isJumbleqPort(port: WebMidiPort) {
  return `${port.manufacturer ?? ""} ${port.name ?? ""}`.toLowerCase().includes("jumbleq");
}

function availablePorts<T extends WebMidiPort>(ports: Map<string, T>) {
  return Array.from(ports.values()).filter((port) => port.state !== "disconnected");
}

export function useJumbleqMidi(onConfig: (config: JumbleqConfig) => void) {
  const [status, setStatus] = useState<MidiStatus>("idle");
  const [error, setError] = useState<string | null>(null);
  const [inputs, setInputs] = useState<MidiPortOption[]>([]);
  const [outputs, setOutputs] = useState<MidiPortOption[]>([]);
  const [selectedInputId, setSelectedInputId] = useState("");
  const [selectedOutputId, setSelectedOutputId] = useState("");
  const [syncReceived, setSyncReceived] = useState(0);
  const [accessGranted, setAccessGranted] = useState(false);
  const [connectedInputName, setConnectedInputName] = useState("JUMBLEQ MIDI");
  const [hasOpenPorts, setHasOpenPorts] = useState(false);
  const [curveEditActive, setCurveEditActive] = useState(false);

  const accessRef = useRef<WebMidiAccess | null>(null);
  const inputRef = useRef<WebMidiInput | null>(null);
  const outputRef = useRef<WebMidiOutput | null>(null);
  const syncValuesRef = useRef<Partial<JumbleqConfig>>({});
  const syncFieldsRef = useRef(new Set<keyof JumbleqConfig>());
  const syncTimerRef = useRef<number | null>(null);
  const curveEditTimerRef = useRef<number | null>(null);
  const curveEditActiveRef = useRef(false);
  const stateChangeHandlerRef = useRef<(() => void) | null>(null);
  const onConfigRef = useRef(onConfig);

  useEffect(() => {
    onConfigRef.current = onConfig;
  }, [onConfig]);

  const clearSyncTimer = useCallback(() => {
    if (syncTimerRef.current) window.clearTimeout(syncTimerRef.current);
    syncTimerRef.current = null;
  }, []);

  const clearCurveEditTimer = useCallback(() => {
    if (curveEditTimerRef.current) window.clearTimeout(curveEditTimerRef.current);
    curveEditTimerRef.current = null;
  }, []);

  const sendRaw = useCallback((data: Uint8Array, description: string) => {
    const output = outputRef.current;
    if (!output || output.state === "disconnected") {
      setError(`Cannot send ${description}: MIDI output is not connected.`);
      return false;
    }

    try {
      output.send(data);
      return true;
    } catch (caught) {
      const detail = caught instanceof Error ? ` ${caught.message}` : "";
      setError(`Could not send ${description}.${detail}`);
      return false;
    }
  }, []);

  const handleMidiMessage = useCallback((event: { data: Uint8Array }) => {
    const decoded = decodeConfigMessage(event.data);
    if (!decoded) return;

    syncValuesRef.current = { ...syncValuesRef.current, [decoded.field]: decoded.value };
    syncFieldsRef.current.add(decoded.field);
    const received = syncFieldsRef.current.size;
    setSyncReceived(received);

    if (received === SYNC_FIELD_COUNT) {
      clearSyncTimer();
      onConfigRef.current(syncValuesRef.current as JumbleqConfig);
      setError(null);
      setStatus("ready");
    }
  }, [clearSyncTimer]);

  const beginSync = useCallback(() => {
    if (!outputRef.current) {
      setError("MIDI output is not connected.");
      setStatus("error");
      return;
    }

    clearSyncTimer();
    clearCurveEditTimer();
    if (curveEditActiveRef.current) {
      sendRaw(CURVE_EDIT_OFF, "curve edit off");
      curveEditActiveRef.current = false;
      setCurveEditActive(false);
    }
    syncValuesRef.current = {};
    syncFieldsRef.current = new Set();
    setSyncReceived(0);
    setError(null);
    setStatus("syncing");
    if (!sendRaw(REQUEST_CURRENT_CONFIG, "current settings request")) {
      setStatus("error");
      return;
    }

    syncTimerRef.current = window.setTimeout(() => {
      const received = syncFieldsRef.current.size;
      setError(`Initial sync timed out (${received}/${SYNC_FIELD_COUNT} settings received).`);
      setStatus("error");
    }, SYNC_TIMEOUT_MS);
  }, [clearCurveEditTimer, clearSyncTimer, sendRaw]);

  const refreshPorts = useCallback((access: WebMidiAccess) => {
    const nextInputs = availablePorts(access.inputs);
    const nextOutputs = availablePorts(access.outputs);
    setInputs(nextInputs.map(portOption));
    setOutputs(nextOutputs.map(portOption));

    const preferredInput = nextInputs.find(isJumbleqPort) ?? (nextInputs.length === 1 ? nextInputs[0] : null);
    const preferredOutput = nextOutputs.find(isJumbleqPort) ?? (nextOutputs.length === 1 ? nextOutputs[0] : null);
    setSelectedInputId((current) => nextInputs.some((port) => port.id === current) ? current : preferredInput?.id ?? "");
    setSelectedOutputId((current) => nextOutputs.some((port) => port.id === current) ? current : preferredOutput?.id ?? "");

    return { preferredInput, preferredOutput };
  }, []);

  const openPorts = useCallback(async (inputId: string, outputId: string) => {
    const access = accessRef.current;
    const input = access?.inputs.get(inputId);
    const output = access?.outputs.get(outputId);
    if (!input || !output || input.state === "disconnected" || output.state === "disconnected") {
      setError("Select an available MIDI input and output.");
      setStatus("selecting");
      return;
    }

    setStatus("connecting");
    setError(null);
    clearSyncTimer();

    if (inputRef.current && inputRef.current !== input) {
      inputRef.current.onmidimessage = null;
      await inputRef.current.close();
    }
    if (outputRef.current && outputRef.current !== output) await outputRef.current.close();

    await Promise.all([input.open(), output.open()]);
    input.onmidimessage = handleMidiMessage;
    inputRef.current = input;
    outputRef.current = output;
    setConnectedInputName(input.name?.trim() || "JUMBLEQ MIDI");
    setHasOpenPorts(true);
    setSelectedInputId(input.id);
    setSelectedOutputId(output.id);
    beginSync();
  }, [beginSync, clearSyncTimer, handleMidiMessage]);

  const connect = useCallback(async () => {
    const midiNavigator = navigator as MidiNavigator;
    if (!midiNavigator.requestMIDIAccess) {
      setStatus("unsupported");
      setError("This browser does not support Web MIDI.");
      return;
    }

    try {
      setStatus("requesting");
      setError(null);
      const access = accessRef.current ?? await midiNavigator.requestMIDIAccess.call(navigator);
      accessRef.current = access;
      setAccessGranted(true);
      if (stateChangeHandlerRef.current) access.removeEventListener("statechange", stateChangeHandlerRef.current);
      const stateChangeHandler = () => {
        const refreshed = refreshPorts(access);
        const activeInput = inputRef.current;
        const activeOutput = outputRef.current;
        if ((activeInput && activeInput.state === "disconnected") || (activeOutput && activeOutput.state === "disconnected")) {
          clearSyncTimer();
          if (activeInput) activeInput.onmidimessage = null;
          inputRef.current = null;
          outputRef.current = null;
          clearCurveEditTimer();
          curveEditActiveRef.current = false;
          setCurveEditActive(false);
          setHasOpenPorts(false);
          setError("JUMBLEQ was disconnected.");
          setStatus("error");
        } else if (!activeInput && (!refreshed.preferredInput || !refreshed.preferredOutput)) {
          setStatus("selecting");
        }
      };
      stateChangeHandlerRef.current = stateChangeHandler;
      access.addEventListener("statechange", stateChangeHandler);
      const ports = refreshPorts(access);

      if (ports.preferredInput && ports.preferredOutput) {
        await openPorts(ports.preferredInput.id, ports.preferredOutput.id);
      } else {
        setError("Select the JUMBLEQ MIDI input and output below.");
        setStatus("selecting");
      }
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "MIDI access was not granted.";
      setError(message);
      setStatus("error");
    }
  }, [clearCurveEditTimer, clearSyncTimer, openPorts, refreshPorts]);

  const connectSelected = useCallback(async () => {
    try {
      await openPorts(selectedInputId, selectedOutputId);
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "Could not open the selected MIDI ports.";
      setError(message);
      setStatus("error");
    }
  }, [openPorts, selectedInputId, selectedOutputId]);

  const sendProgramSetting = useCallback((
    field: ProgramSettingField,
    value: JumbleqConfig[ProgramSettingField],
  ) => {
    try {
      return sendRaw(encodeProgramSetting(field, value), field);
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : `Invalid value for ${field}.`;
      setError(message);
      return false;
    }
  }, [sendRaw]);

  const beginCurveEdit = useCallback(() => {
    clearCurveEditTimer();
    if (curveEditActiveRef.current) return true;
    if (!sendRaw(CURVE_EDIT_ON, "curve edit on")) return false;
    curveEditActiveRef.current = true;
    setCurveEditActive(true);
    return true;
  }, [clearCurveEditTimer, sendRaw]);

  const endCurveEdit = useCallback(() => {
    clearCurveEditTimer();
    if (!curveEditActiveRef.current) return true;
    const sent = sendRaw(CURVE_EDIT_OFF, "curve edit off");
    curveEditActiveRef.current = false;
    setCurveEditActive(false);
    return sent;
  }, [clearCurveEditTimer, sendRaw]);

  const sendCurveSetting = useCallback((field: "curveA" | "curveB", percent: number) => {
    if (!beginCurveEdit()) return false;
    const sent = sendRaw(encodeCurveSetting(field, percent), field);
    clearCurveEditTimer();
    curveEditTimerRef.current = window.setTimeout(() => endCurveEdit(), 800);
    return sent;
  }, [beginCurveEdit, clearCurveEditTimer, endCurveEdit, sendRaw]);

  const saveCurrentConfig = useCallback(() => {
    return sendRaw(SAVE_CURRENT_CONFIG, "save to EEPROM");
  }, [sendRaw]);

  const disconnect = useCallback(async () => {
    endCurveEdit();
    clearSyncTimer();
    const input = inputRef.current;
    const output = outputRef.current;
    inputRef.current = null;
    outputRef.current = null;
    setHasOpenPorts(false);
    if (input) input.onmidimessage = null;
    await Promise.allSettled([input?.close(), output?.close()].filter(Boolean) as Promise<WebMidiPort>[]);
    setSyncReceived(0);
    setError(null);
    setStatus("idle");
  }, [clearSyncTimer, endCurveEdit]);

  useEffect(() => {
    return () => {
      clearSyncTimer();
      clearCurveEditTimer();
      if (curveEditActiveRef.current && outputRef.current?.state !== "disconnected") {
        try { outputRef.current?.send(CURVE_EDIT_OFF); } catch { /* Best-effort cleanup. */ }
      }
      if (inputRef.current) inputRef.current.onmidimessage = null;
      if (accessRef.current && stateChangeHandlerRef.current) {
        accessRef.current.removeEventListener("statechange", stateChangeHandlerRef.current);
      }
    };
  }, [clearCurveEditTimer, clearSyncTimer]);

  return {
    status,
    error,
    inputs,
    outputs,
    selectedInputId,
    selectedOutputId,
    setSelectedInputId,
    setSelectedOutputId,
    syncReceived,
    accessGranted,
    connectedInputName,
    hasOpenPorts,
    curveEditActive,
    connect,
    connectSelected,
    disconnect,
    requestSync: beginSync,
    sendProgramSetting,
    beginCurveEdit,
    endCurveEdit,
    sendCurveSetting,
    saveCurrentConfig,
  };
}
