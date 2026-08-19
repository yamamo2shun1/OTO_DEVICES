"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import {
  decodeConfigMessage,
  JumbleqConfig,
  REQUEST_CURRENT_CONFIG,
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

  const accessRef = useRef<WebMidiAccess | null>(null);
  const inputRef = useRef<WebMidiInput | null>(null);
  const outputRef = useRef<WebMidiOutput | null>(null);
  const syncValuesRef = useRef<Partial<JumbleqConfig>>({});
  const syncFieldsRef = useRef(new Set<keyof JumbleqConfig>());
  const syncTimerRef = useRef<number | null>(null);
  const stateChangeHandlerRef = useRef<(() => void) | null>(null);
  const onConfigRef = useRef(onConfig);

  useEffect(() => {
    onConfigRef.current = onConfig;
  }, [onConfig]);

  const clearSyncTimer = useCallback(() => {
    if (syncTimerRef.current) window.clearTimeout(syncTimerRef.current);
    syncTimerRef.current = null;
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
    syncValuesRef.current = {};
    syncFieldsRef.current = new Set();
    setSyncReceived(0);
    setError(null);
    setStatus("syncing");
    outputRef.current.send(REQUEST_CURRENT_CONFIG);

    syncTimerRef.current = window.setTimeout(() => {
      const received = syncFieldsRef.current.size;
      setError(`Initial sync timed out (${received}/${SYNC_FIELD_COUNT} settings received).`);
      setStatus("error");
    }, SYNC_TIMEOUT_MS);
  }, [clearSyncTimer]);

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
  }, [clearSyncTimer, openPorts, refreshPorts]);

  const connectSelected = useCallback(async () => {
    try {
      await openPorts(selectedInputId, selectedOutputId);
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "Could not open the selected MIDI ports.";
      setError(message);
      setStatus("error");
    }
  }, [openPorts, selectedInputId, selectedOutputId]);

  const disconnect = useCallback(async () => {
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
  }, [clearSyncTimer]);

  useEffect(() => {
    return () => {
      clearSyncTimer();
      if (inputRef.current) inputRef.current.onmidimessage = null;
      if (accessRef.current && stateChangeHandlerRef.current) {
        accessRef.current.removeEventListener("statechange", stateChangeHandlerRef.current);
      }
    };
  }, [clearSyncTimer]);

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
    connect,
    connectSelected,
    disconnect,
    requestSync: beginSync,
  };
}
