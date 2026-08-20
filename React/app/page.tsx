"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { ChangeEvent } from "react";
import Image from "next/image";
import {
  Cable,
  ChevronDown,
  CircleHelp,
  Download,
  Gauge,
  GitBranch,
  Headphones,
  Menu,
  Radio,
  RefreshCw,
  RotateCcw,
  Save,
  Settings2,
  SlidersHorizontal,
  Upload,
  Usb,
  X,
} from "lucide-react";
import { curvePercentToMidiCC, JumbleqConfig, ProgramSettingField, RESTORE_DEFAULT_CONFIG, Source, SYNC_FIELD_COUNT } from "./midi/jumbleq-midi";
import { useJumbleqMidi } from "./midi/use-jumbleq-midi";
import { parseJumbleqPreset, serializeJumbleqPreset } from "./presets/jumbleq-preset";

const sources: Source[] = ["CH 1", "CH 2", "USB 1/2", "USB 3/4"];
const MAX_PRESET_FILE_BYTES = 64 * 1024;

const CURVE_WIDTH_MIN = 0.02;
const CURVE_WIDTH_MAX = 0.60;
const CURVE_LINEAR_BLEND = 0.60;

function clamp01(value: number) {
  return Math.min(1, Math.max(0, value));
}

function curvePercentToWidth(percent: number) {
  const midiCC = curvePercentToMidiCC(percent);
  const normalized = midiCC / 127;
  return CURVE_WIDTH_MAX + ((CURVE_WIDTH_MIN - CURVE_WIDTH_MAX) * normalized);
}

function evaluateFirmwareCurve(normalizedPosition: number, curvePercent: number) {
  const width = curvePercentToWidth(curvePercent);
  const position = clamp01(normalizedPosition) * CURVE_WIDTH_MAX;
  const t = clamp01(position / width);
  const smootherstep = t * t * t * (t * ((t * 6) - 15) + 10);
  return (CURVE_LINEAR_BLEND * t) + ((1 - CURVE_LINEAR_BLEND) * smootherstep);
}

function createCurvePath(curvePercent: number, reverse = false) {
  const pointCount = 64;
  return Array.from({ length: pointCount + 1 }, (_, index) => {
    const xNormalized = index / pointCount;
    const curvePosition = reverse ? 1 - xNormalized : xNormalized;
    const gain = evaluateFirmwareCurve(curvePosition, curvePercent);
    const x = 18 + (284 * xNormalized);
    const y = 144 - (124 * gain);
    return `${index === 0 ? "M" : "L"} ${x.toFixed(2)} ${y.toFixed(2)}`;
  }).join(" ");
}

function SourceSelect({
  label,
  value,
  onChange,
  accent,
}: {
  label: string;
  value: Source;
  onChange: (value: Source) => void;
  accent: "a" | "b" | "neutral";
}) {
  return (
    <label className="source-field">
      <span>{label}</span>
      <div className={`select-shell select-${accent}`}>
        <select value={value} onChange={(event) => onChange(event.target.value as Source)}>
          {sources.map((source) => (
            <option key={source}>{source}</option>
          ))}
        </select>
        <ChevronDown aria-hidden="true" size={16} />
      </div>
    </label>
  );
}

function CurveGraph({ curveA, curveB }: { curveA: number; curveB: number }) {
  const pathA = useMemo(() => createCurvePath(curveA), [curveA]);
  const pathB = useMemo(() => createCurvePath(curveB, true), [curveB]);

  return (
    <svg className="curve-graph" viewBox="0 0 320 164" role="img" aria-label="Crossfader response curves">
      <defs>
        <linearGradient id="gridFade" x1="0" x2="0" y1="0" y2="1">
          <stop offset="0" stopColor="#fff" stopOpacity=".13" />
          <stop offset="1" stopColor="#fff" stopOpacity=".025" />
        </linearGradient>
      </defs>
      {[18, 89, 160, 231, 302].map((x) => (
        <line key={`x-${x}`} x1={x} y1="18" x2={x} y2="146" stroke="url(#gridFade)" />
      ))}
      {[20, 51, 82, 113, 144].map((y) => (
        <line key={`y-${y}`} x1="18" y1={y} x2="302" y2={y} stroke="url(#gridFade)" />
      ))}
      <path d={pathA} className="curve-line curve-a" />
      <path d={pathB} className="curve-line curve-b" />
      <circle cx="18" cy="144" r="3.5" className="dot-a" />
      <circle cx="302" cy="20" r="3.5" className="dot-a" />
      <circle cx="18" cy="20" r="3.5" className="dot-b" />
      <circle cx="302" cy="144" r="3.5" className="dot-b" />
    </svg>
  );
}

export default function Home() {
  const [menuOpen, setMenuOpen] = useState(false);
  const [ch1Type, setCh1Type] = useState<"LINE" | "PHONO">(RESTORE_DEFAULT_CONFIG.ch1Type);
  const [ch2Type, setCh2Type] = useState<"LINE" | "PHONO">(RESTORE_DEFAULT_CONFIG.ch2Type);
  const [assignA, setAssignA] = useState<Source>(RESTORE_DEFAULT_CONFIG.assignA);
  const [assignB, setAssignB] = useState<Source>(RESTORE_DEFAULT_CONFIG.assignB);
  const [assignPost, setAssignPost] = useState<Source>(RESTORE_DEFAULT_CONFIG.assignPost);
  const [curveA, setCurveA] = useState(RESTORE_DEFAULT_CONFIG.curveA);
  const [curveB, setCurveB] = useState(RESTORE_DEFAULT_CONFIG.curveB);
  const [dvs1, setDvs1] = useState(RESTORE_DEFAULT_CONFIG.dvs1);
  const [dvs2, setDvs2] = useState(RESTORE_DEFAULT_CONFIG.dvs2);
  const [returnSource, setReturnSource] = useState<"USB 1/2" | "USB 3/4">(RESTORE_DEFAULT_CONFIG.returnSource);
  const [headphoneSource, setHeadphoneSource] = useState<"Fader A" | "Fader B" | "Thru" | "Master">(RESTORE_DEFAULT_CONFIG.headphoneSource);
  const [magMode, setMagMode] = useState<"CC" | "NOTE">(RESTORE_DEFAULT_CONFIG.magMode);
  const [sensor2, setSensor2] = useState<"A" | "B">(RESTORE_DEFAULT_CONFIG.sensor2);
  const [sensor3, setSensor3] = useState<"A" | "B">(RESTORE_DEFAULT_CONFIG.sensor3);
  const [dirty, setDirty] = useState(false);
  const [saved, setSaved] = useState(false);
  const [presetNotice, setPresetNotice] = useState<{ type: "success" | "error"; message: string } | null>(null);
  const presetFileInputRef = useRef<HTMLInputElement>(null);
  const presetNoticeTimerRef = useRef<number | null>(null);

  const showPresetNotice = useCallback((type: "success" | "error", message: string) => {
    if (presetNoticeTimerRef.current) window.clearTimeout(presetNoticeTimerRef.current);
    setPresetNotice({ type, message });
    presetNoticeTimerRef.current = window.setTimeout(() => setPresetNotice(null), 4200);
  }, []);

  useEffect(() => () => {
    if (presetNoticeTimerRef.current) window.clearTimeout(presetNoticeTimerRef.current);
  }, []);

  const applySyncedConfig = useCallback((config: JumbleqConfig) => {
    setCh1Type(config.ch1Type);
    setCh2Type(config.ch2Type);
    setAssignA(config.assignA);
    setAssignB(config.assignB);
    setAssignPost(config.assignPost);
    setCurveA(config.curveA);
    setCurveB(config.curveB);
    setDvs1(config.dvs1);
    setDvs2(config.dvs2);
    setReturnSource(config.returnSource);
    setHeadphoneSource(config.headphoneSource);
    setMagMode(config.magMode);
    setSensor2(config.sensor2);
    setSensor3(config.sensor3);
    setDirty(false);
    setSaved(false);
  }, []);

  const {
    status: midiStatus,
    error: midiError,
    inputs: midiInputs,
    outputs: midiOutputs,
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
    requestSync,
    sendProgramSetting,
    beginCurveEdit,
    endCurveEdit,
    sendCurveSetting,
    saveCurrentConfig,
  } = useJumbleqMidi(applySyncedConfig);

  const connected = midiStatus === "ready";
  const reconnecting = midiStatus === "reconnecting";
  const connectionBusy = midiStatus === "requesting" || midiStatus === "connecting" || reconnecting;
  const settingsLocked = connectionBusy || midiStatus === "syncing";
  const connectButtonLabel = midiStatus === "unsupported"
    ? "MIDI unsupported"
    : midiStatus === "requesting"
      ? "Requesting access…"
      : midiStatus === "connecting"
        ? "Opening MIDI…"
        : reconnecting
          ? "Reconnecting…"
          : midiStatus === "syncing"
            ? `Syncing ${syncReceived}/${SYNC_FIELD_COUNT}`
            : connected
              ? "JUMBLEQ connected"
              : hasOpenPorts
                ? "Disconnect device"
                : "Connect device";

  const updateProgram = <Key extends ProgramSettingField>(
    setter: (value: JumbleqConfig[Key]) => void,
    field: Key,
    value: JumbleqConfig[Key],
  ) => {
    setter(value);
    if (hasOpenPorts) sendProgramSetting(field, value);
    setDirty(true);
    setSaved(false);
  };

  const updateCurve = (
    setter: (value: number) => void,
    field: "curveA" | "curveB",
    value: number,
  ) => {
    setter(value);
    if (hasOpenPorts) sendCurveSetting(field, value);
    setDirty(true);
    setSaved(false);
  };

  const save = () => {
    if (!hasOpenPorts || !saveCurrentConfig()) return;
    setDirty(false);
    setSaved(true);
    window.setTimeout(() => setSaved(false), 2200);
  };

  const sendConfigToDevice = (config: JumbleqConfig) => {
    if (!hasOpenPorts) return true;
    const results = [
      sendProgramSetting("ch1Type", config.ch1Type),
      sendProgramSetting("ch2Type", config.ch2Type),
      sendProgramSetting("assignA", config.assignA),
      sendProgramSetting("assignB", config.assignB),
      sendProgramSetting("assignPost", config.assignPost),
      sendProgramSetting("dvs1", config.dvs1),
      sendProgramSetting("dvs2", config.dvs2),
      sendProgramSetting("returnSource", config.returnSource),
      sendProgramSetting("headphoneSource", config.headphoneSource),
      sendProgramSetting("sensor2", config.sensor2),
      sendProgramSetting("sensor3", config.sensor3),
      sendProgramSetting("magMode", config.magMode),
      sendCurveSetting("curveA", config.curveA),
      sendCurveSetting("curveB", config.curveB),
      endCurveEdit(),
    ];
    return results.every(Boolean);
  };

  const reset = () => {
    const defaults = RESTORE_DEFAULT_CONFIG;
    applySyncedConfig(defaults);
    sendConfigToDevice(defaults);
    setDirty(true); setSaved(false);
  };

  const exportPreset = () => {
    const preset = { ch1Type, ch2Type, assignA, assignB, assignPost, curveA, curveB, dvs1, dvs2, returnSource, headphoneSource, magMode, sensor2, sensor3 };
    const url = URL.createObjectURL(new Blob([serializeJumbleqPreset(preset)], { type: "application/json" }));
    const anchor = document.createElement("a"); anchor.href = url; anchor.download = "jumbleq-preset.json"; anchor.click(); URL.revokeObjectURL(url);
  };

  const importPreset = async (event: ChangeEvent<HTMLInputElement>) => {
    const input = event.currentTarget;
    const file = input.files?.[0];
    if (!file) return;

    try {
      if (file.size > MAX_PRESET_FILE_BYTES) throw new Error("Preset file is too large.");
      const preset = parseJumbleqPreset(await file.text());
      applySyncedConfig(preset);
      const sent = sendConfigToDevice(preset);
      setDirty(true);
      setSaved(false);

      if (hasOpenPorts && !sent) {
        showPresetNotice("error", "Preset loaded, but some settings could not be sent to JUMBLEQ.");
      } else {
        showPresetNotice("success", hasOpenPorts ? "Preset imported and sent to JUMBLEQ." : "Preset imported into the preview.");
      }
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "Could not import the preset file.";
      showPresetNotice("error", message);
    } finally {
      input.value = "";
    }
  };

  return (
    <main className="app-shell">
      <header className="topbar">
        <button className="icon-button mobile-only" aria-label="Open navigation" onClick={() => setMenuOpen(true)}>
          <Menu size={20} />
        </button>
        <a href="#top" className="brand" aria-label="JUMBLEQ Configurator home">
          <Image
            className="brand-mark-image"
            src="/JUMBLEQ-Icon.png"
            width={34}
            height={34}
            alt=""
            aria-hidden="true"
            priority
          />
          <span className="brand-name">JUMBLEQ</span>
          <span className="brand-product">CONFIGURATOR</span>
        </a>
        <div className="topbar-actions">
          <button className="help-button" aria-label="Open help"><CircleHelp size={18} /><span>Help</span></button>
          <button
            className={`connect-button ${connected ? "is-connected" : ""}`}
            onClick={() => hasOpenPorts ? void disconnect() : void connect()}
            disabled={connectionBusy || midiStatus === "unsupported"}
          >
            <span className="connection-dot" />
            {connectButtonLabel}
          </button>
        </div>
      </header>

      <div className="app-body" id="top">
        <aside className={`sidebar ${menuOpen ? "is-open" : ""}`}>
          <div className="mobile-nav-header">
            <span>MENU</span>
            <button className="icon-button" aria-label="Close navigation" onClick={() => setMenuOpen(false)}><X size={20} /></button>
          </div>
          <nav aria-label="Configurator sections">
            <a className="nav-link is-active" href="#routing" onClick={() => setMenuOpen(false)}><GitBranch size={19} />Routing</a>
            <a className="nav-link" href="#crossfader" onClick={() => setMenuOpen(false)}><SlidersHorizontal size={19} />Crossfader</a>
            <a className="nav-link" href="#controls" onClick={() => setMenuOpen(false)}><Gauge size={19} />Controls</a>
            <a className="nav-link" href="#device" onClick={() => setMenuOpen(false)}><Usb size={19} />Device</a>
          </nav>
          <div className="sidebar-device">
            <Cable size={18} />
            <div>
              <strong>{hasOpenPorts || reconnecting ? connectedInputName : "No device"}</strong>
              <span>{connected ? "USB MIDI · Synced" : reconnecting ? "Waiting for USB" : midiStatus === "syncing" ? `Reading ${syncReceived}/${SYNC_FIELD_COUNT}` : "Connect via USB"}</span>
            </div>
          </div>
          <span className="version">Configurator preview · v0.1</span>
        </aside>

        {menuOpen && <button className="sidebar-scrim" aria-label="Close navigation" onClick={() => setMenuOpen(false)} />}

        <section className="workspace">
          <div className="page-heading">
            <div><p className="eyebrow">SIGNAL FLOW</p><h1>Routing</h1><p>Choose the sources that feed each side of the crossfader.</p></div>
            <div className={`device-pill ${connected ? "online" : reconnecting ? "reconnecting" : ""}`}><span />{connected ? "Online" : reconnecting ? "Reconnecting…" : midiStatus === "syncing" ? "Syncing…" : "Demo mode"}</div>
          </div>

          <fieldset className="settings-fieldset" disabled={settingsLocked} aria-busy={settingsLocked}>
          <section className="routing-grid" id="routing">
            <article className="channel-card channel-a">
              <div className="card-kicker"><span>1</span>INPUT SELECT SWITCH</div>
              <div className="segmented" role="group" aria-label="Channel 1 input type">
                {(["LINE", "PHONO"] as const).map((item) => <button key={item} className={ch1Type === item ? "active" : ""} onClick={() => updateProgram(setCh1Type, "ch1Type", item)}>{item}</button>)}
              </div>
              <div className="mini-meter"><span style={{ height: "36%" }} /><span style={{ height: "62%" }} /><span style={{ height: "78%" }} /><span style={{ height: "48%" }} /><span style={{ height: "28%" }} /></div>
            </article>

            <article className="signal-card">
              <div className="signal-card-header"><div><p className="card-label">CROSSFADER ROUTING</p><h2>Assign sources</h2></div><Settings2 size={20} /></div>
              <div className="source-list">
                <SourceSelect label="Fader A" value={assignA} onChange={(value) => updateProgram(setAssignA, "assignA", value)} accent="a" />
                <SourceSelect label="Fader B" value={assignB} onChange={(value) => updateProgram(setAssignB, "assignB", value)} accent="b" />
                <SourceSelect label="Post fader" value={assignPost} onChange={(value) => updateProgram(setAssignPost, "assignPost", value)} accent="neutral" />
              </div>
              <div className="route-line" aria-hidden="true"><span className="route-a" /><i /><span className="route-b" /></div>
            </article>

            <article className="channel-card channel-b">
              <div className="card-kicker"><span>2</span>INPUT SELECT SWITCH</div>
              <div className="segmented" role="group" aria-label="Channel 2 input type">
                {(["LINE", "PHONO"] as const).map((item) => <button key={item} className={ch2Type === item ? "active" : ""} onClick={() => updateProgram(setCh2Type, "ch2Type", item)}>{item}</button>)}
              </div>
              <div className="mini-meter"><span style={{ height: "48%" }} /><span style={{ height: "74%" }} /><span style={{ height: "88%" }} /><span style={{ height: "66%" }} /><span style={{ height: "40%" }} /></div>
            </article>
          </section>

          <section className="curve-card" id="crossfader">
            <div className="curve-copy"><p className="card-label">CROSSFADER</p><h2>Response curve</h2><p>0% gives the widest response. 100% gives the sharpest cut.</p>{hasOpenPorts && <div className={`curve-edit-status ${curveEditActive ? "active" : ""}`}><span />{curveEditActive ? "Curve edit active" : "Ready to edit"}</div>}<div className="legend"><span><i className="legend-a" />Fader A</span><span><i className="legend-b" />Fader B</span></div></div>
            <CurveGraph curveA={curveA} curveB={curveB} />
            <div className="curve-controls">
              <label htmlFor="curve-a"><span><b>Fader A</b><output htmlFor="curve-a" title={`MIDI CC ${curvePercentToMidiCC(curveA)} · width ${curvePercentToWidth(curveA).toFixed(3)}`}>{curveA}%</output></span><input id="curve-a" aria-label="Fader A curve sharpness" className="range-a" type="range" min="0" max="100" value={curveA} onFocus={() => hasOpenPorts && beginCurveEdit()} onPointerDown={() => hasOpenPorts && beginCurveEdit()} onChange={(event) => updateCurve(setCurveA, "curveA", Number(event.target.value))} onBlur={() => hasOpenPorts && endCurveEdit()} onPointerUp={() => hasOpenPorts && endCurveEdit()} onPointerCancel={() => hasOpenPorts && endCurveEdit()} /></label>
              <label htmlFor="curve-b"><span><b>Fader B</b><output htmlFor="curve-b" title={`MIDI CC ${curvePercentToMidiCC(curveB)} · width ${curvePercentToWidth(curveB).toFixed(3)}`}>{curveB}%</output></span><input id="curve-b" aria-label="Fader B curve sharpness" className="range-b" type="range" min="0" max="100" value={curveB} onFocus={() => hasOpenPorts && beginCurveEdit()} onPointerDown={() => hasOpenPorts && beginCurveEdit()} onChange={(event) => updateCurve(setCurveB, "curveB", Number(event.target.value))} onBlur={() => hasOpenPorts && endCurveEdit()} onPointerUp={() => hasOpenPorts && endCurveEdit()} onPointerCancel={() => hasOpenPorts && endCurveEdit()} /></label>
            </div>
          </section>

          <section className="section-block" id="controls">
            <div className="section-heading"><div><p className="card-label">PERFORMANCE</p><h2>Controls</h2></div><p>Fine-tune the behavior of the hardware controls.</p></div>
            <div className="control-grid">
              <article className="control-card control-card-wide">
                <div className="control-card-title"><span className="control-icon"><SlidersHorizontal size={18} /></span><div><h3>Digital Vinyl System</h3><p>Enable or disable DVS operation for each input channel.</p></div></div>
                <div className="toggle-list">
                  <div><span><b>Channel 1</b><small>{dvs1 ? "DVS enabled" : "DVS disabled"}</small></span><button className={`switch ${dvs1 ? "on" : ""}`} role="switch" aria-checked={dvs1} onClick={() => updateProgram(setDvs1, "dvs1", !dvs1)}><i /></button></div>
                  <div><span><b>Channel 2</b><small>{dvs2 ? "DVS enabled" : "DVS disabled"}</small></span><button className={`switch ${dvs2 ? "on" : ""}`} role="switch" aria-checked={dvs2} onClick={() => updateProgram(setDvs2, "dvs2", !dvs2)}><i /></button></div>
                </div>
              </article>

              <article className="control-card routing-control-card">
                <div className="control-card-title"><span className="control-icon cyan"><Headphones size={18} /></span><div><h3>Monitor routing</h3><p>Select the headphone monitor source.</p></div></div>
                <div className="select-shell select-b routing-select"><select aria-label="Headphone monitor source" value={headphoneSource} onChange={(event) => updateProgram(setHeadphoneSource, "headphoneSource", event.target.value as typeof headphoneSource)}>{["Fader A", "Fader B", "Thru", "Master"].map((source) => <option key={source}>{source}</option>)}</select><ChevronDown size={16} /></div>
              </article>

              <article className="control-card routing-control-card">
                <div className="control-card-title"><span className="control-icon"><Cable size={18} /></span><div><h3>Return routing</h3><p>Select the USB return input.</p></div></div>
                <div className="select-shell select-a routing-select"><select aria-label="USB return input" value={returnSource} onChange={(event) => updateProgram(setReturnSource, "returnSource", event.target.value as "USB 1/2" | "USB 3/4")}><option>USB 1/2</option><option>USB 3/4</option></select><ChevronDown size={16} /></div>
              </article>
            </div>

            <article className="mag-card">
              <div className="mag-copy"><div className="control-card-title"><span className="control-icon"><Radio size={18} /></span><div><h3>Magnetic switches</h3><p>Choose the outgoing MIDI type and fade-down side for the auxiliary sensors.</p></div></div>
                <div className="mag-setting"><span>MIDI output</span><div className="choice-pills">{(["CC", "NOTE"] as const).map((mode) => <button key={mode} className={magMode === mode ? "active" : ""} onClick={() => updateProgram(setMagMode, "magMode", mode)}>{mode === "CC" ? "Control change" : "MIDI note"}</button>)}</div></div>
              </div>
              <div className="switch-preview" aria-label="Magnetic switch assignment preview">
                <div className="hardware-key key-top-a"><span>A</span><small>FADE DOWN</small></div>
                <div className="hardware-key midi-key key-top-midi-a"><span>MIDI</span><small>{magMode}</small></div>
                <div className="hardware-key midi-key key-top-midi-b"><span>MIDI</span><small>{magMode}</small></div>
                <div className="hardware-key key-top-b"><span>B</span><small>FADE DOWN</small></div>

                <div className="hardware-key key-bottom-a"><span>A</span><small>FADE UP</small></div>
                <div className="hardware-key midi-key key-bottom-midi-a"><span>MIDI</span><small>{magMode}</small></div>
                <button className="hardware-key aux-key key-bottom-aux-a" aria-label={`Change auxiliary fade-down assignment from ${sensor2}`} onClick={() => updateProgram(setSensor2, "sensor2", sensor2 === "A" ? "B" : "A")}><span>aux. {sensor2}</span><small>FADE DOWN</small></button>
                <button className="hardware-key aux-key key-bottom-aux-b" aria-label={`Change auxiliary fade-down assignment from ${sensor3}`} onClick={() => updateProgram(setSensor3, "sensor3", sensor3 === "A" ? "B" : "A")}><span>aux. {sensor3}</span><small>FADE DOWN</small></button>
                <div className="hardware-key midi-key key-bottom-midi-b"><span>MIDI</span><small>{magMode}</small></div>
                <div className="hardware-key key-bottom-b"><span>B</span><small>FADE UP</small></div>
              </div>
            </article>
          </section>
          </fieldset>

          <section className="section-block device-section" id="device">
            <div className="section-heading"><div><p className="card-label">SYSTEM</p><h2>Device</h2></div><p>Connection details and preset management.</p></div>
            <article className="device-card">
              <div className="device-summary">
                <div className="device-identity">
                  <span className={`device-art ${connected ? "online" : reconnecting ? "reconnecting" : ""}`}><Usb size={25} /></span>
                  <div>
                    <h3>{hasOpenPorts || reconnecting ? connectedInputName : "JUMBLEQ Configurator demo"}</h3>
                    <p>
                      {connected
                        ? "Current settings loaded from JUMBLEQ"
                        : reconnecting
                          ? "Waiting for JUMBLEQ to reconnect via USB"
                          : midiStatus === "syncing"
                            ? `Reading device settings (${syncReceived}/${SYNC_FIELD_COUNT})`
                            : midiStatus === "unsupported"
                              ? "Web MIDI is not available in this browser"
                              : "Connect a unit to read its current settings"}
                    </p>
                  </div>
                  <span className="firmware-chip">{connected ? `${SYNC_FIELD_COUNT}/${SYNC_FIELD_COUNT} synced` : reconnecting ? "RECONNECTING" : "MIDI Ch.15"}</span>
                </div>

                {midiError && <p className="midi-error" role="alert">{midiError}</p>}

                {accessGranted && (
                  <div className="midi-port-panel">
                    <label>
                      <span>MIDI input</span>
                      <div className="select-shell">
                        <select value={selectedInputId} onChange={(event) => setSelectedInputId(event.target.value)}>
                          <option value="">Select input</option>
                          {midiInputs.map((port) => <option key={port.id} value={port.id}>{port.name}</option>)}
                        </select>
                        <ChevronDown size={16} />
                      </div>
                    </label>
                    <label>
                      <span>MIDI output</span>
                      <div className="select-shell">
                        <select value={selectedOutputId} onChange={(event) => setSelectedOutputId(event.target.value)}>
                          <option value="">Select output</option>
                          {midiOutputs.map((port) => <option key={port.id} value={port.id}>{port.name}</option>)}
                        </select>
                        <ChevronDown size={16} />
                      </div>
                    </label>
                    {!hasOpenPorts && <button className="midi-port-connect" onClick={() => void connectSelected()} disabled={!selectedInputId || !selectedOutputId}>Connect selected ports</button>}
                  </div>
                )}
              </div>

              <div className="device-actions">
                {hasOpenPorts && <button onClick={requestSync} disabled={midiStatus === "syncing"}><RefreshCw size={16} />Read from device</button>}
                <button onClick={reset} disabled={settingsLocked}><RotateCcw size={16} />Restore defaults</button>
                <button onClick={exportPreset}><Download size={16} />Export preset</button>
                <button onClick={() => presetFileInputRef.current?.click()} disabled={settingsLocked}><Upload size={16} />Import preset</button>
                <input ref={presetFileInputRef} id="preset-file" type="file" accept=".json,application/json" aria-label="Import preset file" onChange={(event) => void importPreset(event)} hidden />
              </div>
            </article>
          </section>
        </section>
      </div>

      <div className={`save-bar ${dirty ? "is-visible" : ""}`}>
        <div><span className="unsaved-dot" /><strong>Unsaved changes</strong><small>{hasOpenPorts ? "Changes are active on the device but not stored." : "Changes are active in this preview only."}</small></div>
        <button onClick={save} disabled={!hasOpenPorts} title={!hasOpenPorts ? "Connect JUMBLEQ to save settings." : undefined}><Save size={17} />Save to device</button>
      </div>
      {saved && <div className="toast"><span>✓</span> Save command sent to JUMBLEQ</div>}
      {presetNotice && <div className={`toast preset-toast ${presetNotice.type === "error" ? "is-error" : ""}`} role={presetNotice.type === "error" ? "alert" : "status"}><span>{presetNotice.type === "error" ? "!" : "✓"}</span>{presetNotice.message}</div>}
    </main>
  );
}
