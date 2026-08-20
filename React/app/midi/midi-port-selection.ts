export type MidiPortDescriptor = {
  id: string;
  manufacturer?: string;
  name?: string;
};

export type MidiPortIdentity = {
  id: string;
  manufacturer: string;
  name: string;
};

function normalized(value?: string) {
  return value?.trim().toLowerCase() ?? "";
}

export function midiPortIdentity(port: MidiPortDescriptor): MidiPortIdentity {
  return {
    id: port.id,
    manufacturer: port.manufacturer?.trim() ?? "",
    name: port.name?.trim() ?? "",
  };
}

export function isJumbleqPort(port: MidiPortDescriptor) {
  return `${port.manufacturer ?? ""} ${port.name ?? ""}`.toLowerCase().includes("jumbleq");
}

export function selectInitialMidiPort<Port extends MidiPortDescriptor>(ports: readonly Port[]): Port | null {
  return ports.find(isJumbleqPort) ?? (ports.length === 1 ? ports[0] : null);
}

export function selectReconnectMidiPort<Port extends MidiPortDescriptor>(
  ports: readonly Port[],
  target: MidiPortIdentity,
): Port | null {
  const sameId = ports.find((port) => port.id === target.id);
  if (sameId) return sameId;

  const targetName = normalized(target.name);
  if (!targetName) return null;
  const sameName = ports.filter((port) => normalized(port.name) === targetName);
  if (sameName.length === 0) return null;

  const targetManufacturer = normalized(target.manufacturer);
  if (targetManufacturer) {
    const sameManufacturer = sameName.find((port) => normalized(port.manufacturer) === targetManufacturer);
    if (sameManufacturer) return sameManufacturer;
  }

  return sameName.length === 1 ? sameName[0] : null;
}
