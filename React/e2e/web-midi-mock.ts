import type { Page } from "@playwright/test";

type JumbleqMidiMock = {
  messages: number[][];
  clearMessages: () => void;
  disconnect: () => void;
  reconnect: () => void;
};

declare global {
  interface Window {
    __jumbleqMidiMock: JumbleqMidiMock;
  }
}

export async function installWebMidiMock(page: Page) {
  await page.addInitScript(() => {
    type MidiState = "connected" | "disconnected";
    type MidiMessageHandler = ((event: { data: Uint8Array }) => void) | null;

    const configMessages = [
      [0xce, 0],
      [0xce, 3],
      [0xce, 7],
      [0xce, 8],
      [0xce, 14],
      [0xce, 17],
      [0xce, 18],
      [0xce, 20],
      [0xce, 23],
      [0xce, 27],
      [0xce, 28],
      [0xce, 123],
      [0xbe, 20, 32],
      [0xbe, 21, 95],
    ];
    const sentMessages: number[][] = [];
    const stateListeners = new Set<() => void>();

    const input = {
      id: "jumbleq-input",
      manufacturer: "Microsoft Corporation",
      name: "JUMBLEQ MIDI",
      state: "connected" as MidiState,
      onmidimessage: null as MidiMessageHandler,
      async open() { return input; },
      async close() { return input; },
    };

    const output = {
      id: "jumbleq-output",
      manufacturer: "Microsoft Corporation",
      name: "JUMBLEQ MIDI",
      state: "connected" as MidiState,
      async open() { return output; },
      async close() { return output; },
      send(data: Uint8Array | number[]) {
        const message = Array.from(data);
        sentMessages.push(message);
        if (message[0] === 0xce && message[1] === 126) {
          window.setTimeout(() => {
            for (const configMessage of configMessages) {
              input.onmidimessage?.({ data: new Uint8Array(configMessage) });
            }
          }, 0);
        }
      },
    };

    const access = {
      inputs: new Map([[input.id, input]]),
      outputs: new Map([[output.id, output]]),
      addEventListener(type: string, listener: () => void) {
        if (type === "statechange") stateListeners.add(listener);
      },
      removeEventListener(type: string, listener: () => void) {
        if (type === "statechange") stateListeners.delete(listener);
      },
    };

    const notifyStateChange = () => {
      for (const listener of stateListeners) listener();
    };

    Object.defineProperty(navigator, "requestMIDIAccess", {
      configurable: true,
      value: async () => access,
    });

    window.__jumbleqMidiMock = {
      messages: sentMessages,
      clearMessages() {
        sentMessages.length = 0;
      },
      disconnect() {
        input.state = "disconnected";
        output.state = "disconnected";
        notifyStateChange();
      },
      reconnect() {
        input.state = "connected";
        output.state = "connected";
        notifyStateChange();
      },
    };
  });
}

export async function midiMessages(page: Page) {
  return page.evaluate(() => window.__jumbleqMidiMock.messages);
}

export async function clearMidiMessages(page: Page) {
  await page.evaluate(() => window.__jumbleqMidiMock.clearMessages());
}

export async function disconnectMockDevice(page: Page) {
  await page.evaluate(() => window.__jumbleqMidiMock.disconnect());
}

export async function reconnectMockDevice(page: Page) {
  await page.evaluate(() => window.__jumbleqMidiMock.reconnect());
}
