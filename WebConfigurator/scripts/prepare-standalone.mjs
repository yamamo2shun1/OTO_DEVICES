import { cp, mkdir } from "node:fs/promises";
import { resolve } from "node:path";

const projectRoot = process.cwd();
const standaloneRoot = resolve(projectRoot, ".next", "standalone");

await cp(
  resolve(projectRoot, "public"),
  resolve(standaloneRoot, "public"),
  { recursive: true },
);

await mkdir(resolve(standaloneRoot, ".next"), { recursive: true });
await cp(
  resolve(projectRoot, ".next", "static"),
  resolve(standaloneRoot, ".next", "static"),
  { recursive: true },
);

console.log("Copied public and static assets into the standalone output.");
