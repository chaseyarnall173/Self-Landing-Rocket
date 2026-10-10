// Render still images of the CAD viewer with headless Chromium (Playwright).
//
//   node cad/tools/render_views.mjs <three_package_dir> <out_png_dir>
//
// <three_package_dir> is a local copy of the three@0.160.0 npm package (the
// page normally loads it from cdn.jsdelivr.net). Run `python cad/build.py` first.
import { createServer } from "node:http";
import { readFileSync, existsSync, statSync } from "node:fs";
import { join, extname, resolve } from "node:path";
import { createRequire } from "node:module";

const [threeDir, outDir] = process.argv.slice(2).map((p) => resolve(p));
const viewer = resolve(new URL("../out/viewer", import.meta.url).pathname);
const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PW || "playwright");

const TYPES = { ".html": "text/html", ".js": "text/javascript", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = createServer((req, res) => {
  const url = decodeURIComponent(req.url.split("?")[0]);
  let file = url.startsWith("/three/") ? join(threeDir, url.slice(7)) : join(viewer, url === "/" ? "index.html" : url);
  if (!existsSync(file) || statSync(file).isDirectory()) { res.writeHead(404); res.end(); return; }
  let body = readFileSync(file);
  if (file.endsWith("index.html")) body = Buffer.from(String(body).replaceAll("https://cdn.jsdelivr.net/npm/three@0.160.0/", "/three/"));
  res.writeHead(200, { "content-type": TYPES[extname(file)] || "application/octet-stream" });
  res.end(body);
}).listen(0);
const port = server.address().port;

const SHOTS = [
  ["cad_flight", "mode=flight&view=overview&tags=1&horizontal=1"],
  ["cad_landing", "mode=landing&view=landed&tags=0"],
  ["cad_legs", "mode=landing&view=legs&tags=0"],
  ["cad_exploded", "mode=flight&view=exploded&tags=0&horizontal=1"],
  ["cad_section", "mode=flight&view=section&tags=1&horizontal=1"],
  ["cad_airbrakes", "mode=landing&view=airbrakes&tags=0"],
  ["cad_avionics", "mode=flight&view=avionics&tags=0"],
  ["cad_chute", "mode=flight&view=chute&tags=0"],
  ["cad_motorbay", "mode=flight&view=motorbay&tags=0"],
  ["cad_throttle", "mode=flight&view=throttle&tags=0"],
];
const browser = await chromium.launch({ args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"] });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 }, deviceScaleFactor: 1 });
page.on("console", (m) => { if (m.type() === "error") console.error("page:", m.text()); });
for (const [name, q] of SHOTS) {
  await page.goto(`http://127.0.0.1:${port}/?render&${q}`);
  await page.waitForFunction(() => window.__ready === true, null, { timeout: 120000 });
  await page.waitForTimeout(600);
  await page.screenshot({ path: join(outDir, name + ".png") });
  console.log("rendered", name);
}
await browser.close();
server.close();
