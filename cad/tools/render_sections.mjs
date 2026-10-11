// Render the section-by-section guide: X-ray views with numbered callouts and one
// thumbnail per part, from the CAD viewer in headless Chromium (Playwright).
//
//   node cad/tools/render_sections.mjs <three_package_dir> <docs_img_dir>
//
// Writes <docs_img_dir>/sections/<key>-<n>.png, <docs_img_dir>/parts/<PID>.png and
// cad/out/viewer/sections_render.json (which view each callout landed in).
import { createServer } from "node:http";
import { readFileSync, existsSync, statSync, mkdirSync, writeFileSync } from "node:fs";
import { join, extname, resolve } from "node:path";
import { createRequire } from "node:module";

const [threeDir, imgDir] = process.argv.slice(2).map((p) => resolve(p));
const viewer = resolve(new URL("../out/viewer", import.meta.url).pathname);
const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PW || "playwright");
const TYPES = { ".html": "text/html", ".js": "text/javascript", ".json": "application/json", ".glb": "model/gltf-binary" };
const server = createServer((req, res) => {
  const url = decodeURIComponent(req.url.split("?")[0]);
  const file = url.startsWith("/three/") ? join(threeDir, url.slice(7)) : join(viewer, url === "/" ? "index.html" : url);
  if (!existsSync(file) || statSync(file).isDirectory()) { res.writeHead(404); res.end(); return; }
  let body = readFileSync(file);
  if (file.endsWith("index.html")) body = Buffer.from(String(body).replaceAll("https://cdn.jsdelivr.net/npm/three@0.160.0/", "/three/"));
  res.writeHead(200, { "content-type": TYPES[extname(file)] || "application/octet-stream" });
  res.end(body);
}).listen(0);
const base = `http://127.0.0.1:${server.address().port}/?render&tags=0`;
mkdirSync(join(imgDir, "sections"), { recursive: true });
mkdirSync(join(imgDir, "parts"), { recursive: true });

const browser = await chromium.launch({ args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"] });
const ready = (page) => page.waitForFunction(() => window.__ready === true, null, { timeout: 180000 });
const big = await browser.newPage({ viewport: { width: 1600, height: 1000 }, deviceScaleFactor: 1 });
big.on("console", (m) => { if (m.type() === "error") console.error("page:", m.text()); });
await big.goto(base); await ready(big);
const sections = await big.evaluate(() => window.__sections);
const report = [];
const only = process.env.ONLY ? process.env.ONLY.split(",") : null;   // e.g. ONLY=avionics,legs for a quick check
const thumbsOnly = process.env.THUMBS === "only";                       // THUMBS=only: just the part thumbnails
for (const S of sections.filter((s) => !thumbsOnly && (!only || only.includes(s.key)))) {
  const parts = await big.evaluate((k) => window.__prepareSection(k), S.key);
  for (let v = 0; v < S.views.length; v++) {
    const n = await big.evaluate(([k, i]) => window.__showSection(k, i), [S.key, v]);
    await big.waitForTimeout(250);
    await big.screenshot({ path: join(imgDir, "sections", `${S.key}-${v + 1}.png`) });
    console.log(`section ${S.key} view ${v + 1}: ${n} callouts`);
  }
  report.push({ key: S.key, views: S.views.length, parts });
}
if (!thumbsOnly) writeFileSync(join(viewer, "sections_render.json"), JSON.stringify(report, null, 1));

if (process.env.THUMBS === "0") { await browser.close(); server.close(); process.exit(0); }
const small = await browser.newPage({ viewport: { width: 360, height: 270 }, deviceScaleFactor: 1 });
await small.goto(base); await ready(small);
const pids = sections.filter((s) => !only || only.includes(s.key)).flatMap((s) => s.pids);
for (const pid of pids) {
  await small.evaluate((p) => window.__thumb(p), pid);
  await small.waitForTimeout(60);
  await small.screenshot({ path: join(imgDir, "parts", `${pid}.png`) });
}
console.log(`${pids.length} thumbnails`);
await browser.close();
server.close();
