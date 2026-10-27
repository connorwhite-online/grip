// node render.mjs config.json out.png
// config: { width, height, view: {azim, elev, zoom}, parts: [{file, color, opacity, metalness}] }
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const [cfgPath, out] = process.argv.slice(2);
const cfg = JSON.parse(fs.readFileSync(cfgPath, 'utf8'));
const here = path.dirname(new URL(import.meta.url).pathname);
const files = cfg.parts.map(p => path.resolve(path.dirname(cfgPath), p.file));

const server = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split('?')[0]);
  let f;
  if (u.startsWith('/part/')) f = files[Number(u.slice(6))];
  else f = path.join(here, u === '/' ? 'index.html' : u);
  if (!f || !fs.existsSync(f)) { res.writeHead(404); return res.end(); }
  const type = f.endsWith('.js') ? 'text/javascript' : f.endsWith('.html') ? 'text/html' : 'application/octet-stream';
  res.writeHead(200, { 'content-type': type });
  fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, r));
const port = server.address().port;
const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: cfg.width || 1200, height: cfg.height || 1400 } });
page.on('console', m => { if (m.type() === 'error') console.error(m.text()); });
await page.goto(`http://localhost:${port}/`);
await page.waitForFunction(() => window.ready === true);
const parts = cfg.parts.map((p, i) => ({ ...p, url: `/part/${i}` }));
await page.evaluate(c => window.renderScene(c), { ...cfg, parts });
await page.locator('canvas').screenshot({ path: out });
await browser.close();
server.close();
