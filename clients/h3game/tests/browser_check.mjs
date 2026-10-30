// Headless Chromium check of the patched h3game/web/index.html.
//
// Loads the page from a running (patched) bridge, records the seat POST
// bodies for ~2 s and checks: frame_seq present and strictly increasing,
// frame_age_ms small, owner set. Then fakes document.hidden = true right after
// a rAF frame that has not been POSTed yet, and checks that every hidden POST,
// from the first one on, repeats the last POSTed frame_seq with zeros. With
// SERVER_URL and ADMIN_TOKEN it first holds pitch=+1 in live mode, ARMs and
// STARTs, and checks the server never moves toward center after the hide
// (STOPPING -> BRAKED_DISARMED at the held counts). Also probes whether a real
// background tab stops rAF (reported, not asserted: headless Chromium may keep
// rendering).
//
// Usage:
//   BRIDGE_URL=http://127.0.0.1:8891 [SERVER_URL=http://127.0.0.1:8892 ADMIN_TOKEN=...] \
//   [PLAYWRIGHT_MODULE=/path/to/node_modules/playwright] [CHROMIUM_PATH=/path/to/chrome] \
//   node browser_check.mjs
// SERVER_URL (bridge in server mode): also checks the server got fresh frame_seq poses.
// Prints one JSON line with the results; exit code 0 = all checks passed.

import { createRequire } from 'node:module';
import path from 'node:path';

const require = createRequire(import.meta.url);
const pwPath = process.env.PLAYWRIGHT_MODULE ? path.resolve(process.env.PLAYWRIGHT_MODULE) : 'playwright';
const { chromium } = require(pwPath);

const BRIDGE = process.env.BRIDGE_URL;
const SERVER = process.env.SERVER_URL || null;
const ADMIN_TOKEN = process.env.ADMIN_TOKEN || null;
if (!BRIDGE) {
  console.error('BRIDGE_URL is required');
  process.exit(2);
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const failures = [];
const check = (ok, msg) => { if (!ok) failures.push(msg); };
const status = async () => (await fetch(SERVER + '/v1/status')).json();
const admin = async (p) => (await fetch(SERVER + p, { method: 'POST', headers: { Authorization: 'Bearer ' + ADMIN_TOKEN } })).status;
const waitRig = async (want, ms = 5000) => {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) { if ((await status()).rig_state === want) return true; await sleep(25); }
  failures.push(`rig_state ${want} not reached`);
  return false;
};

const launch = { headless: true };
if (process.env.CHROMIUM_PATH) launch.executablePath = process.env.CHROMIUM_PATH;
const browser = await chromium.launch(launch);
const result = {};
try {
  const ctx = await browser.newContext();
  const page = await ctx.newPage();
  const posts = [];
  page.on('request', (req) => {
    if (req.method() !== 'POST' || new URL(req.url()).pathname !== '/') return;
    try { posts.push({ t: Date.now(), body: JSON.parse(req.postData() || '{}') }); } catch (e) { /* ignore */ }
  });
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));

  await page.goto(BRIDGE + '/', { waitUntil: 'load' });
  await sleep(2000);

  const visible = posts.slice();
  const seqs = visible.map((p) => p.body.frame_seq);
  const ages = visible.map((p) => p.body.frame_age_ms);
  result.visible_posts = visible.length;
  result.frame_seq_first_last = [seqs[0], seqs[seqs.length - 1]];
  result.frame_age_ms_max = Math.max(...ages.filter((a) => typeof a === 'number'));
  result.owner = visible.length ? visible[0].body.owner : null;
  check(visible.length >= 30, `expected ~60 POSTs in 2 s, got ${visible.length}`);
  check(seqs.every((s) => Number.isInteger(s) && s >= 0), 'frame_seq missing or not an integer');
  // Only the first POSTs may repeat 0 before the first rAF frame.
  const later = seqs.filter((s) => s > 0);
  check(later.length >= visible.length - 5, 'frame_seq did not start advancing');
  check(later.every((s, i) => i === 0 || s > later[i - 1]), 'frame_seq not strictly increasing between POSTs');
  check(ages.every((a) => typeof a === 'number' && a >= 0 && a < 100), `frame_age_ms not small: max ${result.frame_age_ms_max}`);
  check(Number.isInteger(result.owner) && result.owner > 0, 'owner not substituted by the bridge');

  // Fake a hidden tab right after a completed rAF frame, before the next POST:
  // frameSeq is then ahead of the last POSTed frame_seq. The first hidden POST
  // must repeat the last POSTed frame_seq (with zeros); a new frame_seq with
  // zeros would look like a fresh "go to center" frame to the server.
  // The page's own POSTs are recorded in-page so "last POSTed" is exact.
  await page.evaluate(() => {
    window.__seat = [];
    const f = window.fetch;
    window.fetch = (u, o) => {
      if (u === '/' && o && o.method === 'POST') { try { window.__seat.push(JSON.parse(o.body)); } catch (e) { /* ignore */ } }
      return f(u, o);
    };
  });
  const hideAfterFrame = () => page.evaluate(() => new Promise((res) => requestAnimationFrame(() => setTimeout(() => {
    const sent = window.__seat.length ? window.__seat[window.__seat.length - 1].frame_seq : null;
    Object.defineProperty(document, 'hidden', { configurable: true, get: () => true });
    document.dispatchEvent(new Event('visibilitychange'));
    res({ n: window.__seat.length, sent, frameSeq });
  }, 0))));
  const unhide = () => page.evaluate(() => { delete document.hidden; document.dispatchEvent(new Event('visibilitychange')); });

  // Server mode with an admin token: hold a pose while ACTIVE, then hide.
  let held = null;
  if (SERVER && ADMIN_TOKEN) {
    await page.click('#bLive');
    await page.keyboard.down('ArrowDown');   // live mode: target pitch = +1
    await sleep(1500);
    check(await admin('/v1/arm') === 200, 'ARM failed');
    await waitRig('HOLDING');
    check(await admin('/v1/start') === 200, 'START failed');
    await waitRig('ACTIVE');
    await sleep(1500);                       // smoothing and axes settle
    const st = await status();
    held = { rig: st.rig_state, targets: st.axes.map((a) => a.target_count), counts: st.axes.map((a) => a.actual_count) };
    result.active_before_hide = held;
    check(held.rig === 'ACTIVE' && Math.abs(held.targets[0]) > 300, `not ACTIVE with a held pose: ${JSON.stringify(held)}`);
  }

  // Retry until the race is set up (hide while frameSeq > last POSTed frame_seq).
  let hide = null;
  for (let i = 0; i < 10; i++) {
    const h = await hideAfterFrame();
    if (h.sent !== null && h.frameSeq > h.sent) { hide = h; break; }
    await unhide();                          // well inside source_freshness_ms: stays ACTIVE
    await sleep(50);
  }
  result.hide = hide;
  check(hide !== null, 'could not hide with an unsent frame pending');

  let samples = [];
  if (held) {
    // While still ACTIVE, the targets must stay at the held pose (no move toward center).
    const t0 = Date.now();
    let st;
    do {
      st = await status();
      samples.push({ dt: Date.now() - t0, rig: st.rig_state, targets: st.axes.map((a) => a.target_count) });
      await sleep(20);
    } while (st.rig_state === 'ACTIVE' && Date.now() - t0 < 3000);
    await waitRig('BRAKED_DISARMED');
    const fin = await status();
    const active = samples.filter((x) => x.rig === 'ACTIVE');
    const drift = Math.max(0, ...active.map((x) => Math.max(...x.targets.map((t, i) => Math.abs(t - held.targets[i])))));
    const finalDrift = Math.max(...fin.axes.map((a, i) => Math.abs(a.actual_count - held.counts[i])));
    result.after_hide = { active_samples: active.length, max_target_drift_while_active: drift,
      final_counts: fin.axes.map((a) => a.actual_count), final_drift: finalDrift, last_frame_seq: fin.session.last_frame_seq };
    check(drift <= 3, `targets moved while ACTIVE after hide: ${drift} counts`);
    check(finalDrift <= 25, `final counts not held after hide: drift ${finalDrift}`);
    check(hide === null || fin.session.last_frame_seq === hide.sent,
      `server accepted a frame_seq sent while hidden: ${fin.session.last_frame_seq} vs ${hide && hide.sent}`);
    await page.keyboard.up('ArrowDown');
  } else {
    await sleep(700);
  }

  const seat = await page.evaluate(() => window.__seat);
  const hidden = hide ? seat.slice(hide.n) : [];
  const hseqs = hidden.map((b) => b.frame_seq);
  result.hidden_posts = hidden.length;
  result.hidden_frame_seq = [...new Set(hseqs)];
  result.first_hidden_post = hidden[0] || null;
  check(hidden.length >= 10, `expected POSTs while hidden, got ${hidden.length}`);
  check(hide === null || hseqs.every((s) => s === hide.sent),
    `hidden POST frame_seq ${result.hidden_frame_seq} != last POSTed ${hide && hide.sent}`);
  check(hidden.every((b) => b.pitch === 0 && b.roll === 0 && b.yaw === 0), 'hidden POSTs not zero');
  check(hidden.length < 2 || hidden[hidden.length - 1].frame_age_ms > hidden[0].frame_age_ms,
    'frame_age_ms does not grow while frames are held');
  await unhide();

  // Probe: does a real background tab stop rAF here? (info only)
  const before = await page.evaluate(() => frameSeq);
  const other = await ctx.newPage();
  await other.goto('about:blank');
  await other.bringToFront();
  await sleep(800);
  const after = await page.evaluate(() => [frameSeq, document.visibilityState]);
  result.background_tab_probe = { frames_advanced: after[0] - before, visibility_state: after[1] };
  await other.close();

  if (SERVER) {
    const st = await (await fetch(SERVER + '/v1/status')).json();
    result.server_session = st.session;
    check(st.session && st.session.freshness_mode === 'frame_seq', 'server freshness_mode is not frame_seq');
    check(st.session && st.session.last_frame_seq > 0, 'server never accepted a frame_seq');
  }
  result.page_errors = errors;
} finally {
  await browser.close();
}
result.failures = failures;
console.log(JSON.stringify(result));
process.exit(failures.length ? 1 : 0);
