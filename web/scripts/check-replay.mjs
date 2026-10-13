// Run against the read-only viewer after SectionBuild has verified a full build.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import fs from 'node:fs/promises';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = fileURLToPath(new URL('../..', import.meta.url));
const run = process.argv[2];
assert(run && /^[A-Za-z0-9_-]+$/.test(run), 'Provide one run ID, such as cube_sections_v1');
const directory = path.join(root, 'runs', run);
const prefix = process.argv[3]
  ? path.resolve(root, process.argv[3])
  : path.join(root, 'research', 'blondie', run);
const base = process.env.PEN_ART_VIEWER_URL ?? 'http://127.0.0.1:8765';
const digest = data => createHash('sha256').update(data).digest('hex');
const read = async name => JSON.parse(await fs.readFile(path.join(directory, name), 'utf8'));
assert.equal((await read('run.json')).id, run,
  'Run metadata must identify the requested directory');
const state = await read('section_build.json');
assert.equal(state.status, 'FULL_BUILD_VERIFIED', 'Full robot build must pass first');
assert.equal(state.completed_sections, state.required_sections);
const validation = await read('offline_dual_arm_validation.json');
const plan = await read('offline_dual_arm_plan.json');
const turns = plan.actions.flatMap(action => action.trajectory.edges
  .filter(edge => edge.kind === 'INDEX_TURN')
  .map(edge => ({ ...edge, start: action.start_s + edge.start_s,
    end: action.start_s + edge.end_s })));
const angleAt = time => {
  let index = plan.initial_table_index ?? 0;
  for (const edge of turns) {
    if (time < edge.start) break;
    if (time <= edge.end) {
      const u = (time - edge.start) / (edge.end - edge.start);
      return (edge.table_index + (edge.table_to_index - edge.table_index)
        * u ** 3 * (10 + u * (-15 + 6 * u))) * Math.PI / 2;
    }
    index = edge.table_to_index;
  }
  return index * Math.PI / 2;
};
assert.equal(validation.status, 'CERTIFIED_GEOMETRIC_REPLAY');
const inputs = {};
for (const name of ['offline_dual_arm_plan.json', 'offline_dual_arm_validation.json']) {
  const local = await fs.readFile(path.join(directory, name));
  const served = await fetch(`${base}/api/runs/${run}/artifacts/${name}`);
  assert(served.ok, `Viewer must serve ${name}`);
  assert.equal(digest(Buffer.from(await served.arrayBuffer())), digest(local),
    `Viewer must serve the same ${name}`);
  inputs[`runs/${run}/${name}`] = digest(local);
}
assert.equal(validation.source_plan_sha256, inputs[`runs/${run}/offline_dual_arm_plan.json`]);
const executablePath = process.env.PEN_ART_BROWSER_EXECUTABLE
  ?? (existsSync('/opt/google/chrome/chrome') ? '/opt/google/chrome/chrome' : undefined);
const browser = await chromium.launch({
  executablePath,
  headless: true,
  args: ['--no-sandbox', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
});
try {
  const page = await browser.newPage({ viewport: { width: 1500, height: 1100 } });
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(`${base}/?workbench=1&run=${run}&stage=motion`);
  await page.locator('#dual-play').waitFor();
  assert.equal(await page.locator('#run-select').inputValue(), run,
    'Viewer must select the requested run');
  assert.equal(await page.locator('#dual-rate').inputValue(), '10');
  const horizon = Number(await page.locator('#dual-seek').getAttribute('max'));
  assert(Number.isFinite(horizon) && horizon > 0);
  assert(Math.abs(horizon - validation.horizon_s) < 1e-6,
    'Viewer must play the full independently verified horizon');
  const began = Date.now();
  const observed = [];
  const observedAngles = [];
  await page.locator('#dual-play').click();
  let time = Number(await page.locator('#dual-seek').inputValue());
  assert(Number.isFinite(time) && time >= 0 && time < horizon,
    'Playback must start before the completed state');
  while (time < horizon - 1e-6) {
    await page.waitForTimeout(1000);
    time = Number(await page.locator('#dual-seek').inputValue());
    assert(Number.isFinite(time));
    assert(!observed.length || time >= observed.at(-1), 'Playback clock regressed');
    observed.push(time);
    if (plan.turntable) {
      const shown = Number(await page.locator('#table-angle').getAttribute('data-radians'));
      // Playback can advance one UI tick between the clock and angle reads.
      const latest = Number(await page.locator('#dual-seek').inputValue());
      const bound = plan.turntable.velocity_rad_s * Math.abs(latest - time) + 1e-7;
      assert(Math.abs(shown - angleAt(time)) <= bound, 'Displayed plate angle disagrees with plan');
      observedAngles.push(shown);
    }
    assert(Date.now() - began < horizon * 180 + 15000, 'Playback timed out');
    assert.equal(errors.length, 0, errors.join('\n'));
  }
  const badge = await page.locator('#badge').innerText();
  assert.equal(badge, 'One-arm geometric replay passed');
  assert.equal(await page.locator('#dual-rate').inputValue(), '10');
  assert.equal(errors.length, 0, errors.join('\n'));
  if (plan.turntable) {
    assert(Math.abs(Number(await page.locator('#table-angle').getAttribute('data-radians'))
      - plan.turntable.home_index * Math.PI / 2) < 1e-8, 'Plate must return home');
    if (turns.length) assert(observedAngles.some(angle => Math.abs(angle) > 0.1),
      'Playback must include actual plate rotation');
  }
  await fs.mkdir(path.dirname(prefix), { recursive: true });
  if ((await page.locator('#schedule-view').innerText()) === 'Show full robot cell') {
    await page.locator('#schedule-view').click();
  }
  assert.equal(await page.locator('#schedule-view').innerText(), 'Focus sculpture');
  await page.waitForTimeout(500);
  await page.screenshot({ path: `${prefix}_full_cell.png`, fullPage: true });
  await page.locator('#schedule-view').click();
  await page.waitForTimeout(500);
  await page.screenshot({ path: `${prefix}_completed.png`, fullPage: true });
  const receipt = {
    status: 'PASSED_BROWSER_REPLAY', run, rate: 10, horizon_s: horizon,
    wall_seconds: (Date.now() - began) / 1000, observed_times_s: observed, errors,
    indexed_turns: turns.length, observed_table_angles_rad: observedAngles,
    final_clock: await page.locator('#dual-clock').innerText(), badge, inputs,
    screenshots: [`${prefix}_full_cell.png`, `${prefix}_completed.png`],
    simulation_only: true,
  };
  await fs.writeFile(`${prefix}_browser_replay.json`, `${JSON.stringify(receipt, null, 2)}\n`);
  console.log(JSON.stringify(receipt));
} finally {
  await browser.close();
}
