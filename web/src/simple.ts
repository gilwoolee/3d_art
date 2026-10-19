// One page: the textured model, the wireframe strokes and the robot's saved
// motion, side by side. It only reads saved files; nothing here plans or checks.
import './style.css';
import { Viewer, type BuildRiser, type CellArmView, type Curve, type RobotRig } from './viewer';
import { coordinatedPose, tableAngleAt, type CoordinatedPlan, type IndexedTable } from './timing';

interface RunRow { id: string; name: string; updated_at: string }
interface RunDetail extends RunRow { artifacts: string[] }
interface CellManifest {
  arms: (Omit<CellArmView, 'rig' | 'url'> & { robot_manifest: string; robot_visual: string })[];
  world_from_build: number[][];
  environment: { platform_top_z_m?: number; table_z_m: number; platform_size_m: number[]; build_riser?: BuildRiser };
  turntable?: IndexedTable;
}
interface Validation { status: string; source_plan_sha256?: string; collision_scope?: string }

const escape = (s: unknown) => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]!));
async function json<T>(url: string): Promise<T> {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${response.status}: ${url}`);
  return response.json();
}
const minutes = (seconds: number) => `${(seconds / 60).toFixed(1)} min`;

const app = document.querySelector<HTMLElement>('#app')!;
app.innerHTML = `<div class="simple">
  <header class="simple-bar">
    <strong>Pen Art</strong>
    <select id="simple-run" aria-label="Saved run"></select>
    <span id="simple-status" class="simple-status"></span>
    <a class="simple-link" id="simple-workbench">Full workbench ↗</a>
  </header>
  <main class="simple-panels">
    <section><h2>0 · Original picture</h2><div class="simple-photo"><img id="simple-photo" alt="The object's original picture"></div><p id="simple-photo-note"></p></section>
    <section><h2>1 · Original model</h2><div class="simple-view" id="simple-model"></div><p id="simple-model-note"></p></section>
    <section><h2>2 · Wireframe strokes<label class="simple-toggle"><input type="checkbox" id="simple-ghost" checked> Show statue</label></h2><div class="simple-view" id="simple-strokes"></div><p id="simple-strokes-note"></p></section>
    <section><h2>3 · Robot motion</h2><div class="simple-view" id="simple-robot"></div><p id="simple-robot-note"></p></section>
  </main>
  <footer class="simple-controls">
    <button id="simple-play" disabled>Play</button>
    <select id="simple-rate" aria-label="Playback speed"><option value="1">1×</option><option value="10">10×</option><option value="30" selected>30×</option><option value="100">100×</option></select>
    <input id="simple-seek" type="range" min="0" max="1" step="any" value="0" disabled aria-label="Build time">
    <span id="simple-clock" class="mono">–</span>
    <button id="simple-focus" disabled>Show full robot cell</button>
  </footer>
</div>`;
const $ = <T extends HTMLElement = HTMLElement>(selector: string) => app.querySelector<T>(selector)!;
const views = {
  model: new Viewer($('#simple-model'), () => {}),
  strokes: new Viewer($('#simple-strokes'), () => {}),
  robot: new Viewer($('#simple-robot'), () => {}),
};
let request = 0;
let timer: ReturnType<typeof setInterval> | null = null;
const stop = () => { if (timer) clearInterval(timer); timer = null; $('#simple-play').textContent = 'Play'; };
$<HTMLInputElement>('#simple-ghost').addEventListener('change', event => views.strokes.setSource((event.target as HTMLInputElement).checked, true));

async function show(runId: string) {
  const current = ++request;
  stop();
  const url = (file: string) => `/api/runs/${runId}/artifacts/${file}`;
  const status = (text: string) => { if (current === request) $('#simple-status').textContent = text; };
  for (const id of ['#simple-photo-note', '#simple-model-note', '#simple-strokes-note', '#simple-robot-note']) $(id).textContent = '';
  $<HTMLButtonElement>('#simple-play').disabled = $<HTMLInputElement>('#simple-seek').disabled = $<HTMLButtonElement>('#simple-focus').disabled = true;
  $('#simple-clock').textContent = '–';
  $<HTMLAnchorElement>('#simple-workbench').href = `/?workbench=1&run=${encodeURIComponent(runId)}`;
  history.replaceState(null, '', `/?run=${encodeURIComponent(runId)}`);
  status('Loading…');
  const run = await json<RunDetail>(`/api/runs/${runId}`);
  if (current !== request) return;
  const has = (file: string) => run.artifacts.includes(file);

  // 0. The object's original picture from objs/, when there is one.
  const photo = $<HTMLImageElement>('#simple-photo');
  photo.hidden = false;
  photo.onload = () => { if (current === request) $('#simple-photo-note').textContent = 'Picture from objs/.'; };
  photo.onerror = () => {
    if (current !== request) return;
    photo.hidden = true;
    $('#simple-photo-note').textContent = 'No picture in objs/ for this model.';
  };
  photo.src = `/api/runs/${runId}/picture`;

  // 1. The original model, textured when the run kept it.
  // (The run's file list names only its JSON files, so the models are simply tried.)
  views.model.setCurves([]);
  const progress = (fraction: number) => { if (current === request) $('#simple-model-note').textContent = fraction < 1 ? `Loading ${(fraction * 100).toFixed(0)}%` : ''; };
  const note = (text: string) => { if (current === request) $('#simple-model-note').textContent = text; };
  const modelDone = views.model.load(url('normalized.glb'), progress).then(() => note('Textured source model.'))
    .catch(() => views.model.load(url('geometry.glb'), progress).then(() => note('Geometry only; this run kept no textured model.')))
    .catch(() => note('This run has no saved model.'));

  // 2. The strokes the robot is asked to draw, over the faded shape.
  const strokesDone = (async () => {
    if (!has('construction_plan.json')) { $('#simple-strokes-note').textContent = 'This run has no saved strokes.'; return; }
    const [construction] = await Promise.all([
      json<{ actions: (Curve & { parent_curve?: string })[] }>(url('construction_plan.json')),
      views.strokes.load(url('geometry.glb'), () => {}),
    ]);
    if (current !== request) return;
    views.strokes.setSource($<HTMLInputElement>('#simple-ghost').checked, true);
    views.strokes.setCurves(construction.actions);
    const contours = construction.actions.filter(a => a.parent_curve?.startsWith('contour-')).length;
    const zigzags = construction.actions.filter(a => a.parent_curve?.startsWith('zigzag-')).length;
    const length = construction.actions.reduce((sum, a) => sum + (a.length_m ?? 0), 0);
    $('#simple-strokes-note').textContent = `${construction.actions.length} strokes (${contours} contours, ${zigzags} zigzags), ${length.toFixed(1)} m of wire.`;
  })();

  // 3. The saved robot motion, when the run has a plan.
  const robotDone = (async () => {
    views.robot.clearCell();
    views.robot.clearSequence();
    const planFile = ['offline_dual_arm_plan.json', 'dual_arm_repair_plan.json', 'dual_arm_plan.json'].find(has);
    if (!has('cell_manifest.json') || !planFile) { $('#simple-robot-note').textContent = 'This run has no saved robot motion.'; return; }
    const cell = await json<CellManifest>(url('cell_manifest.json'));
    const arms: CellArmView[] = await Promise.all(cell.arms.map(async arm => ({
      ...arm, rig: await json<RobotRig>(url(arm.robot_manifest)), url: url(arm.robot_visual),
    })));
    if (current !== request) return;
    await views.robot.loadCell(arms, { url: url('geometry.glb'), world_from_build: cell.world_from_build });
    if (current !== request) return;
    const origin = cell.world_from_build.map(row => row[3]);
    views.robot.setPlatform(origin.slice(0, 2), cell.environment.platform_top_z_m ?? origin[2], cell.environment.table_z_m,
      ...cell.environment.platform_size_m as [number, number], cell.environment.build_riser);
    if (cell.turntable) views.robot.setTurntable(cell.turntable, cell.environment.build_riser);
    views.robot.setCellReferenceVisible(false);
    $('#simple-robot-note').textContent = 'Loading the saved motion…';
    const bytes = await (await fetch(url(planFile))).arrayBuffer();
    if (current !== request) return;
    const plan = JSON.parse(new TextDecoder().decode(bytes)) as CoordinatedPlan;
    // Say whether this exact file is the one the separate verification passed.
    const validationFile = planFile.replace('_plan.json', '_validation.json');
    let verified = false;
    if (has(validationFile)) {
      const validation = await json<Validation>(url(validationFile));
      const digest = crypto.subtle
        ? [...new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))].map(b => b.toString(16).padStart(2, '0')).join('')
        : null;
      verified = validation.status === 'CERTIFIED_GEOMETRIC_REPLAY' && validation.collision_scope === 'full_robot'
        && digest !== null && validation.source_plan_sha256 === digest;
    }
    if (current !== request) return;
    const total = plan.modeled_duration_s;
    const rows = plan.actions.flatMap(action => action.trajectory.edges
      .filter(edge => edge.kind === 'DRAW' && edge.tcp0_m && edge.tcp1_m)
      .map(edge => ({ action, edge, start: action.start_s + edge.start_s, end: action.start_s + edge.end_s })))
      .sort((a, b) => a.start - b.start);
    const strokes = new Set(plan.actions.filter(a => !a.auxiliary).map(a => a.stroke_id)).size;
    $('#simple-robot-note').textContent = `${strokes} of ${plan.requested_strokes} strokes, ${minutes(total)} of drawing. `
      + (verified ? 'This plan passed the separate full-robot verification (simulation only).' : 'Not verified: no matching verification report for this plan.');
    const seek = $<HTMLInputElement>('#simple-seek');
    seek.max = String(total);
    const draw = (time: number) => {
      time = Math.max(0, Math.min(total, time));
      seek.value = String(time);
      const angle = tableAngleAt(plan, time);
      if (plan.turntable) views.robot.setTurntableAngle(angle);
      for (const arm of arms) {
        const moves = plan.actions.filter(a => a.arm === arm.id);
        const motion = moves.find(a => a.start_s <= time && time < a.end_s);
        const past = moves.filter(a => a.end_s <= time).at(-1);
        views.robot.setCellArmPose(arm.id, motion ? coordinatedPose(motion, time)
          : past ? coordinatedPose(past, past.end_s) : (plan.initial_positions?.[arm.id] ?? arm.initial_positions));
      }
      // Wire stays on the plate, so earlier wire turns with the table.
      const placed = (point: number[], index?: number) => {
        if (!plan.turntable || index === undefined) return point;
        const axis = plan.turntable.world_from_axis;
        const x = point[0] - axis[0][3], y = point[1] - axis[1][3];
        const turn = angle - index * Math.PI / 2, c = Math.cos(turn), s = Math.sin(turn);
        return [axis[0][3] + c * x - s * y, axis[1][3] + s * x + c * y, point[2]];
      };
      const paths: number[][][] = [];
      for (const row of rows) {
        if (row.start > time) break;
        paths.push([placed(row.edge.tcp0_m!, row.edge.table_index), row.end <= time
          ? placed(row.edge.tcp1_m!, row.edge.table_index)
          : views.robot.setCellArmPose(row.action.arm, coordinatedPose(row.action, time))]);
      }
      views.robot.setDepositedPaths(paths);
      $('#simple-clock').textContent = `${minutes(time)} / ${minutes(total)}`;
    };
    const play = $<HTMLButtonElement>('#simple-play'), focus = $<HTMLButtonElement>('#simple-focus');
    play.onclick = () => {
      if (timer) { stop(); return; }
      if (Number(seek.value) >= total) draw(0);
      play.textContent = 'Pause';
      let previous = performance.now();
      timer = setInterval(() => {
        const now = performance.now();
        const next = Number(seek.value) + (now - previous) / 1000 * Number($<HTMLSelectElement>('#simple-rate').value);
        previous = now;
        draw(next);
        if (next >= total) stop();
      }, 40);
    };
    seek.oninput = () => draw(Number(seek.value));
    const allPoints = rows.flatMap(row => [row.edge.tcp0_m!, row.edge.tcp1_m!]);
    let close = true;
    const frame = () => { if (close && allPoints.length) views.robot.focusPoints(allPoints); else views.robot.focus('cell'); focus.textContent = close ? 'Show full robot cell' : 'Focus on the sculpture'; };
    focus.onclick = () => { close = !close; frame(); };
    play.disabled = seek.disabled = focus.disabled = false;
    frame();
    draw(0);
  })();
  const results = await Promise.allSettled([modelDone, strokesDone, robotDone]);
  const failed = results.find(result => result.status === 'rejected') as PromiseRejectedResult | undefined;
  status(failed ? `Could not load everything: ${failed.reason instanceof Error ? failed.reason.message : failed.reason}` : escape(run.name ?? runId));
}

async function start() {
  const runs = (await json<RunRow[]>('/api/runs')).sort((a, b) => b.updated_at.localeCompare(a.updated_at));
  const wanted = new URLSearchParams(location.search).get('run');
  // A run missing from runs/viewer_runs.json still opens by its address.
  if (wanted && !runs.some(run => run.id === wanted)) {
    const extra = await json<RunRow>(`/api/runs/${encodeURIComponent(wanted)}`).catch(() => undefined);
    if (extra) runs.unshift(extra);
  }
  const select = $<HTMLSelectElement>('#simple-run');
  select.innerHTML = runs.map(run => `<option value="${escape(run.id)}">${escape(run.id)}</option>`).join('');
  if (wanted && runs.some(run => run.id === wanted)) select.value = wanted;
  select.onchange = () => show(select.value).catch(error => { $('#simple-status').textContent = String(error); });
  if (select.value) await show(select.value);
  else $('#simple-status').textContent = 'No saved runs.';
}
start().catch(error => { $('#simple-status').textContent = String(error); });
