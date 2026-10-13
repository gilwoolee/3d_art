import './style.css';
import { Viewer, curveColors, displayRole, type Curve, type Junction, type Deformation, type RobotRig, type ToolModel, type CellArmView, type BuildRiser } from './viewer';
import { timedPose, coordinatedPose, coordinatedTimeline, replayProgressLabel, replayState, playbackReady, tableAngleAt, type IndexedTable, type TimedSchedule, type CoordinatedPlan, type CoordinatedAction } from './timing';
import { renderMotionReuse, type MotionReuse } from './motion-reuse';

interface SmoothAudit {
  status:string; requested_operations:number; source_schedule_sha256:string;
  operations:{id:string;stroke_id:string|null;kind:string;status:string;tracking_status:string;
    pieces_checked:number;position_error_bound_m:number;orientation_error_bound_rad:number;
    witnesses:{positions:number[];pairs:{first:string;second:string}[]}[]}[];
  limitations:string[];
}

interface Stage { status: string; artifact?:string; message?: string; budgets?: number[]; curve_count?:number }
interface CellConfig { drawing_pen_tilt_deg?:number; drawing_orientation_policy?:string; setup_description?:string }
interface MountAssessment {
  status:string;limitations:string[];drawing_pen_tilt_deg?:number;
  candidates:{id:string;label:string;world_from_base:Record<string,number[][]>;
    pose_count:number;requested_pose_count:number;clear_pose_count:number;minimum_best_joint_margin:number;
    arms:Record<string,{index:number;clear_solutions:{positions:number[]}[]}[]>;
    pose_pairs:{samples:number[];positions:Record<string,number[]>|null}[];clear_pair_count:number}[];
}
interface CellManifest {
  turntable?:IndexedTable;
  world_from_build:number[][]; placement_confirmed:boolean; limitations:string[];
  environment:{table_z_m:number;platform_size_m:number[];platform_top_z_m?:number;build_riser?:BuildRiser};
  initial_inter_arm_check:{status:string}|null;
  arms:{id:string;robot_manifest:string;robot_visual:string;world_from_base:number[][];
    flange_to_tcp:number[][];initial_positions:number[];tool_model:ToolModel;
    initial_static_contacts:unknown[]}[];
}
interface CellDrawingVariants {
  status:string; groups:[string,string][]; requested_groups?:number;
  variants:{arm:string;stroke_id:string;variant:string;start_anchor:string;reverse:boolean;
    roll_rad:number;pen_tilt_deg?:number;positions:number[][];tcp_targets_m:number[][];
    maximum_sampled_interpolation_error_m:number}[];
}
interface AllocationAction {
  mode_id:string; stroke_id:string; arm:string; variant:string; start_anchor:string;
  predecessor_mode:string|null; support_modes:string[]; start_s:number; end_s:number;
  drawing_s:number;
}
interface AllocationResult {
  solver:string; status:string; wall_time_s:number; makespan_s:number|null;
  objective_bound_s:number|null; actions:AllocationAction[]; enabled_arms:string[];
}
interface MaterialOrderProblem {
  modes:{id:string;arm:string;task:string}[];
  precedence_modes?:[string,string][];
}
type ReplayFailure=NonNullable<NonNullable<CoordinatedPlan['replay']>['failures']>[number];
interface PenOrientationProbe {
  tilt_deg:number;scope:string;original_witness:ReplayFailure;
  original_pair:{status:string};tilted_pair?:{status:string};
  flange_separation_before_m?:number;flange_separation_after_m?:number;
  arms:{arm:string;ik_found:boolean;positions?:number[];tcp_displacement_m?:number;static_contacts?:unknown[]}[];
}
interface RestartPreview { arm:string; stroke_id:string; restart_count:number; trajectory:CoordinatedAction["trajectory"]; prior_paths:number[][][]; }
interface CellAllocation {
  status:string; fixed_mode_count:number; requested_strokes:number;
  estimated_speedup:number|null; results:{dual:AllocationResult;single:AllocationResult};
  initial_positions?:Record<string,number[]>;
  support_first?:{members:string[];policy:string};
  process_assumptions?:{nominal_speed_m_s:number;cooling_s:number;drawing_clock?:string};
  motion_preparation?:{
    resource_mode?:'single'|'dual';
    status:string;prepared_strokes:number;requested_strokes:number;
    active_motion?:{mode_id:string;arm:string;stroke_id:string;started_at:string};
    failures:{mode_id?:string;scope?:string;evidence?:{phase:string;status:string;arm:string;positions?:number[];pairs?:{first:string;second:string}[]}[]}[];
    motions:{mode_id:string;arm:string;stroke_id:string;trajectory:CoordinatedAction['trajectory']}[];
    replay_status?:string;retimed_schedule?:{makespan_s:number};
    replay?:CoordinatedPlan['replay'];replay_in_progress?:boolean;
    replay_attempts?:{status:string;failures:{time_s?:number;positions?:number[][];pairs?:{first:string;second:string}[]}[]}[];
    refinements?:{kind:string;modes?:string[];pairs?:string[][];arm?:string;
      after_mode?:string;next_mode?:string|null;park_duration_s?:number;witness?:ReplayFailure}[];
  };
}
interface GeometricPartition {
  status:string; seam_width_m:number; join_nodes:string[];
  preserve_structural_units:boolean; cycle_blocks_preserved:number;
  structural_units:{id:string;members:string[];work_zone:string;length_m:number}[];
  zones:{id:string;stroke_count:number;length_m:number}[];
  curves:(Curve&{work_zone:string;allowed_arms:string[];allocation_group?:string})[];
  source_length_m:number;partitioned_length_m:number;
  construction_status:string;construction_actions:number;
}
interface Run { artifact_revision?:string; id: string; name: string; created_at: string; updated_at: string; stages: Record<string, Stage>; artifacts: string[]; cell_source_run?:string; workbench_primary?:boolean; workbench_collection?:string; drawing_pen_tilt_deg?:number; resource_mode?:'single'|'dual' }
interface CollisionRow { id:string; status:string; tracking_status?:string; position_error_bound_m?:number; orientation_error_bound_rad?:number; witnesses:{edge_index:number;alpha:number;positions:number[];pairs:{first:string;second:string}[]}[] }
interface CollisionAudit {tool_model?:ToolModel;source_motion_sha256:string; status:string; counts:Record<string,number>; config:{clearance_m:number}; strokes:CollisionRow[]}
interface DepartureAttempt {direction:number[];status:string;failed_sample?:number;collision_evidence?:{scope:string;status:string;pairs:{first:string;second:string}[]}[]}
interface SequenceOperation {id:string;kind:string;stroke_id:string|null;positions:number[][];tcp_path_m:number[][];extrusion_enabled:boolean;completed_strokes:number;tcp_length_m:number;departure_direction?:number[];departure_attempts?:DepartureAttempt[]}
interface SequencePreview {tool_model?:ToolModel;status:string;completed_strokes:number;requested_strokes:number;operations:SequenceOperation[];flange_to_tcp:number[][];material_segments:number;failures:{phase:string;status:string}[];settings:{contact_radius_m?:number};search?:{attempts:number;backtracks:number;current_depth?:number}}
interface Manifest { source_triangles: number; source_vertices: number; geometry_triangles: number; bounds_m: number[][]; max_extent_m: number; source: {name: string}; diagnostics: {watertight: boolean; component_count: number}; texture: {has_base_color: boolean} }
interface SelectionStep {selected_count:number;selection_score:number;partial_seconds:number;connected_length_m:number}
interface SparseSelection {drawing_resources?:number;drawing_loads_s?:number[];algorithm:string;source_curve_count:number;coverage_score:number;coverage_retention:number;raw_curve_seconds:number;support_reserve_s:number;phases:{objective:string;status:string;seconds:number}[];feature_channels?:{channel:string;selected_coverage:number;available_coverage:number}[];held_out_views:{azimuth_deg:number;elevation_deg:number;boundary_coverage:number;all_candidates_boundary_coverage:number;interior_coverage?:number|null;all_candidates_interior_coverage?:number|null;image:string}[]}
interface CurveSet { curves: Curve[]; total_length_m: number; label: string; method: string; selection?:{budget_seconds:number;overhead_reserve_s:number;status:string;trace:SelectionStep[];cost?:{partial_seconds:number;connected_length_m:number}} & Partial<SparseSelection> }
interface CandidateScore {
  id:string;label:string;source_file:string;curve_count:number;pen_stroke_count:number;
  raw_length_m:number;connected_length_m:number;added_length_m:number;material_components:number;
  boundary_coverage:number;worst_view_boundary_coverage:number;keypoint_coverage:number|null;
  shape_proxy_score:number;pareto:boolean;interior_coverage?:number|null;
  timing:{assumed_speed_m_s:number;partial_estimate_s:number;remaining_budget_s:number;within_partial_budget:boolean};
  views:{azimuth_deg:number;elevation_deg:number;raw_preview:string;connected_preview:string;
    raw:{boundary_coverage:number;outside_fraction:number};connected:{boundary_coverage:number;outside_fraction:number}}[];
}
interface CandidateComparison {status:string;provisional_choice:string|null;settings:{target_seconds:number;boundary_tolerance_m:number};candidates:CandidateScore[]}
interface StrokeGraph { edges: Curve[]; nodes: Junction[]; source_method: string; candidate_to_build:number[][]; metrics: { input_curves:number; material_members:number; proposed_pen_strokes:number; material_components:number; ground_contacts:number; total_length_m:number; added_length_m:number }; diagnostics: {code:string;message:string}[] }
interface Structure { modeled_pass:boolean; maximum_displacement_m?:number; maximum_stress_pa?:number; resting_com_inside_contacts?:boolean; failure?:string; edges:Deformation[] }
interface ProcessRow { id:string; modeled_pass:boolean; requires_calibration:boolean; directions:{reverse:boolean;failures:string[];advisories:string[]}[] }
interface ProcessScreen { acceptable_actions:number; rejected_actions:number; calibration_required_actions:number; edges:ProcessRow[]; settings:Record<string,number|null|boolean|string>; timing_assumption:{speed_m_s:number;deposition_only_s:number} }
interface BuildAction extends Curve { parent_curve?:string; stage?:string; junction_visits?:{node:string;segment_boundary:number}[]; start:string; end:string; source_stroke:string; depends_on:number[]; existing_join_nodes:string[]; members:string[] }
interface Construction { status:string; actions:BuildAction[]; total_length_m:number; states:number }

const stages = [
  { id: 'asset', name: 'Model preparation', title: 'Start with the shape.', description: 'A textured reference and a separate geometry copy, placed in a shared physical coordinate frame.' },
  { id: 'curves', name: 'Candidate curves', title: 'Find the essential lines.', description: 'Explore the 3D strokes before making decisions about connections, support, or printing order.' },
  { id: 'graph', name: 'Connected structure', title: 'Make every line belong.', description: 'Inspect physical junctions, permanent connectors, and the supporting structure.' },
  { id: 'construction', name: 'Construction order', title: 'Build it one stroke at a time.', description: 'Replay the construction sequence and inspect the support available at each step.' },
  { id: 'cell', name: 'Robot setup', title: 'Place the robot and pen.', description: 'Inspect robot bases, individual tool transforms and the shared build platform.' },
  { id: 'motion', name: 'Robot motion', title: 'Give the arm a path.', description: 'Inspect continuous robot configurations, pen orientation, and clearance around the growing sculpture.' },
  { id: 'timing', name: 'Pen timing', title: 'Put motion and material in sync.', description: 'Review printing speed, extrusion commands, cooling holds, and the full fabrication duration.' },
  { id: 'validation', name: 'Validation', title: 'Check the complete build.', description: 'Review the evidence for recognition, connectivity, physical support, and robot execution.' },
];
let run: Run;
let manifest: Manifest;
let activeStage = 'asset';
let modelMode = 'textured';
let displayedModel = 'textured';
let method = 'geometric';
let structureVariant = 'backbone';
let budget = 40;
let currentCurves: CurveSet | null = null;
let processRows = new Map<string,ProcessRow>();
let stageRequest = 0;
let stageLoads = 0;
let modelRequest = 0;
let runRequest = 0;
let playback: ReturnType<typeof setInterval> | null = null;
let cellReferenceVisible=false;
let selectedCellCandidate:string|null=null;
let selectedCellWaypoint=0;
let allRuns:Run[]=[];
let runScope:'current'|'reuse'|'all'='current';
let motionMode:'drawings'|'sequence' = 'drawings';
const $ = <T extends HTMLElement = HTMLElement>(selector: string) => document.querySelector<T>(selector)!;
const escape = (s: unknown) => String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]!));
const number = (n: number) => new Intl.NumberFormat('en-US').format(n);
async function json<T>(url: string): Promise<T> { const response = await fetch(url); if (!response.ok) throw new Error(`${response.status}: ${url}`); return response.json(); }
function playbackLoading(message:string) {
  const button=$<HTMLButtonElement>('#robot-play');
  button.classList.remove('hidden');button.disabled=true;button.textContent='Loading playback…';
  $('#playback-load').classList.remove('hidden');$('#playback-load').textContent=message;
}
async function artifactText(response:Response,progress:(received:number,total:number|null)=>void) {
  const length=Number(response.headers.get('content-length'));
  const total=Number.isFinite(length)&&length>0?length:null;
  if(!response.body)return response.text();
  const reader=response.body.getReader(),decoder=new TextDecoder();
  const chunks:string[]=[];let received=0,lastUpdate=0;
  progress(0,total);
  while(true) {
    const {value,done}=await reader.read();
    if(done)break;
    received+=value.byteLength;chunks.push(decoder.decode(value,{stream:true}));
    if(performance.now()-lastUpdate>100){progress(received,total);lastUpdate=performance.now();}
  }
  chunks.push(decoder.decode());progress(received,total);
  return chunks.join('');
}
async function checkedArtifact<T>(file:string,onDigest?:(digest:string)=>void):Promise<T> {
  const artifactRun=encodeURIComponent(run.id);
  const loadRequest=stageRequest;
  const tracked=['dual_arm_plan.json','dual_arm_repair_plan.json','offline_dual_arm_plan.json',
    'dual_arm_validation.json','dual_arm_repair_validation.json','offline_dual_arm_validation.json'].includes(file)
    && ['cell','motion','timing','validation'].includes(activeStage);
  const label=file.endsWith('_validation.json')?'validation report':'motion plan';
  const status=(message:string)=>{if(tracked&&loadRequest===stageRequest)playbackLoading(message);};
  // A running planner may atomically replace a checkpoint between these reads.
  for(let attempt=0;attempt<3;attempt++) {
    status(`Starting ${label} download…`);
    const response=await fetch(`/api/runs/${artifactRun}/artifacts/${file}`);
    if(!response.ok)throw new Error(`Could not load ${file}`);
    const raw=tracked?await artifactText(response,(received,total)=>{
      const downloaded=(received/1e6).toFixed(1);
      status(`Downloading ${label}: ${downloaded}${total?` / ${(total/1e6).toFixed(1)}`:''} MB${total?` (${Math.min(100,Math.floor(received/total*100))}%)`:''}`);
    }):await response.text();
    status(`Downloaded ${label}. Checking saved evidence…`);
    const validity=await json<{current:boolean;artifact_sha256:string;stale_inputs:string[]}>(`/api/runs/${artifactRun}/checks/${file}`);
    const digest=[...new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(raw)))].map(v=>v.toString(16).padStart(2,'0')).join('');
    if(digest!==validity.artifact_sha256)continue;
    if(!validity.current)throw new Error(`Artifact inputs changed: ${validity.stale_inputs.join(', ')}`);
    onDigest?.(digest);
    status(`Checked ${label}. Preparing the full playback…`);
    // Let the status paint before parsing a large local JSON payload.
    if(tracked)await new Promise<void>(resolve=>setTimeout(resolve,0));
    return JSON.parse(raw) as T;
  }
  throw new Error('The planner is updating this result. Refresh to load its next checkpoint.');
}
function url(file: string) { return `/api/runs/${encodeURIComponent(run.id)}/artifacts/${file}`; }
function statusText(status: string) { return ({complete:'Available',partial:'Partial result',pending:'Not generated',running:'Processing',awaiting_compute:'GPU job prepared',failed:'Needs attention'} as Record<string,string>)[status] ?? status; }
function runLabel(value:Run) { if(value.workbench_primary)return value.name; return value.name+(value.resource_mode==='single'?' · one active arm':'')+((value.drawing_pen_tilt_deg??0)>0?` · ${value.drawing_pen_tilt_deg}° ${'initial pen lean'}`:''); }
function curveStageKey() { return method === 'selected' ? 'selection' : method === 'wir3d' ? 'wir3d' : 'geometric'; }
function defaultCurveMethod(value:Run) {
  if(value.artifacts.includes('selected_candidates.json') && (value.stages.selection?.curve_count ?? 0)>0) return 'selected';
  if(value.artifacts.some(file=>/^wir3d_\d+\.json$/.test(file))) return 'wir3d';
  return 'geometric';
}

$('#app').innerHTML = `
<header class="topbar"><div class="brand"><svg viewBox="0 0 30 35" fill="none" aria-hidden="true"><path d="M4 28V12l11-8 11 8v16l-11 5L4 28Z M4 12l11 7 11-7 M15 19v14 M4 28l11-9 11 9" stroke="#526e59" stroke-width="1.3"/></svg>pen<span>/</span>art</div><div class="top-right"><span class="local"><i class="dot"></i>Local workbench</span><select id="run-scope" aria-label="Run collection"></select><select id="run-select" aria-label="Model run"></select></div></header>
<div id="error" class="error-banner hidden" role="alert"></div>
<div class="layout"><aside class="sidebar"><div class="eyebrow">The pipeline</div><nav id="stage-nav" style="display:contents"></nav><div class="sidebar-footer"><strong>Spatial strokes. Shared workspace.</strong>Inspect every transformation<br>from mesh to sculpture.</div></aside>
<main class="main"><div class="heading"><div><div class="eyebrow" id="breadcrumb">Model / 01</div><h1 id="title"></h1><p id="subtitle"></p></div><span id="badge" class="badge">Available</span></div>
<div class="work-area"><section><div class="viewer-card"><div class="toolbar"><div class="segmented" id="display-tabs"><button data-mode="textured" class="selected">Textured reference</button><button data-mode="geometry">Geometry copy</button></div><span class="eyebrow" id="frame-label">Build frame · meters</span></div>
<div class="canvas-wrap" id="viewport"><div class="loader hidden" id="loader">Loading model…</div><div class="view-buttons" aria-label="Camera presets"><button data-view="perspective">3D</button><button data-view="front">Front</button><button data-view="side">Side</button><button data-view="top">Top</button></div><div class="viewer-hint">Drag to orbit · scroll to zoom · right-drag to pan</div><div class="empty-state hidden" id="empty"></div></div>
<div class="viewer-footer"><div class="legend" id="legend"><span><i class="reference"></i>Source geometry</span></div><span id="scale-label">150 mm maximum extent</span></div></div>
<div class="note" id="note"><span class="symbol">↳</span><span id="note-text"></span></div>
<div class="stage-summary"><div class="summary-item"><div class="eyebrow">Input</div><p id="input-summary"></p></div><div class="summary-item"><div class="eyebrow">Output</div><p id="output-summary"></p></div><div class="summary-item"><div class="eyebrow">Next step</div><p id="next-summary"></p></div></div></section>
<aside class="inspector"><section class="panel" id="metrics"></section><section class="panel"><h2>Viewport</h2><div class="control"><label>Ground grid <input type="checkbox" id="grid" checked></label></div><div class="control"><label>Slow orbit <input type="checkbox" id="orbit"></label></div><div class="control hidden" id="overlay-control"><label>Reference overlay <input type="checkbox" id="overlay"></label></div><div class="control hidden" id="count-control"><label>Visible curves <span id="curve-count"></span></label><input id="curve-slider" aria-label="Visible curves" type="range" min="0"></div></section><section class="panel hidden" id="selection-panel"><h2>Selected stroke</h2><div class="selection" id="selection">Click a stroke to inspect it.</div></section></aside></div></main></div>`;

const viewer = new Viewer($('#viewport'), curve => {
  const row = curve ? processRows.get(curve.id) : undefined;
  const failures = row ? [...new Set(row.directions.flatMap(d => d.failures))] : [];
  $('#selection').innerHTML = curve ? `<strong>${escape(curve.id)}</strong><br>${(curve.length_m * 1000).toFixed(1)} mm of stroke<br>${curve.segments.length} Bézier pieces<br>${escape(curve.role ?? 'Candidate curve')}<br>${curve.closed ? 'Closed contour' : 'Open curve'}${row ? `<br><br>${row.modeled_pass ? 'No geometric screen failure; pen capability and construction order remain unverified.' : failures.map(f=>escape(f.toLowerCase().replaceAll('_',' '))).join('<br>')}` : ''}` : 'Click a stroke to inspect it.';
});
$('#frame-label').insertAdjacentHTML('afterend','<button id="robot-play" class="mode-button hidden">Play robot motion</button>');
$('.toolbar').insertAdjacentHTML('afterend','<div id="playback-load" class="playback-load hidden" role="status" aria-live="polite"></div>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="reference-panel" class="reference-panel hidden"></section>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="criteria-panel" class="criteria-panel hidden"></section>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="comparison-panel" class="comparison-panel hidden"></section>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="selection-evidence" class="comparison-panel hidden"></section>');
$('#viewport').insertAdjacentHTML('beforeend', '<figure id="source-thumbnail" class="source-thumbnail hidden"><img alt="Original textured model reference"><figcaption>Original model</figcaption></figure>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="timing-panel" class="comparison-panel hidden"></section>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="audit-panel" class="comparison-panel hidden"></section>');
$('.stage-summary').insertAdjacentHTML('afterend', '<section id="motion-reuse-panel" class="comparison-panel hidden"></section>');
$('.inspector').insertAdjacentHTML('beforeend', '<section id="graph-controls" class="panel hidden"></section>');
$('#viewport').insertAdjacentHTML('beforeend', '<div id="paired-labels" class="paired-labels hidden"><span>Original textured model</span><span>Candidate strokes</span></div>');
$('#overlay-control').insertAdjacentHTML('beforebegin', '<div class="control hidden" id="paired-control"><label>Compare with original <input type="checkbox" id="paired" checked></label></div>');
function updatePairedView() {
  const enabled = activeStage === 'curves' && $<HTMLInputElement>('#paired').checked;
  viewer.compareSource = enabled;
  $('#paired-labels').classList.toggle('hidden', !enabled);
  $('#paired-control').classList.toggle('hidden', activeStage !== 'curves');
  $('#overlay-control').classList.toggle('hidden', enabled || !['curves','graph','construction'].includes(activeStage));
  $('#source-thumbnail').classList.toggle('hidden', enabled || !['curves','graph','construction'].includes(activeStage) || !run.artifacts.some(f=>f.startsWith('wir3d_reference_')));
  if(activeStage === 'curves')viewer.setSource($<HTMLInputElement>('#overlay').checked, true);
}
$('#paired').addEventListener('change', updatePairedView);
$('#grid').addEventListener('change', () => { viewer.grid.visible = $<HTMLInputElement>('#grid').checked; });
$('#orbit').addEventListener('change', () => { viewer.controls.autoRotate = $<HTMLInputElement>('#orbit').checked; });
$('#overlay').addEventListener('change', () => { viewer.setSource($<HTMLInputElement>('#overlay').checked, true); });
$('#curve-slider').addEventListener('input', () => { const count = Number($<HTMLInputElement>('#curve-slider').value); viewer.setLimit(count); $('#curve-count').textContent = `${count} / ${currentCurves?.curves.length ?? 0}`; });
document.querySelectorAll<HTMLButtonElement>('[data-view]').forEach(button => button.addEventListener('click', () => viewer.view(button.dataset.view!)));

function showError(error: unknown) { $('#error').textContent = String(error); $('#error').classList.remove('hidden'); }
function metric(label: string, value: string) { return `<div class="metric"><span>${label}</span><strong>${value}</strong></div>`; }
function toolSummary(tool?:ToolModel) {
  return `<h2>Pen & mount geometry</h2>${metric('Geometry',tool?.configured ? `${tool.visual_bodies.length} configured bodies` : 'Not supplied')}${tool?.configured ? `<p class="description">Blue-gray solids show the supplied non-contact geometry used by the collision checker. ${tool.calibration_declared ? 'The profile declares calibration; physical validation is recorded separately.' : 'The profile does not declare calibration.'}</p>` : '<p class="description">The blue axis is the assumed flange-to-tip transform, not a pen-body model. Measured pen and mount geometry are still required.</p>'}`;
}
function renderComparison(report:CandidateComparison, file:string|null) {
  const panel=$('#comparison-panel');
  panel.classList.remove('hidden');
  const expected=run.artifacts.filter(name=>['geometric_candidates.json','surface_contours.json'].includes(name)||/^wir3d_\d+\.json$/.test(name));
  if(report.status!=='COMPLETE_GEOMETRIC_COMPARISON'||expected.some(name=>!report.candidates.some(c=>c.source_file===name))) {
    panel.textContent='Candidate comparison is updating. Ranked results will appear when all available candidates have been evaluated.';
    return;
  }
  const selected=report.candidates.find(c=>c.source_file===file) ?? report.candidates[0];
  if(!selected) {panel.textContent='Candidate comparison is running.';return;}
  const pct=(value:number)=>(100*value).toFixed(0)+'%';
  const hasInterior=report.candidates.some(c=>c.interior_coverage!=null);
  panel.innerHTML=`<h2>Shape and drawing cost</h2><p>Fixed-view outline coverage and automatic feature proximity are geometric proxies for shape. They do not establish human recognition. The drawing estimate includes proposed connections and feed starts/stops at an assumed ${(selected.timing.assumed_speed_m_s*1000).toFixed(1)} mm/s; travel, joins, cooling and acceleration add time.</p><div class="comparison-table-wrap"><table><thead><tr><th>Candidate</th><th>Outline coverage</th><th>Keypoint coverage</th>${hasInterior?'<th>Interior edges</th>':''}<th>Connected length</th><th>Aggregate drawing work</th></tr></thead><tbody>${report.candidates.map(c=>`<tr class="${c===selected?'selected':''}"><td><button data-compare-candidate="${escape(c.source_file)}">${escape(c.label)}</button><small>${c.curve_count} curves → ${c.pen_stroke_count} pen strokes${c.pareto?' · quality/length frontier':''}</small></td><td>${pct(c.boundary_coverage)}<small>Worst view ${pct(c.worst_view_boundary_coverage)}</small></td><td>${c.keypoint_coverage===null?'Unavailable':pct(c.keypoint_coverage)}</td>${hasInterior?`<td>${c.interior_coverage==null?'Unavailable':pct(c.interior_coverage)}</td>`:''}<td>${c.connected_length_m.toFixed(2)} m<small>${c.added_length_m.toFixed(2)} m connectors/supports</small></td><td>${(c.timing.partial_estimate_s/60).toFixed(1)} min<small>Across all pens; not wall time</small></td></tr>`).join('')}</tbody></table></div><p>Two-arm fabrication time requires a dependency- and collision-aware schedule. These aggregate costs do not determine whether a candidate meets the 30-minute target.${report.candidates.length===1 ? ' Optimized WIR3D candidates will be added as they finish.' : ''}</p><div class="comparison-controls"><label>View evidence<select id="comparison-geometry" aria-label="Comparison geometry"><option value="raw">Original candidate strokes</option><option value="connected">With proposed connections</option></select></label><span>Gray: reference silhouette${hasInterior?' · blue: interior edge evidence':''} · green: projected strokes · ${(report.settings.boundary_tolerance_m*1000).toFixed(1)} mm outline tolerance</span></div><div id="comparison-views" class="comparison-views"></div><a class="download" href="${url('candidate_comparison.json')}" target="_blank">Open comparison metrics ↗</a>`;
  const show=(kind:'raw'|'connected')=>{
    $('#comparison-views').innerHTML=selected.views.map(v=>`<figure><img src="${url(kind==='raw'?v.raw_preview:v.connected_preview)}" alt="${escape(selected.label)} at azimuth ${v.azimuth_deg} and elevation ${v.elevation_deg} degrees with ${kind==='raw'?'original strokes':'proposed connections'}"><figcaption>${v.azimuth_deg}° / ${v.elevation_deg}° · outline ${pct(v[kind].boundary_coverage)} · outside ${pct(v[kind].outside_fraction)}</figcaption></figure>`).join('');
  };
  $('#comparison-geometry').addEventListener('change',()=>show($<HTMLSelectElement>('#comparison-geometry').value as 'raw'|'connected'));
  panel.querySelectorAll<HTMLButtonElement>('[data-compare-candidate]').forEach(button=>button.addEventListener('click',()=>{
    const filename=button.dataset.compareCandidate!;
    method=filename==='geometric_candidates.json'?'geometric':filename==='surface_contours.json'?'contours':filename==='selected_candidates.json'?'selected':'wir3d';
    if(method==='wir3d')budget=Number(filename.match(/^wir3d_(\d+)\.json$/)![1]);
    selectStage('curves').catch(showError);
  }));
  show('raw');
}
function renderSelection(candidate:CurveSet) {
  const result=candidate.selection!;
  if(result.algorithm==='fixed_curve_max_coverage_then_min_cost') {
    const sparse=result as typeof result & SparseSelection;
    const resources=sparse.drawing_resources ?? 1;
    $('#metrics').insertAdjacentHTML('beforeend', `<h2>Offline curve selection</h2>${metric('Source candidates',String(sparse.source_curve_count))}${metric('Kept whole curves',String(candidate.curves.length))}${metric('Selection-view coverage',`${(100*sparse.coverage_score).toFixed(1)}%`)}${metric(resources>1?'Total drawing work':'Raw drawing + feed',`${(sparse.raw_curve_seconds/60).toFixed(1)} min`)}${metric(resources>1?'Per-pen raw allowance':'Raw curve allowance',`${(sparse.budget_seconds/60).toFixed(1)} min`)}${resources>1?metric('Pens budgeted',String(resources))+metric('Provisional drawing loads',(sparse.drawing_loads_s??[]).map(v=>(v/60).toFixed(1)+' min').join(' / ')):''}${metric('Support allowance',`${(sparse.support_reserve_s/60).toFixed(1)} min`)}${metric('Motion/other allowance',`${(sparse.overhead_reserve_s/60).toFixed(1)} min`)}<p class="description">Maximize geometric coverage, then reduce drawing cost while retaining ${(100*sparse.coverage_retention).toFixed(0)}% of the achieved score. Both solves use the fixed candidate set. ${resources>1?'Whole curves fit separate drawing budgets; this is not the final arm assignment or schedule. ':''}Added supports and the complete schedule are checked afterward.</p>${sparse.phases.map(p=>metric(p.objective==='maximize_coverage'?'Coverage solve':'Sparsity solve',`${p.status.toLowerCase()} · ${p.seconds.toFixed(2)} s`)).join('')}`);
    const detailed=!!sparse.feature_channels?.length;
    const pct=(value:number|null|undefined)=>value==null?'—':`${(100*value).toFixed(0)}%`;
    const channels=['boundary','interior','keypoints'].flatMap(channel=>{
      const rows=sparse.feature_channels?.filter(row=>row.channel===channel)??[];
      return rows.length ? [{channel,selected:rows.reduce((sum,row)=>sum+row.selected_coverage,0)/rows.length,available:rows.reduce((sum,row)=>sum+row.available_coverage,0)/rows.length}] : [];
    });
    $('#selection-evidence').classList.remove('hidden');
    $('#selection-evidence').innerHTML=`<h2>${detailed?'Preserve the shape inside the outline':'Views excluded from selection'}</h2><p>${detailed?'Gray: silhouette. Blue: visible interior edges from the mesh. Green: selected strokes. Selection balances evidence across image regions and retains achieved coverage in each view and feature channel.':'Gray shows the reference silhouette; green shows selected curves.'} These measurements do not establish human recognition. The eight views below are excluded from optimization.</p>${detailed?`<table><thead><tr><th>Selection-view evidence</th><th>Selected curves</th><th>All source candidates</th></tr></thead><tbody>${channels.map(c=>`<tr><td>${c.channel==='boundary'?'Outline':c.channel==='interior'?'Interior edges':'Automatic keypoints'}</td><td>${pct(c.selected)}</td><td>${pct(c.available)}</td></tr>`).join('')}</tbody></table>`:''}<div class="comparison-views">${sparse.held_out_views.map(v=>`<figure><img src="${url(v.image)}" alt="Selected curves and ${detailed?'visible edge':'silhouette'} evidence at ${v.azimuth_deg} degrees azimuth and ${v.elevation_deg} degrees elevation"><figcaption>${v.azimuth_deg}° / ${v.elevation_deg}° · outline ${pct(v.boundary_coverage)} / ${pct(v.all_candidates_boundary_coverage)}${detailed?` · interior ${pct(v.interior_coverage)} / ${pct(v.all_candidates_interior_coverage)}`:''}<br>Selected / all candidates</figcaption></figure>`).join('')}</div>`;
    return;
  }
  const trace=result.trace;
  $('#metrics').insertAdjacentHTML('beforeend',`<h2>Automatic selection</h2>${metric('Drawing allowance',(result.budget_seconds/60).toFixed(1)+' min')}${metric('Reserved overhead',(result.overhead_reserve_s/60).toFixed(1)+' min')}<p class="description">Curves retain their source geometry. Each addition is costed with automatic connections. The overhead allowance is provisional; travel, joins and cooling require a complete timed plan.</p>${trace.length?`<label class="control">Selection progress<select id="selection-step" class="method-select" aria-label="Selection progress">${trace.map((step,i)=>`<option value="${i}">${step.selected_count} curves · ${(step.partial_seconds/60).toFixed(1)} min</option>`).join('')}</select></label><div id="selection-step-metrics"></div>`:''}`);
  if(!trace.length)return;
  const show=(index:number)=>{
    const step=trace[index];
    viewer.setLimit(step.selected_count);
    $<HTMLInputElement>('#curve-slider').value=String(step.selected_count);
    $('#curve-count').textContent=`${step.selected_count} / ${candidate.curves.length}`;
    $('#selection-step-metrics').innerHTML=metric('Connected drawing length',step.connected_length_m.toFixed(2)+' m')+metric('Selection-view proxy',(step.selection_score*100).toFixed(0)+'%');
  };
  $<HTMLSelectElement>('#selection-step').value=String(trace.length-1);
  $('#selection-step').addEventListener('change',()=>show(Number($<HTMLSelectElement>('#selection-step').value)));
  $<HTMLInputElement>('#curve-slider').oninput=()=>{
    const count=Number($<HTMLInputElement>('#curve-slider').value);
    const index=trace.findIndex(step=>step.selected_count===count);
    if(index>=0){$<HTMLSelectElement>('#selection-step').value=String(index);show(index);}
  };
  show(trace.length-1);
}
function empty(title: string, message: string) {
  $('#empty').innerHTML = `<div class="empty-inner"><svg viewBox="0 0 50 50" fill="none" aria-hidden="true"><path d="M8 37V17L25 7l17 10v20L25 46 8 37Z M8 17l17 11 17-11 M25 28v18" stroke="currentColor" stroke-width="1.1" stroke-dasharray="3 3"/></svg><h3>${escape(title)}</h3><p>${escape(message)}</p></div>`;
  $('#empty').classList.remove('hidden');
}

async function loadModel(mode: string) {
  const request = ++modelRequest;
  displayedModel = mode;
  $('#loader').classList.remove('hidden');
  const modelUrl = url(mode === 'robot' ? 'robot_reference.glb' : mode === 'geometry' ? 'geometry.glb' : 'normalized.glb');
  await viewer.load(modelUrl, progress => { if (request === modelRequest) $('#loader').textContent = `Loading ${mode === 'geometry' ? 'geometry' : 'textured reference'}… ${Math.round(progress * 100)}%`; });
  if (request === modelRequest) {
    $('#loader').classList.add('hidden');
    viewer.setSource((mode === 'robot' && ['motion','timing','validation'].includes(activeStage)) || activeStage === 'asset' || $<HTMLInputElement>('#overlay').checked, mode !== 'robot' && activeStage !== 'asset');
  }
}

async function selectStage(id: string) {
  stageLoads++;
  const selectedRun=run.id;
  try { await renderStage(id); }
  catch(error) {
    if(run.id===selectedRun&&activeStage===id&&['cell','motion','timing','validation'].includes(id)) {
      const button=$<HTMLButtonElement>('#robot-play');
      button.classList.remove('hidden');button.disabled=true;button.textContent='Playback unavailable';
      $('#playback-load').classList.remove('hidden');$('#playback-load').textContent=String(error);
    }
    throw error;
  }
  finally { stageLoads--; }
}

async function renderStage(id: string) {
  const previousCellMode=activeStage===id?document.querySelector<HTMLSelectElement>('#cell-mode')?.value:undefined;
  viewer.camera.zoom=1;
  if (playback) { clearInterval(playback); playback = null; }
  $('#robot-play').classList.add('hidden');
  $<HTMLButtonElement>('#robot-play').disabled=false;
  $('#playback-load').classList.add('hidden');
  $<HTMLButtonElement>('#robot-play').onclick=null;
  const request = ++stageRequest;
  activeStage = id;
  const selectedUrl=new URL(location.href);
  selectedUrl.searchParams.set('run',run.id);
  selectedUrl.searchParams.set('stage',id);
  history.replaceState(null,'',selectedUrl);
  $('#comparison-panel').classList.add('hidden');
  $('#selection-evidence').classList.add('hidden');
  $('#timing-panel').classList.add('hidden');
  $('#audit-panel').classList.add('hidden');
  viewer.clearCell();
  viewer.clearSequence();
  $('.main').dataset.stage = id;
  const index = stages.findIndex(stage => stage.id === id);
  const stage = stages[index];
  const status = run.stages[id === 'curves' ? curveStageKey() : id]?.status ?? 'pending';
  $('.stage-button[data-stage="curves"] .stage-status').textContent = statusText(run.stages[curveStageKey()]?.status ?? 'pending');
  document.querySelectorAll<HTMLButtonElement>('.stage-button').forEach(button => { button.classList.toggle('active', button.dataset.stage === id); button.setAttribute('aria-current', button.dataset.stage === id ? 'step' : 'false'); });
  $('#breadcrumb').textContent = `${runLabel(run)} / ${String(index + 1).padStart(2, '0')}`;
  $('#title').textContent = stage.title;
  $('#subtitle').textContent = stage.description;
  $('#badge').textContent = statusText(status);
  $('#badge').classList.toggle('pending', status !== 'complete');
  $('#empty').classList.add('hidden');
  const previews = run.artifacts.filter(file => /^wir3d_(reference|sketch)_\d+\.png$/.test(file));
  const referenceImage=previews.find(file=>file.startsWith('wir3d_reference_'));
  $('#source-thumbnail').classList.toggle('hidden',!referenceImage||!['curves','graph','construction'].includes(id));
  if(referenceImage)$<HTMLImageElement>('#source-thumbnail img').src=url(referenceImage);
  const sdfPreview = run.artifacts.filter(file => /^wir3d_sdf_step_\d{5}\.png$/.test(file)).sort().at(-1);
  const optimizationPreviews = new Map<string, {file:string; budget:string; phase:string; iteration:string}>();
  for(const file of [...run.artifacts].sort()) {
    const match=file.match(/^wir3d_optimization_(\d+)_(geo|sem)_(\d{5})\.png$/);
    if(match)optimizationPreviews.set(`${match[1]}-${match[2]}`,{file,budget:match[1],phase:match[2],iteration:match[3]});
  }
  $('#reference-panel').classList.toggle('hidden', id !== 'curves' || (!previews.length && !sdfPreview && !optimizationPreviews.size));
  $('#reference-panel').innerHTML = ['reference', 'sketch'].map(kind => {
    const files = previews.filter(file => file.startsWith(`wir3d_${kind}_`));
    return files.length ? `<h2>${kind === 'reference' ? 'WIR3D textured reference views' : 'WIR3D line reference views'}</h2><div class="reference-gallery">${files.map((file,i) => `<a href="${url(file)}" target="_blank"><img src="${url(file)}" alt="${kind === 'reference' ? 'Textured model' : 'Line drawing'} view ${i+1}" loading="lazy"/></a>`).join('')}</div>` : '';
  }).join('');
  if(sdfPreview) {
    const iteration=sdfPreview.match(/step_(\d+)/)![1];
    $('#reference-panel').insertAdjacentHTML('beforeend',`<h2>Surface-distance model · iteration ${Number(iteration)}</h2><p class="description">WIR3D compares its learned distance field with sampled surface distances. These are preprocessing diagnostics in WIR3D coordinates, not generated pen strokes.</p><a href="${url(sdfPreview)}" target="_blank"><img class="sdf-preview" src="${url(sdfPreview)}" alt="Learned and reference surface-distance fields at training iteration ${Number(iteration)}" /></a>`);
  }
  for(const preview of optimizationPreviews.values()) {
    const label=`${preview.budget}-curve budget · ${preview.phase==='geo'?'geometry':'semantic'} optimization · iteration ${Number(preview.iteration)}`;
    $('#reference-panel').insertAdjacentHTML('beforeend',`<h2>WIR3D progress: ${label}</h2><p class="description">Intermediate optimizer comparison image. These are unfinished visual diagnostics; final 3D curves appear as a selectable candidate only after verified export.</p><a href="${url(preview.file)}" target="_blank"><img class="sdf-preview" src="${url(preview.file)}" alt="Intermediate WIR3D ${label}" /></a>`);
  }
  $('#display-tabs').classList.toggle('hidden', id !== 'asset');
  ['overlay-control','count-control','selection-panel'].forEach(name => $(`#${name}`).classList.toggle('hidden', !['curves','graph','construction'].includes(id)));
  if (id === 'construction') $('#count-control').classList.add('hidden');
  $('#graph-controls').classList.add('hidden');
  $('#criteria-panel').classList.add('hidden');
  $('#motion-reuse-panel').classList.add('hidden');
  processRows.clear();
  viewer.setCurves([]);
  viewer.setPlatform();
  viewer.model.position.set(0,0,0);
  viewer.setSource(id === 'asset');
  updatePairedView();
  currentCurves = null;
  $<HTMLInputElement>('#curve-slider').oninput = null;
  $('#frame-label').textContent = 'Build frame · meters';
  $('#scale-label').textContent = `Object: ${manifest.bounds_m[1].map((v,i)=>((v-manifest.bounds_m[0][i])*100).toFixed(1)).join(' × ')} cm`;
  if ((id === 'curves' && viewer.compareSource && displayedModel !== 'textured') || (['asset', 'curves', 'graph', 'construction'].includes(id) && ['robot','cell'].includes(displayedModel))) {
    await loadModel(id === 'curves' && viewer.compareSource ? 'textured' : modelMode);
    if (request !== stageRequest) return;
  }
  if(['cell','motion','timing','validation'].includes(id) && run.artifacts.includes('cell_manifest.json')) {
    playbackLoading('Loading robot setup and saved playback files…');
    ++modelRequest;$('#loader').classList.add('hidden');
    const cell=await checkedArtifact<CellManifest>('cell_manifest.json');
    const cellConfig=await checkedArtifact<CellConfig>('cell_config.json');
    const penTilt=cellConfig.drawing_pen_tilt_deg??0;
    const penAngleLabel=penTilt===0?'Vertical':cellConfig.drawing_orientation_policy==='outward_radial'
      ? `${penTilt.toFixed(0)}° outward body lean · nozzle inward/down`
      : `${penTilt.toFixed(0)}° toward build center`;
    const arms:CellArmView[]=await Promise.all(cell.arms.map(async arm=>({
      ...arm,rig:await json<RobotRig>(url(arm.robot_manifest)),url:url(arm.robot_visual),
    })));
    if(request!==stageRequest)return;
    await viewer.loadCell(arms,{url:url('geometry.glb'),world_from_build:cell.world_from_build});
    if(request!==stageRequest)return;
    displayedModel='cell';
    const origin=cell.world_from_build.map(row=>row[3]);
    viewer.setPlatform(origin.slice(0,2),cell.environment.platform_top_z_m??origin[2],cell.environment.table_z_m,...cell.environment.platform_size_m as [number,number],cell.environment.build_riser);
    if(cell.turntable)viewer.setTurntable(cell.turntable,cell.environment.build_riser);
    $('#frame-label').textContent='Shared world frame · meters';
    $('#badge').textContent=`${arms.length===1?'One-arm':'Two-arm'} setup · provisional`;
    $('#metrics').innerHTML=`<h2>${arms.length===1?'One robot arm':'Two independent arms'}</h2>${metric('Arm count',String(arms.length))}${metric('Placement',cell.placement_confirmed?'Measured':'Development assumption')}${metric('Initial inter-arm clearance',cell.initial_inter_arm_check?.status==='CERTIFIED_INTER_ARM_INTERVAL'?'Clear at initial poses':escape(cell.initial_inter_arm_check?.status.replaceAll('_',' ')??'Not applicable'))}<p id="cell-setup-scope" class="description">This setup snapshot checks the imported arm meshes. Staged start poses and timed trajectories are checked separately.</p>${cell.arms.map((arm,i)=>`<h2>${escape(arm.id)}</h2>${metric('Tip marker',i===0?'Blue':'Bronze')}${metric('Base XYZ',arm.world_from_base.slice(0,3).map(row=>row[3].toFixed(2)).join(', ')+' m')}${metric('Pen/mount bodies',arm.tool_model.configured?'Configured':'Not supplied')}${metric('Initial static contacts',String(arm.initial_static_contacts.length))}`).join('')}<a class="download" href="${url('cell_manifest.json')}" target="_blank">Open shared cell manifest ↗</a>`;
    $('#note-text').textContent=`${arms.length===1?'The 7-DoF arm uses':'Both 7-DoF arms use'} explicit base and pen-tip transforms. The placement is provisional; the model remains uniformly scaled to 15 cm maximum extent.`;
    $('#input-summary').textContent=arms.length===1?'One robot description, the pen-tip transform and a shared build frame.':'Two robot descriptions, individual tool transforms and one shared build frame.';
    $('#output-summary').textContent=`${arms.length===1?'One-arm':'Two-arm'} kinematic and initial-clearance preview. No task assignment yet.`;
    $('#next-summary').textContent=arms.length===1?'Plan the complete stroke order, pen orientation and collision-free motions.':'Search per-arm stroke alternatives, assignments and collision-free overlapping trajectories.';
    $('#legend').innerHTML=arms.map((arm,i)=>`<span><i style="background:${i===0?'#327aa0':'#c0813f'}"></i>Arm ${i+1} tip</span>`).join('')+`<span>Reference shape · ${(manifest.max_extent_m*100).toFixed(1)} cm</span>`;
    const setupMetrics=$('#metrics').innerHTML;
    const unavailable:string[]=[];
    async function optionalResult<T>(file:string,label:string,onDigest?:(digest:string)=>void):Promise<T|null> {
      if(!run.artifacts.includes(file))return null;
      try{return await checkedArtifact<T>(file,onDigest);}
      catch(error){console.warn(`${label} unavailable`,error);unavailable.push(`${label} is unavailable or needs regeneration; current geometry remains visible.`);return null;}
    }
    const checkpoint=run.artifacts.includes('cell_pipeline_result.json')?await json<{status:string;updated_at:string;elapsed_s:number;detail?:string;no_time_limit?:boolean}>(url('cell_pipeline_result.json')):null;
    const validationSummary=await optionalResult<{generated_at:string;physical_status:string;checks:{name:string;status:string;detail:string;evidence:string|null}[]}>('validation_report.json','Validation summary');
    const candidates=run.artifacts.includes('cell_drawing_variants.json')?await checkedArtifact<CellDrawingVariants>('cell_drawing_variants.json'):null;
    const allocation=await optionalResult<CellAllocation>('cell_allocation.json','Saved allocation');
    const allocationProblem=allocation?await optionalResult<MaterialOrderProblem>('allocation_problem.json','Material ordering constraints'):null;
    if(allocation){
      const config=await checkedArtifact<{initial_positions:Record<string,number[]>;support_first?:CellAllocation['support_first'];process:{nominal_speed_m_s:number};cooling_s:number;timing:{drawing_clock?:string}}>('allocation_config.json');
      allocation.initial_positions=config.initial_positions;
      allocation.support_first=config.support_first;
      allocation.process_assumptions={nominal_speed_m_s:config.process.nominal_speed_m_s,cooling_s:config.cooling_s,drawing_clock:config.timing.drawing_clock};
      const preparation=await optionalResult<NonNullable<CellAllocation['motion_preparation']>>('allocation_motions.json','Prepared motions');
      if(preparation)allocation.motion_preparation=preparation;
    }
    const restart=await optionalResult<RestartPreview>('stroke_restart_preview.json','One-arm restart trial');
    const construction=allocation?await checkedArtifact<Construction>('construction_plan.json'):null;
    const partition=run.artifacts.includes('geometric_partition.json')?await checkedArtifact<GeometricPartition>('geometric_partition.json'):null;
    const orientationProbe=await optionalResult<PenOrientationProbe>('pen_orientation_probe.json','Pen-angle diagnostic');
    const mountAssessment=await optionalResult<MountAssessment>('cell_mounting_assessment.json','Base mounting assessment');
    const timingEligibility=await optionalResult<{status:string;requested_modes:number;active_mode?:string;modes:{mode_id:string;status:string;reason?:string}[]}>('allocation_timing.json','Fixed-path timing');
    const availablePlans:CoordinatedPlan[]=[];
    for(const file of ['dual_arm_plan.json','dual_arm_repair_plan.json','offline_dual_arm_plan.json']){
      if(!run.artifacts.includes(file))continue;
      let digest='';
      const candidate=await optionalResult<CoordinatedPlan>(file,'Saved motion replay',value=>{digest=value;});
      if(!candidate)continue;
      candidate.file_name=file;
      if(run.artifacts.includes('trajectory_manifest.json')){
        const exported=await optionalResult<{source_plan:string;source_plan_sha256:string}>('trajectory_manifest.json','Trajectory export');
        candidate.trajectory_export_available=exported?.source_plan===file&&exported.source_plan_sha256===digest;
      }
      const replayFile=file.replace('_plan.json','_validation.json');
      candidate.replay_in_progress=run.stages.validation?.status==='running'&&run.stages.validation.artifact===replayFile;
      if(run.artifacts.includes(replayFile)){
        try{
          const replay=await checkedArtifact<NonNullable<CoordinatedPlan['replay']>>(replayFile);
          if(replay.source_plan_sha256===digest)candidate.replay=replay;
        }catch(error){console.warn('Replay audit unavailable or stale',error);}
      }
      if(file==='offline_dual_arm_plan.json'){
        candidate.completed_strokes=candidate.replay?.completed_strokes??0;
        candidate.checked_intervals=candidate.replay?.checked_intervals??0;
      }
      availablePlans.push(candidate);
    }
    availablePlans.sort((a,b)=>Number(b.file_name==='offline_dual_arm_plan.json')-Number(a.file_name==='offline_dual_arm_plan.json') || b.completed_strokes-a.completed_strokes || a.modeled_duration_s-b.modeled_duration_s);
    const plan=availablePlans[0]??null;
    const motionReuse=await optionalResult<MotionReuse>('viewer_motion_reuse.json','Motion cache evidence');
    if(allocation?.motion_preparation&&plan?.file_name==='offline_dual_arm_plan.json'){
      allocation.motion_preparation.replay=plan.replay;
      allocation.motion_preparation.replay_in_progress=plan.replay_in_progress;
    }
    if(plan&&run.artifacts.includes('cell_comparison.json')){
      try{plan.comparison=await checkedArtifact<NonNullable<CoordinatedPlan['comparison']>>('cell_comparison.json');}
      catch(error){console.warn('Comparison unavailable or stale',error);}
    }
    if(request!==stageRequest)return;
    const options=[...(restart?[['restarts','One-arm restart trial']]:[]),...(mountAssessment?[['mounting','Base mounting assessment']]:[]),...(partition?[['partition','Structural work regions']]:[]),...(allocation?[['allocation','Offline allocation']]:[]),...(allocationProblem?.precedence_modes?.length&&construction?[['material-order','Material ordering constraints']]:[]),...(orientationProbe?[['orientation','Pen-angle diagnostic']]:[]),...(construction?[['scale','Size and speed study']]:[]),...(allocation?.motion_preparation?.motions.length?[['prepared','Selected full motions']]:[]),...(candidates?[['candidates','Fixed drawing paths']]:[]),...(plan?[['schedule',!plan.replay_in_progress&&replayState(plan)!=='unverified'?(replayState(plan)==='full'?'Checked motion replay':'Checked prefix replay'):plan.full_schedule?'Full schedule · validation pending or rejected':'Earlier checked prefix']]:[])];
    const show=(mode:string)=>{
      if(playback){clearInterval(playback);playback=null;}
      $('#robot-play').classList.add('hidden');
      $<HTMLButtonElement>('#robot-play').disabled=false;
      $('#playback-load').classList.add('hidden');
      $<HTMLButtonElement>('#robot-play').onclick=null;
      viewer.clearSequence();$('#timing-panel').classList.add('hidden');
      $('#motion-reuse-panel').classList.add('hidden');
      viewer.camera.zoom=1;
      $('#frame-label').textContent='Shared world frame · meters';
      $('#scale-label').textContent=`Object: ${manifest.bounds_m[1].map((v,i)=>((v-manifest.bounds_m[0][i])*100).toFixed(1)).join(' × ')} cm`;
      viewer.grid.position.set(origin[0],origin[1],-.0003);
      viewer.cell.visible=true;viewer.platform.visible=true;
      viewer.setCellReferenceVisible(cellReferenceVisible);
      for(const arm of arms)viewer.highlightCellLinks(arm.id,[]);
      for(const arm of arms){viewer.setCellArmBase(arm.id,arm.world_from_base);viewer.setCellArmPose(arm.id,arm.initial_positions);}
      viewer.setCurves([]);
      viewer.focus('cell');
      $('#metrics').innerHTML=setupMetrics;
      if(mode==='mounting'&&mountAssessment){
        renderMountAssessment(mountAssessment,arms,cell,partition);
      }
      else if(mode==='partition'&&partition){
        renderGeometricPartition(partition,cell,arms,allocation);
      }
      else if(mode==='allocation'&&allocation&&construction){
        renderCellAllocation(allocation,construction,cell,arms);
      }
      else if(mode==='material-order'&&allocationProblem&&allocation&&construction){
        renderMaterialOrders(allocationProblem,allocation,construction,cell);
      }
      else if(mode==='orientation'&&orientationProbe){
        renderPenOrientationProbe(orientationProbe,arms);
      }
      else if(mode==='restarts'&&restart&&allocation){
        renderPreparedMotions({...allocation,motion_preparation:{status:'PARTIAL_RESTART_TRIAL',prepared_strokes:1,requested_strokes:1,failures:[],motions:[{mode_id:'restart-trial',arm:restart.arm,stroke_id:restart.stroke_id,trajectory:restart.trajectory}]}},arms,restart);
      }
      else if(mode==='prepared'&&allocation?.motion_preparation){
        renderPreparedMotions(allocation,arms);
      }
      else if(mode==='scale'&&construction){
        renderScaleStudy(construction);
      }
      else if(mode==='schedule'&&plan){
        $('#title').textContent=id==='validation'?'Replay the saved two-arm build.':'Coordinate two pens on one clock.';
        $('#subtitle').textContent='Inspect simultaneous robot motion, automatic joins, feed commands and the checked construction prefix.';
        renderCellSchedule(plan,arms,motionReuse);
      }
      else if(candidates)renderCellCandidates(candidates,arms);
      if($<HTMLButtonElement>('#robot-play').classList.contains('hidden')) {
        $('#playback-load').classList.remove('hidden');
        $('#playback-load').textContent=plan?'Choose “Checked motion replay” in Planning result to play the saved build.':
          unavailable.length?unavailable.join(' '):'This run has no saved full-build playback yet.';
      }
      $('#metrics').insertAdjacentHTML('afterbegin',`<label class="control">Planning result<select class="method-select" id="cell-mode" aria-label="Planning result">${options.map(([value,label])=>`<option value="${value}" ${value===mode?'selected':''}>${label}</option>`).join('')}</select></label>`);
      $('#metrics').insertAdjacentHTML('afterbegin',metric(arms.length===1?'Initial drawing pen angle':'Configured drawing pen angle',penAngleLabel));
      const setupDescription=cellConfig.setup_description ?? (arms.length===1
        ? 'One Flexiv arm with a rigid pen mount. The initial pen lean is a starting preference; motion planning can adjust orientation to clear deposited material. Tool dimensions and process parameters remain provisional.'
        : undefined);
      if(setupDescription)$('#metrics').insertAdjacentHTML('beforeend',`<p class="description">${escape(setupDescription)}</p>`);
      $('#metrics').insertAdjacentHTML('beforeend',`<label class="control"><input type="checkbox" id="cell-reference" ${cellReferenceVisible?'checked':''}> Show source mesh reference</label>`);
      $('#cell-reference').addEventListener('change',()=>{
        cellReferenceVisible=$<HTMLInputElement>('#cell-reference').checked;
        viewer.setCellReferenceVisible(cellReferenceVisible);
      });
      if(timingEligibility&&mode!=='restarts'&&mode!=='schedule'){
        const unresolved=timingEligibility.modes.filter(row=>row.status==='TIMING_UNRESOLVED');
        const timed=timingEligibility.modes.filter(row=>row.status==='TIMED').length;
        $('#metrics').insertAdjacentHTML('afterbegin',`<section class="planner-checkpoint" aria-label="Fixed-path timing"><strong>Fixed-path timing · ${timed}/${timingEligibility.requested_modes}</strong><p>${unresolved.length} solver-unresolved ${unresolved.length===1?'mode':'modes'}. These modes are excluded from allocation; the drawing geometry is retained.${timingEligibility.active_mode?` Currently timing ${escape(timingEligibility.active_mode)}.`:''}</p>${unresolved.slice(0,6).map(row=>`<p>${escape(row.mode_id)}: ${escape(row.reason)}</p>`).join('')}<a href="${url('allocation_timing.json')}" target="_blank">Open timing eligibility ↗</a></section>`);
      }
      if(checkpoint&&mode!=='restarts'&&mode!=='schedule'){
        const phases:Record<string,string>={GENERATING_FIXED_CATALOGUE:'Generating drawing paths',ALLOCATING_FIXED_MODES:'Timing and allocating fixed paths',REALIZING_SELECTED_MOTIONS:'Preparing complete robot motions',PLANNING_TIME_BUDGET:'Planning budget reached',SELECTED_SCHEDULE_REPLAYED:'Complete modeled replay passed',SUPERSEDED_TIMING_FORMULATION:'Retained timing diagnostic'};
        const phase=allocation?.motion_preparation?.status==='SELECTED_MOTIONS_PREPARED'&&run.stages.validation?.status==='running'?'Checking complete shared schedule':phases[checkpoint.status]??checkpoint.status.replaceAll('_',' ').toLowerCase();
        const active=allocation?.motion_preparation?.active_motion;
        $('#metrics').insertAdjacentHTML('afterbegin',`<section class="planner-checkpoint" aria-label="Planner checkpoint"><strong>${escape(phase)}</strong><p>Saved checkpoint · ${escape(new Date(checkpoint.updated_at).toLocaleTimeString())}${allocation?.motion_preparation?` · ${allocation.motion_preparation.prepared_strokes}/${allocation.motion_preparation.requested_strokes} motions prepared`:''}. This reports planning progress; physical fabrication is unverified.</p><p>${checkpoint.no_time_limit?'No overall planning timeout or fabrication-time target.':'Priority: complete the pipeline. The 30-minute fabrication target is advisory.'}</p>${active?`<p>${allocation?.motion_preparation?.status==='PREPARING_SELECTED_MOTIONS'?'Preparing':'Last attempted'} ${escape(active.arm)} · ${escape(active.stroke_id)}.</p>`:''}${checkpoint.detail?`<p>${escape(checkpoint.detail)}</p>`:''}<a href="${url('cell_pipeline_result.json')}" target="_blank">Open planner report ↗</a></section>`);
      }
      if(id==='validation'&&validationSummary){
        const panel=$('#timing-panel');panel.classList.remove('hidden');
        panel.insertAdjacentHTML('afterbegin',`<h2>Validation evidence</h2><p>Snapshot ${escape(new Date(validationSummary.generated_at).toLocaleString())}. Modeled checks and unmeasured physical properties are separate; the 30-minute target is advisory.</p><table><thead><tr><th>Check</th><th>Result</th><th>Scope</th></tr></thead><tbody>${validationSummary.checks.map(c=>`<tr><td>${escape(c.name)}</td><td>${escape(c.status.replaceAll('_',' ').toLowerCase())}</td><td>${escape(c.detail)}${c.evidence?` <a href="${url(c.evidence)}" target="_blank">Evidence ↗</a>`:''}</td></tr>`).join('')}</tbody></table><a href="${url('validation_report.json')}" target="_blank">Open validation report ↗</a>`);
      }
      for(const message of unavailable)$('#metrics').insertAdjacentHTML('beforeend',`<p class="description">${escape(message)}</p>`);
      $('#cell-mode').addEventListener('change',()=>show($<HTMLSelectElement>('#cell-mode').value));
    };
    show(previousCellMode&&options.some(([value])=>value===previousCellMode)?previousCellMode:id==='motion'&&plan?.full_schedule?'schedule':id==='motion'&&restart?'restarts':id==='cell'&&mountAssessment?'mounting':id==='motion'&&allocation?.motion_preparation?.motions.length?'prepared':['timing','validation'].includes(id)&&plan?.actions.length?'schedule':id==='cell'&&allocation?'allocation':id==='cell'&&partition?'partition':id!=='cell'&&plan?.replay?.status==='CERTIFIED_GEOMETRIC_REPLAY'?'schedule':allocation?'allocation':plan?.actions.length?'schedule':'candidates');
    return;
  }
  if (id === 'asset') {
    const dimensions = manifest.bounds_m[1].map((v, i) => Math.round((v - manifest.bounds_m[0][i]) * 1000));
    $('#metrics').innerHTML = `<h2>Model overview</h2><div class="metric big"><strong>${(manifest.source_triangles / 1e6).toFixed(2)}<span style="font-size:15px"> M</span></strong><span class="metric-label">Source triangles</span></div>${metric('Processing triangles',number(manifest.geometry_triangles))}${metric('Dimensions',dimensions.join(' × ') + ' mm')}${metric('Texture',manifest.texture.has_base_color ? 'Preserved' : 'None')}${metric('Up direction','Z in build frame')}<a class="download" href="${url('asset_manifest.json')}" target="_blank">Open asset manifest ↗</a>`;
    $('#note-text').textContent = 'The textured reference keeps the original appearance. The simplified geometry copy supports faster curve extraction; its neutral material does not replace the source texture.';
    $('#input-summary').textContent = manifest.source.name;
    $('#output-summary').textContent = 'Normalized textured model, geometry copy, and WIR3D input bundle.';
    $('#next-summary').textContent = 'Extract visually meaningful 3D curves and compare abstraction budgets.';
    $('#legend').innerHTML = '<span><i class="reference"></i>Source geometry</span>';
    return;
  }
  if (id === 'curves') {
    if(method==='selected' && (!run.artifacts.includes('selected_candidates.json') || !(run.stages.selection?.curve_count ?? 0))) method='geometric';
    const options = ['<option value="geometric">Geometry baseline</option>', '<option value="wir3d">WIR3D abstraction</option>'];
    if(run.artifacts.includes('surface_contours.json'))options.push('<option value="contours">Surface outlines and openings</option>');
    if(run.artifacts.includes('selected_candidates.json') && (run.stages.selection?.curve_count ?? 0)>0)options.push('<option value="selected">Budgeted selection</option>');
    const availableBudgets = (run.stages.wir3d?.budgets ?? []).filter(b => run.artifacts.includes(`wir3d_${b}.json`));
    if (!availableBudgets.includes(budget) && availableBudgets.length) budget = availableBudgets[0];
    const file = method === 'geometric' ? 'geometric_candidates.json' : method === 'contours' ? 'surface_contours.json' : method === 'selected' ? 'selected_candidates.json' : availableBudgets.includes(budget) ? `wir3d_${budget}.json` : null;
    const loadedCurves = file && run.artifacts.includes(file) ? (method==='selected'?await checkedArtifact<CurveSet>(file):await json<CurveSet>(url(file))) : null;
    if (request !== stageRequest) return;
    currentCurves = loadedCurves;
    $<HTMLInputElement>('#curve-slider').disabled = !currentCurves;
    $('#curve-count').textContent = currentCurves ? `${currentCurves.curves.length} / ${currentCurves.curves.length}` : '—';
    $('#metrics').innerHTML = `<h2>Curve candidates</h2><select class="method-select" id="method" aria-label="Curve generation method">${options.join('')}</select><div class="metric big"><strong>${currentCurves?.curves.length ?? '—'}</strong><span class="metric-label">Candidate strokes</span></div>${metric('Total stroke length',currentCurves ? `${currentCurves.total_length_m.toFixed(2)} m` : '—')}${metric('Physical connections',run.artifacts.includes('stroke_graph.json') ? 'See connected structure' : 'Not generated')}<p class="description">${method === 'contours' ? (currentCurves?.method === 'mesh_contours' ? 'Connected contours traced across shared triangle edges. Whole curves are retained across image occlusions; hidden portions remain part of each contour.' : 'Whole visible outlines and openings lifted onto the mesh. Depth jumps split curves; connections are not inferred from an image.') : method === 'geometric' ? 'Cross-sections extracted from the mesh. A geometric supplement to compare with the visual abstraction.' : method==='selected' ? 'Existing curves selected automatically for additional shape coverage within an assumed drawing budget.' : 'Visual and geometric optimization with WIR3D. Results appear here after the GPU run is imported.'}</p>${file ? `<a class="download" href="${url(file)}" target="_blank">Open curve artifact ↗</a>` : ''}`;
    $<HTMLSelectElement>('#method').value = method;
    $('#method').addEventListener('change', () => { method = $<HTMLSelectElement>('#method').value; selectStage('curves').catch(showError); });
    if (method === 'wir3d' && availableBudgets.length) {
      $('#method').insertAdjacentHTML('afterend', `<label class="control">Optimization budget<select id="budget" class="method-select" aria-label="Curve budget">${availableBudgets.map(b => `<option value="${b}">Target ${b}</option>`).join('')}</select></label>`);
      $<HTMLSelectElement>('#budget').value = String(budget);
      $('#budget').addEventListener('change', () => { budget = Number($<HTMLSelectElement>('#budget').value); selectStage('curves').catch(showError); });
    }
    if (currentCurves) {
      viewer.setCurves(currentCurves.curves);
      const slider = $<HTMLInputElement>('#curve-slider');
      slider.max = String(currentCurves.curves.length); slider.value = slider.max;
      viewer.setLimit(currentCurves.curves.length);
      $('#curve-count').textContent = `${slider.max} / ${slider.max}`;
      viewer.setSource($<HTMLInputElement>('#overlay').checked, true);
    } else empty('Curve output is not available yet', run.stages.wir3d?.message ?? 'The prepared job needs to run in the WIR3D GPU environment.');
    if (method === 'wir3d' && run.artifacts.includes('wir3d_keypoints.json')) {
      const keypoints = await json<{points:number[][]}>(url('wir3d_keypoints.json'));
      if (request !== stageRequest) return;
      const markers = keypoints.points.map((position,i) => ({id:`semantic-${i}`,position,ground:false}));
      $('#metrics').insertAdjacentHTML('beforeend', `${metric('Automatic keypoints',String(markers.length))}<label class="control">Show semantic keypoints<input id="keypoint-toggle" type="checkbox" checked></label><p class="description">Semantic features extracted automatically. These markers are not physical anchors or planned joins.</p>`);
      viewer.setJunctions(markers, true);
      $('#keypoint-toggle').addEventListener('change', () => viewer.setJunctions(markers,$<HTMLInputElement>('#keypoint-toggle').checked));
      if (!currentCurves) {
        $('#empty').classList.add('hidden');
        viewer.setSource(true,true);
        $('#output-summary').textContent = 'Automatic semantic keypoints; curve optimization in progress.';
      }
    }
    if(currentCurves?.selection)renderSelection(currentCurves);
    $('#note-text').textContent = method==='contours' ? (currentCurves?.method==='mesh_contours' ? 'Each curve follows the mesh continuously. Surface connectivity preserves whole loops before selection; it does not establish printable support or recognition.' : 'Visible outlines and openings are traced onto the original surface. Discontinuities remain separate; this is a candidate pool, not a connected or accepted sculpture.') : method==='selected' ? (currentCurves?.selection?.algorithm ? 'Whole curves selected for geometric coverage and raw drawing cost. Added supports and full motion costs are checked afterward; recognition and physical fabrication remain unverified.' : 'A subset selected for shape coverage and connected drawing cost. The reserved overhead, recognition and fabrication feasibility still need validation.') : method === 'geometric' ? 'These are genuine geometry-derived candidates, not WIR3D results. The curves have not yet been selected for recognition, connected, or checked for fabrication.' : 'WIR3D curves capture appearance. Physical connections, supports, and robot paths are generated in the following stages.';
    $('#input-summary').textContent = method==='selected' ? 'Available source curves, fixed selection views, semantic keypoints and an assumed drawing budget.' : method === 'contours' ? 'The same mesh and fixed camera directions; no hand-drawn features.' : method === 'geometric' ? 'Simplified mesh, sampled along three coordinate axes.' : 'Textured model, automatic semantic keypoints, and multiview renders.';
    $('#output-summary').textContent = !currentCurves && method === 'wir3d' && run.artifacts.includes('wir3d_keypoints.json') ? 'Automatic semantic keypoints; curve optimization in progress.' : '3D cubic Bézier curves with preserved geometry and provenance.';
    $('#next-summary').textContent = 'Select meaningful curves and connect them into one supported structure.';
    $('#legend').innerHTML = '<span><i></i>Candidate strokes</span><span><i class="reference"></i>Reference overlay</span>';
    if(run.artifacts.includes('candidate_comparison.json')) {
      try {
        const comparison=await checkedArtifact<CandidateComparison>('candidate_comparison.json');
        if(request!==stageRequest)return;
        renderComparison(comparison,file);
      } catch(error) {
        if(request!==stageRequest)return;
        $('#comparison-panel').classList.remove('hidden');
        $('#comparison-panel').textContent=`Comparison needs a refresh: ${String(error)}`;
      }
    }
    if(method === 'selected' && run.artifacts.includes('abstraction_review.json')) {
      const review = await checkedArtifact<{status:string;message:string}>('abstraction_review.json');
      if(request !== stageRequest)return;
      if(review.status === 'NEEDS_REVISION') {
        $('#badge').textContent = 'Shape needs revision';
        $('#badge').classList.add('pending');
        $('#note-text').textContent = review.message;
      } else if(review.status === 'VISUAL_CANDIDATE') {
        $('#badge').textContent = 'Visual candidate · unvalidated';
        $('#badge').classList.add('pending');
        $('#note-text').textContent = review.message;
      }
    }
    return;
  }
  if (id === 'graph' && run.artifacts.includes('stroke_graph.json')) {
    const supported = structureVariant === 'backbone' && ['backbone_graph.json','backbone_structure.json','backbone_process.json','backbone_pen_strokes.json'].every(file=>run.artifacts.includes(file));
    const prefix = supported ? 'backbone_' : '';
    const [graph,structure,process,strokes] = await Promise.all([
      json<StrokeGraph>(url(supported?'backbone_graph.json':'stroke_graph.json')),json<Structure>(url(supported?'backbone_structure.json':'structural_screen.json')),json<ProcessScreen>(url(supported?'backbone_process.json':'process_screen.json')),json<CurveSet>(url(`${prefix}pen_strokes.json`)),
    ]);
    if (request !== stageRequest) return;
    processRows = new Map(process.edges.map(e=>[e.id,e]));
    currentCurves = strokes;
    viewer.setCurves(strokes.curves);
    viewer.model.position.z = graph.candidate_to_build?.[2]?.[3] ?? 0;
    viewer.setJunctions(graph.nodes,false);
    viewer.setSource($<HTMLInputElement>('#overlay').checked,true);
    const slider = $<HTMLInputElement>('#curve-slider');
    slider.disabled = false; slider.max = String(strokes.curves.length); slider.value = slider.max;
    $('#curve-count').textContent = `${slider.max} / ${slider.max}`;
    $('#badge').textContent = 'Pen sketch · uncalibrated';
    $('#badge').classList.add('pending');
    const components = new Map(graph.nodes.map(n=>[n.id,n.id]));
    const root = (id:string):string => { const next=components.get(id) ?? id; return next===id?id:root(next); };
    for(const edge of graph.edges as BuildAction[])components.set(root(edge.start),root(edge.end));
    const m = graph.metrics ?? {
      input_curves:strokes.curves.length, material_members:graph.edges.length,
      material_components:new Set(graph.nodes.map(n=>root(n.id))).size,
      ground_contacts:graph.nodes.filter(n=>n.ground).length,
      total_length_m:strokes.total_length_m, added_length_m:0,
    };
    $('#metrics').innerHTML = `<h2>Continuous pen strokes</h2><div class="metric big"><strong>${number(strokes.curves.length)}</strong><span class="metric-label">Proposed pen strokes</span></div>${metric('Material graph members',number(m.material_members))}${metric('Material components',number(m.material_components))}${metric('Ground contacts',number(m.ground_contacts))}${metric('Object dimensions',manifest.bounds_m[1].map((v,i)=>((v-manifest.bounds_m[0][i])*100).toFixed(1)).join(' × ')+' cm')}${metric('Total drawing length',m.total_length_m.toFixed(2)+' m')}${metric('Added material',m.added_length_m.toFixed(2)+' m')}<p class="description">Each contour stays continuous through incidental intersections. Graph members track connections; they are not separate pen starts. Ordering can add intentional cooling pauses.</p><a class="download" href="${url(`${prefix}pen_strokes.json`)}" target="_blank">Open continuous strokes ↗</a>`;
    if(run.artifacts.includes('backbone_graph.json')) {
      $('#metrics').insertAdjacentHTML('afterbegin', `<label class="control">Structure version<select id="structure-version" aria-label="Structure version"><option value="backbone" ${supported?'selected':''}>Automatic supports · intact contours</option><option value="original" ${!supported?'selected':''}>Original connected graph</option></select></label>`);
      $('#structure-version').addEventListener('change',()=>{structureVariant=$<HTMLSelectElement>('#structure-version').value;selectStage('graph').catch(showError);});
      if(supported) {
        const report=await checkedArtifact<{selected_posts:string[];added_length_m:number;complete_actions:number}>('backbone_report.json');
        if(request!==stageRequest)return;
        $('#metrics').insertAdjacentHTML('beforeend', `<h3>Automatic support proposal</h3>${metric('Added posts',String(report.selected_posts.length))}${metric('Added length',`${(report.added_length_m*100).toFixed(1)} cm`)}${metric('Intact construction actions',String(report.complete_actions))}<p class="description">Whole contours are preserved. Grounded starts do not prove that a stroke stays upright while being extruded.</p>`);
      }
    }
    $('#graph-controls').classList.remove('hidden');
    $('#graph-controls').innerHTML = `<h2>Geometry & pen checks</h2>${metric('Geometry needs repair',number(process.rejected_actions))}${metric('Need pen calibration',number(process.calibration_required_actions))}${metric('Cooled frame',structure.failure ? 'Analysis unresolved' : structure.modeled_pass ? 'Modeled pass' : 'Modeled failure')}${metric('Peak deflection',structure.maximum_displacement_m !== undefined ? (structure.maximum_displacement_m*1000).toFixed(3)+' mm' : 'Unavailable')}<div class="control"><label>Highlight geometry issues <input id="rejected" type="checkbox"></label></div><div class="control"><label>Show physical junctions <input id="junctions" type="checkbox"></label></div>${['framework','artistic','connector','support','base','brace'].filter(role=>strokes.curves.some(curve=>displayRole(curve)===role)).map(role=>`<div class="control"><label>${role[0].toUpperCase()+role.slice(1)} strokes <input data-role="${role}" type="checkbox" checked></label></div>`).join('')}<div class="control"><label>Deformation overlay <span id="deformation-scale">Off</span></label><input id="deformation" aria-label="Deformation magnification" type="range" min="0" max="100" value="0" step="5" ${structure.failure ? 'disabled' : ''}></div><p class="description">${structure.failure ? 'The structural calculation is unresolved. This is not a demonstrated physical failure, and no deformation field is shown.' : 'Assumed material, rigid welds, and a clamped bed. Purple deformation lines are magnified, not measured.'}</p><a class="download" href="${url(supported?'backbone_process.json':'process_screen.json')}" target="_blank">Open pen diagnostics ↗</a>`;
    $('#rejected').addEventListener('change',()=>viewer.highlight($<HTMLInputElement>('#rejected').checked ? process.edges.filter(e=>!e.modeled_pass).map(e=>e.id) : []));
    $('#junctions').addEventListener('change',()=>{viewer.junctions.visible = $<HTMLInputElement>('#junctions').checked;});
    document.querySelectorAll<HTMLInputElement>('[data-role]').forEach(control=>control.addEventListener('change',()=>viewer.setRoleVisible(control.dataset.role!,control.checked)));
    $('#deformation').addEventListener('input',()=>{const scale = Number($<HTMLInputElement>('#deformation').value); $('#deformation-scale').textContent = scale ? `${scale}×` : 'Off'; viewer.setDeformation(structure.edges,scale);});
    const sourceMethod=graph.source_method ?? strokes.method;
    $('#note-text').textContent = `This ${sourceMethod?.includes('wireprint') ? 'WirePrint framework' : sourceMethod === 'reference_fixture' ? 'reference fixture' : sourceMethod === 'geometric_sections' ? 'geometric baseline' : sourceMethod === 'budgeted_selection' ? 'budgeted selection' : 'WIR3D sketch'} proposes ${strokes.curves.length} continuous pen strokes. Unknown free-space, curvature, and cooling capabilities are calibration questions, not automatic rejections. ${process.rejected_actions} strokes still need geometric repair. Construction order and robot access remain to be checked.`;
    $('#input-summary').textContent = `${m.input_curves} curves, with contacts detected in 3D.`;
    $('#output-summary').textContent = `${m.material_components} material component${m.material_components === 1 ? '' : 's'}, real connectors and supports, cooled-frame and deposition screens.`;
    $('#next-summary').textContent = 'Choose a sparse sketch, draw its base and frame first, then plan contours and deliberate joins.';
    $('#legend').innerHTML = [...new Set(strokes.curves.map(displayRole))].map(role=>`<span><i style="background:#${curveColors[role].toString(16).padStart(6,'0')}"></i>${role[0].toUpperCase()+role.slice(1)}</span>`).join('');
    $('#criteria-panel').classList.remove('hidden');
    const criteria = [['minimum_stroke_m','Minimum continuous stroke'],['maximum_free_tip_length_m','Length before reaching another anchor'],['maximum_horizontal_reach_m','Horizontal reach'],['maximum_downward_motion_m','Downward drawing'],['minimum_curvature_radius_m','Bend radius']];
    $('#criteria-panel').innerHTML = `<h2>A practical pen check</h2><p>Begin with a bed line and join, an upright stroke, a bridge, and a curved stroke. Use those results to set the pen envelope. This preview does not infer printer-style overhang rules or require stopping at every crossing.</p><table><thead><tr><th>Capability</th><th>Current evidence</th></tr></thead><tbody>${criteria.map(([key,label])=>`<tr><td>${label}</td><td>${process.settings[key] == null ? 'Not measured — no hard limit' : `${Number(process.settings[key])*1000} mm (${process.settings.calibrated ? 'calibrated' : 'assumed, advisory only'})`}</td></tr>`).join('')}<tr><td>Start at the bed or existing material</td><td>Checked during construction planning</td></tr><tr><td>Keep the path above the bed</td><td>Geometric constraint</td></tr></tbody></table><h2>Drawing workload at an assumed speed</h2><p>At an assumed <strong id="speed-value">${process.timing_assumption.speed_m_s*1000} mm/s</strong>, this sketch needs <strong id="draw-time">${(process.timing_assumption.deposition_only_s/60).toFixed(1)} minutes</strong> of aggregate drawing across the pens. Parallel elapsed time depends on assignment and construction dependencies. Joins, travel, cooling, and startup add time. This is a what-if calculation, not a pen measurement.</p><div class="control"><label>Assumed drawing speed<input id="speed-assumption" aria-label="Assumed drawing speed" type="range" min="1" max="10" step="0.5" value="${process.timing_assumption.speed_m_s*1000}"></label></div><p>Complete modeled duration also includes travel, joining, and cooling. No fabrication-time target is imposed.</p>`;
    $('#speed-assumption').addEventListener('input',()=>{const speed = Number($<HTMLInputElement>('#speed-assumption').value); $('#speed-value').textContent = `${speed} mm/s`; $('#draw-time').textContent = `${(m.total_length_m*1000/speed/60).toFixed(1)} minutes`;});
    return;
  }
  if (id === 'construction' && ['partial','complete','failed'].includes(status) && run.artifacts.includes('construction_plan.json')) {
    const supported=structureVariant==='backbone'&&run.artifacts.includes('backbone_construction.json');
    const constructionFile=supported?'backbone_construction.json':'construction_plan.json';
    const [plan,graph] = await Promise.all([json<Construction>(url(constructionFile)),json<StrokeGraph>(url(supported?'backbone_graph.json':'stroke_graph.json'))]);
    let sectionSchedule:CoordinatedPlan|null=null;
    if(run.artifacts.includes('offline_dual_arm_plan.json')) {
      const saved=await checkedArtifact<CoordinatedPlan>('offline_dual_arm_plan.json');
      if(saved.section_schedule||saved.partial_section_schedule) {
        sectionSchedule=saved;
        const source=new Map(plan.actions.map(a=>[a.id,a]));
        plan.actions=saved.actions.filter(a=>!a.auxiliary).map(a=>{
          const original=source.get(a.stroke_id);
          if(!original)throw Error(`Missing source section ${a.stroke_id}`);
          let segments=original.segments;
          if(original.closed && a.start_visit) {
            const boundary=original.junction_visits?.[a.start_visit]?.segment_boundary;
            if(boundary===undefined)throw Error(`Missing closed-contour start for ${a.stroke_id}`);
            segments=[...segments.slice(boundary),...segments.slice(0,boundary)];
          }
          const start=a.start_anchor ?? (a.reverse?original.end:original.start);
          return {...original,start,end:original.closed?start:a.reverse?original.start:original.end,
            segments:a.reverse?segments.slice().reverse().map(s=>s.slice().reverse()):segments};
        });
      }
    }
    if (request !== stageRequest) return;
    if (!plan.actions.length) {
      empty('No construction order found',plan.status.replaceAll('_',' ').toLowerCase());
      $('#metrics').innerHTML = `<h2>Construction search</h2>${metric('Result',escape(plan.status))}<p class="description">A bounded search failure is not proof of physical impossibility.</p>`;
      return;
    }
    viewer.setCurves(plan.actions);
    viewer.model.position.z = graph.candidate_to_build?.[2]?.[3] ?? 0;
    viewer.setSource($<HTMLInputElement>('#overlay').checked,true);
    $('#badge').textContent = sectionSchedule?'Saved section order':supported?'Intact contours · automatic supports':'Order preview · geometry only';
    $('#metrics').innerHTML = `<h2>Construction preview</h2><div class="metric big"><strong>${plan.actions.length}</strong><span class="metric-label">Directed drawing strokes</span></div>${metric('Order source',sectionSchedule?'Saved robot schedule':'Geometry search')}${metric('Robot motion',sectionSchedule?'See motion validation':'Not planned')}${metric('Material / pen','Uncalibrated')}<p class="description">Starts use the bed or existing material. A closed contour can start at an available junction. This is a geometric order, not a physically validated robot program.</p><a class="download" href="${url(constructionFile)}" target="_blank">Open construction preview ↗</a>`;
    $('#graph-controls').classList.remove('hidden');
    $('#graph-controls').innerHTML = `<h2>Replay the build</h2><div class="control"><label>Completed strokes <span id="build-count"></span></label><input id="build-step" aria-label="Completed construction strokes" type="range" min="0" max="${plan.actions.length}" value="0"></div><div class="playback-buttons"><button id="build-previous">Previous</button><button id="build-play">Play</button><button id="build-next">Next</button></div><p class="description" id="build-detail"></p><p class="description">Playback is schematic; it does not represent printing speed or cooling time. The dot marks the latest stroke’s starting anchor.</p>`;
    const drawStep = (step:number) => {
      step = Math.min(plan.actions.length,Math.max(0,step));
      $<HTMLInputElement>('#build-step').value = String(step);
      $('#build-count').textContent = `${step} / ${plan.actions.length}`;
      viewer.setLimit(step); viewer.select(step-1);
      const action = plan.actions[step-1];
      viewer.setJunctions(action ? graph.nodes.filter(n=>n.id === action.start) : [],true);
      $('#build-detail').textContent = action && sectionSchedule ? `${action.id} · ${action.stage} · source ${action.parent_curve}. Starts at ${action.start}. ${(action.length_m*1000).toFixed(1)} mm.` : action ? `${action.id}: ${(action.length_m*1000).toFixed(1)} mm. Starts ${action.depends_on.length ? `from material drawn in step ${action.depends_on[0]+1}` : 'on the bed'}. ${action.existing_join_nodes?.length ?? 0} existing junctions along this stroke.` : 'Begin with a bare build surface.';
    };
    const stop = () => { if (playback) clearInterval(playback); playback=null; $('#build-play').textContent='Play'; };
    $('#build-step').addEventListener('input',()=>{stop();drawStep(Number($<HTMLInputElement>('#build-step').value));});
    $('#build-previous').addEventListener('click',()=>{stop();drawStep(Number($<HTMLInputElement>('#build-step').value)-1);});
    $('#build-next').addEventListener('click',()=>{stop();drawStep(Number($<HTMLInputElement>('#build-step').value)+1);});
    $('#build-play').addEventListener('click',()=>{
      if (playback) {stop();return;}
      if (Number($<HTMLInputElement>('#build-step').value) === plan.actions.length) drawStep(0);
      $('#build-play').textContent='Pause';
      playback=setInterval(()=>{const next=Number($<HTMLInputElement>('#build-step').value)+1;drawStep(next);if(next>=plan.actions.length)stop();},500);
    });
    drawStep(0);
    $('#note-text').textContent = sectionSchedule ? 'This is the section order saved in the complete robot schedule. Use Robot motion to inspect timed travel, cooling, and pen events; this stepper only shows geometry.' : supported ? 'A fixed-catalogue solver selects start supports and an intact-contour order. Forward replay checks grounded completed-stroke prefixes and member coverage. Robot motion, cooling, and stability during extrusion remain unverified.' : 'Backward search removes a possible last stroke while retaining grounded earlier material. Forward replay checks anchors and member coverage. Robot motion, cooling, and intermediate stiffness remain unverified.';
    $('#input-summary').textContent = 'Connected material graph and continuous pen contours.';
    $('#output-summary').textContent = `${plan.actions.length} directed strokes with explicit starting dependencies.`;
    $('#next-summary').textContent = 'Search robot paths and pen-off transits while preserving alternative construction choices.';
    $('#legend').innerHTML = '<span><i></i>Drawn material</span><span><i style="background:#295d48"></i>Latest stroke</span>';
    return;
  }
  if(['timing','validation'].includes(id) && run.artifacts.includes('timed_schedule.json')) {
    const schedule=await checkedArtifact<TimedSchedule>('timed_schedule.json');
    const robot=await json<RobotRig>(url('robot_manifest.json'));
    const sequence=await checkedArtifact<SequencePreview>('sequence_motion.json');
    const placement=await json<{selected:{center_xy_m:number[];height_above_table_m:number};assumptions:{table_z_m:number}}>(url('placement_search.json'));
    if(request!==stageRequest)return;
    await loadModel('robot');
    if(request!==stageRequest)return;
    viewer.setToolModel(sequence.tool_model);
    viewer.setPlatform(placement.selected.center_xy_m,placement.selected.height_above_table_m+placement.assumptions.table_z_m,placement.assumptions.table_z_m,
      manifest.bounds_m[1][0]-manifest.bounds_m[0][0]+.04,manifest.bounds_m[1][1]-manifest.bounds_m[0][1]+.04);
    const seek=renderTiming(schedule,robot);
    if(run.artifacts.includes('smooth_collisions.json')) {
      try {
        const audit=await checkedArtifact<SmoothAudit>('smooth_collisions.json');
        if(request!==stageRequest)return;
        renderSmoothAudit(audit,schedule,robot,seek);
      } catch(error) {
        if(request!==stageRequest)return;
        $('#audit-panel').classList.remove('hidden');
        $('#audit-panel').innerHTML=`<h2>Smooth-path audit unavailable</h2><p>${escape(error)}</p>`;
      }
    }
    if(id==='validation') {
      $('#title').textContent='Check the path that will actually run.';
      $('#subtitle').textContent='Audit the timed cubic geometry against the modeled arm, work surfaces and growing material.';
    }
    return;
  }
  if (id === 'motion' && run.artifacts.includes('robot_manifest.json')) {
    const robot = await json<RobotRig & {variant:string;joint_names:string[];missing:string[]}>(url('robot_manifest.json'));
    if (request !== stageRequest) return;
    $('#badge').textContent = 'Setup preview';
    $('#title').textContent = 'Set up the arm.';
    $('#subtitle').textContent = 'A manufacturer robot model for developing the pen mount and motion planner.';
    $('#frame-label').textContent = 'Robot base · meters';
    $('#scale-label').textContent = 'Simulation reference · mount and pen pending';
    $('#metrics').innerHTML = `<h2>Flexiv setup</h2>${metric('Reference model',escape(robot.variant))}${metric('Active joints',String(robot.joint_names.length))}${metric('Full-job motion','Not generated')}<p class="description">The exact physical arm variant has not been confirmed. This reference uses the manufacturer’s geometry and joint limits.</p><a class="download" href="${url('robot_manifest.json')}" target="_blank">Open robot manifest ↗</a>`;
    if (run.artifacts.includes('placement_search.json')) {
      const placement = await json<{selected:{center_xy_m:number[];height_above_table_m:number;coverage:number;build_to_robot_base:number[][]};assumptions:{table_z_m:number};candidates:unknown[];curve_sample_verification:{coverage:number;samples:unknown[]}}>(url('placement_search.json'));
      if (request !== stageRequest) return;
      const chosen = placement.selected;
      const strokes = await json<{curves:Curve[]}>(url('pen_strokes.json'));
      if (request !== stageRequest) return;
      const matrix = chosen.build_to_robot_base;
      viewer.setCurves(strokes.curves.map(curve => ({...curve, segments:curve.segments.map(segment => segment.map(point => matrix.slice(0,3).map(row => row[0]*point[0]+row[1]*point[1]+row[2]*point[2]+row[3])))})));
      viewer.setPlatform(chosen.center_xy_m, chosen.height_above_table_m + placement.assumptions.table_z_m, placement.assumptions.table_z_m,
        manifest.bounds_m[1][0]-manifest.bounds_m[0][0]+.04, manifest.bounds_m[1][1]-manifest.bounds_m[0][1]+.04);

      $('#metrics').innerHTML += `<h2>Platform placement search</h2>${metric('Positions compared',String(placement.candidates.length))}${metric('Center X / Y',chosen.center_xy_m.map(v=>(v*100).toFixed(0)).join(' / ')+' cm')}${metric('Height above table',(chosen.height_above_table_m*100).toFixed(0)+' cm')}${metric('Volume probe IK',(chosen.coverage*100).toFixed(0)+'%')}${metric('Curve sample IK',(placement.curve_sample_verification.coverage*100).toFixed(0)+'% of '+placement.curve_sample_verification.samples.length)}<p class="description">Provisional grid search: table at arm-base Z=0, vertical pen, assumed 16 cm axial tool offset. Ranked by sampled reach then joint-limit margin. The translucent riser illustrates the selected build plane. Placement ranking uses reach only; any path collision audit is shown separately below.</p><a class="download" href="${url('placement_search.json')}" target="_blank">Open placement results ↗</a>`;
    }
    $('#note-text').textContent = 'This is a robot model preview. The measured pen-tip transform, pen and mount geometry, process calibration, and collision-checked motion plan are still required.';
    $('#input-summary').textContent = 'Pinned Flexiv robot description and joint limits.';
    $('#output-summary').textContent = 'Arm reference posture and provisional build-platform placement.';
    $('#next-summary').textContent = 'Confirm the arm variant and calibrate the mounted pen.';
    $('#legend').innerHTML = '<span><i class="reference"></i>Flexiv reference posture</span><span>Translucent riser · placed strokes</span>';
    await loadModel('robot');
    if (request !== stageRequest) return;
    if (run.artifacts.includes('drawing_motion.json') && robot.kinematic_chain) {
      const motionResponse = await fetch(url('drawing_motion.json'));
      if (!motionResponse.ok) throw new Error('Could not load drawing motion');
      const motionText = await motionResponse.text();
      const motion = JSON.parse(motionText) as {status:string;requested_strokes:number;flange_to_tcp:number[][];strokes:{id:string;status:string;positions?:number[][];maximum_sampled_interpolation_error_m?:number;tcp_targets_m:number[][];source_stroke:string}[]};
      if (request !== stageRequest) return;
      if (motionMode === 'sequence' && run.artifacts.includes('sequence_motion.json')) {
        const sequence = await checkedArtifact<SequencePreview>('sequence_motion.json');
        if (request !== stageRequest) return;
        renderSequence(sequence,robot);
        return;
      }
      let collision:CollisionAudit|null = null;
      if (run.artifacts.includes('drawing_collisions.json')) {
        const candidate = await json<CollisionAudit>(url('drawing_collisions.json'));
        const digest = [...new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(motionText)))].map(v=>v.toString(16).padStart(2,'0')).join('');
        if (request !== stageRequest) return;
        if (candidate.source_motion_sha256 === digest) collision = candidate;
      }
      viewer.setToolModel(collision?.tool_model);
      const collisionRows = new Map(collision?.strokes.map(row=>[row.id,row]) ?? []);
      const paths = motion.strokes.filter(stroke => stroke.positions?.length);
      if (paths.length) {
        $('#title').textContent = 'Follow the drawing strokes.';
        $('#badge').textContent = 'Motion preview';
        $('#subtitle').textContent = collision ? 'Inspect drawing paths and the supplied collision model. Drawing contact and travel are separate checks.' : 'Inspect joint motion along individual strokes. Travel between strokes and collision checks are still pending.';
        $('#metrics').innerHTML = `<details class="setup-details"><summary>Robot & placement assumptions</summary>${$('#metrics').innerHTML}</details>`;
        if (run.artifacts.includes('sequence_motion.json')) {
          $('#metrics').insertAdjacentHTML('beforeend','<button id="sequence-mode" class="mode-button">View construction + travel</button>');
          $('#sequence-mode').addEventListener('click',()=>{motionMode='sequence';selectStage('motion').catch(showError);});
        }
        $('#metrics').insertAdjacentHTML('beforeend', `<h2>Drawing motion</h2>${metric('Paths generated',`${paths.length} / ${motion.requested_strokes}`)}<div class="motion-controls"><label class="control">Stroke<select id="motion-stroke" aria-label="Motion stroke">${paths.map((stroke,i)=>`<option value="${i}">${i+1}. ${escape(stroke.id)}</option>`).join('')}</select></label><label class="control">Waypoint <span id="motion-count"></span></label><div class="metric"><span>TCP sample error</span><strong id="motion-error">—</strong></div><input id="motion-sample" aria-label="Motion waypoint" type="range" min="0" value="0"><button id="motion-play">Play stroke</button><button id="motion-refresh">Refresh results</button></div><p class="description">Blue line: assumed pen axis. Playback shows individual drawing paths at a schematic speed. Jumps between strokes are selection changes, not planned travel.</p><a class="download" href="${url('drawing_motion.json')}" target="_blank">Open motion preview ↗</a>`);
        $('#metrics').insertAdjacentHTML('beforeend', collision ? `<h2>Arm & static surfaces</h2>${metric('Clear paths',String(collision.counts?.CERTIFIED_MODELED_PATH ?? 0))}${metric('Collision paths',String(collision.counts?.COLLISION ?? 0))}${metric('Unresolved paths',String(collision.counts?.CLEARANCE_UNRESOLVED ?? 0))}<div class="metric"><span>TCP path bound</span><strong id="tracking-bound">—</strong></div><p id="motion-collision-status" class="description"></p><button id="collision-witness" class="hidden">Show collision evidence</button><p class="description">Checks filled convex hulls of the URDF arm, table and raised platform with ${(collision.config.clearance_m*1000).toFixed(1)} mm clearance and interval motion bounds. ${collision.tool_model?.configured ? 'Configured pen/mount bodies are included.' : 'Pen and mount bodies are not included.'} The contact tip, growing material and travel are not included. The TCP bound applies between waypoints against the linear reference path. Work-cell dimensions are provisional.</p><a class="download" href="${url('drawing_collisions.json')}" target="_blank">Open collision audit ↗</a>` : '<h2>Arm & static surfaces</h2><p class="description">No collision audit matches this motion snapshot yet. Refresh after the audit completes.</p>');
        $('#metrics').insertAdjacentHTML('beforeend',toolSummary(collision?.tool_model));
        let strokeIndex = 0;
        const drawSample = (index:number) => {
          const path = paths[strokeIndex].positions!;
          index = Math.max(0,Math.min(path.length-1,index));
          const slider = $<HTMLInputElement>('#motion-sample'); slider.max=String(path.length-1);slider.value=String(index);
          $('#motion-count').textContent=`${index+1} / ${path.length}`;
          const actual = viewer.setRobotPose(path[index],robot,motion.flange_to_tcp);
          const target = paths[strokeIndex].tcp_targets_m[index];
          const error = Math.hypot(...actual.map((value,i)=>value-target[i]));
          $('#motion-error').textContent=(error*1000).toFixed(3)+' mm';
          viewer.selectById(paths[strokeIndex].source_stroke);
          viewer.highlightRobotLinks([],robot);
          const row = collisionRows.get(paths[strokeIndex].id);
          if (collision) {
            $('#motion-collision-status').textContent = row?.status === 'CERTIFIED_MODELED_PATH' ? 'Selected stroke: clear within the stated rigid arm model.' : row ? `Selected stroke: ${row.status.toLowerCase().replaceAll('_',' ')}.` : 'Selected stroke: not audited yet.';
            $('#collision-witness').classList.toggle('hidden',!row?.witnesses.length);
            $('#tracking-bound').textContent = row?.tracking_status === 'BOUNDED' ? `${((row.position_error_bound_m ?? 0)*1000).toFixed(3)} mm / ${((row.orientation_error_bound_rad ?? 0)*180/Math.PI).toFixed(3)}°` : 'Not established';
          }
          if (error > .0005) throw new Error('Displayed robot TCP differs from the planned sample.');
        };
        const stopMotion = () => {if(playback) clearInterval(playback);playback=null;$('#motion-play').textContent='Play stroke';};
        $('#motion-stroke').addEventListener('change',()=>{stopMotion();strokeIndex=Number($<HTMLSelectElement>('#motion-stroke').value);drawSample(0);});
        $('#motion-sample').addEventListener('input',()=>{stopMotion();drawSample(Number($<HTMLInputElement>('#motion-sample').value));});
        $('#motion-refresh').addEventListener('click',()=>{const id=run.id;json<Run>(`/api/runs/${encodeURIComponent(id)}`).then(updated=>{if(run.id!==id)return;run=updated;return selectStage('motion');}).catch(showError);});
        if (collision) $('#collision-witness').addEventListener('click',()=>{
          stopMotion();
          const row = collisionRows.get(paths[strokeIndex].id);
          const witness = row?.witnesses[0];
          if (!witness) return;
          viewer.setRobotPose(witness.positions,robot,motion.flange_to_tcp);
          viewer.highlightRobotLinks(witness.pairs.flatMap(pair=>[pair.first,pair.second]).map(name=>name.split(':')[0]),robot);
          $('#motion-count').textContent=`Edge ${witness.edge_index+1} · ${(witness.alpha*100).toFixed(1)}%`;
          $('#motion-error').textContent='Evidence pose';
          $('#motion-collision-status').textContent=witness.pairs.map(pair=>`${pair.first} / ${pair.second}`).join('; ');
        });
        $('#motion-play').addEventListener('click',()=>{
          if (playback) {stopMotion();return;}
          if(Number($<HTMLInputElement>('#motion-sample').value)>=paths[strokeIndex].positions!.length-1)drawSample(0);
          $('#motion-play').textContent='Pause';
          playback=setInterval(()=>{const next=Number($<HTMLInputElement>('#motion-sample').value)+1;drawSample(next);if(next>=paths[strokeIndex].positions!.length-1)stopMotion();},60);
        });
        drawSample(0);
        $('#legend').innerHTML='<span><i></i>Selected drawing stroke</span><span>Blue · assumed pen axis</span>';
        $('#note-text').textContent=collision ? 'Joint paths include a separate arm/static-surface collision audit. Drawing contact, deposited material, travel, timing and process feasibility remain unvalidated.' : 'These joint paths pass sampled TCP tracking checks. The preview does not yet include collisions, pen-off travel, extrusion timing, or process feasibility.';
        $('#output-summary').textContent=`${paths.length} individual stroke paths in the robot base frame.`;
        $('#next-summary').textContent='Check tool and material collisions, then plan travel between strokes.';
      }
    }
    return;
  }
  empty('This stage has not been generated', run.stages[id]?.message ?? 'Its outputs will appear here as the pipeline advances. Earlier stages remain available for inspection.');
  $('#metrics').innerHTML = `<h2>${escape(stage.name)}</h2>${metric('Status',statusText(status))}<p class="description">No result is presented as validated until this stage has been executed and checked.</p>`;
  $('#note-text').textContent = 'The current run contains model preparation and geometric candidates. The remaining stages are tracked separately; no robot program or fabrication success is implied.';
  $('#input-summary').textContent = stages[index - 1]?.name ?? 'Prepared model';
  $('#output-summary').textContent = 'Awaiting stage execution.';
  $('#next-summary').textContent = stages[index + 1]?.name ?? 'Physical fabrication and inspection';
}

function renderGeometricPartition(data:GeometricPartition,cell:CellManifest,arms:CellArmView[],allocation:CellAllocation|null) {
  const colors:Record<string,number>={arm_1:0x327aa0,arm_2:0xc0813f,seam:0x8e648d};
  const point=(p:number[])=>cell.world_from_build.slice(0,3).map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]+row[3]);
  for(const arm of arms)viewer.setCellArmPose(arm.id,allocation?.initial_positions?.[arm.id]??arm.initial_positions);
  viewer.setCurves(data.curves.map(c=>({...c,color:colors[c.work_zone]??0x777777,segments:c.segments.map(s=>s.map(point))})));
  viewer.cell.visible=false;viewer.platform.visible=false;
  viewer.focus('curves');
  $('#title').textContent='Assign whole structural units.';
  $('#subtitle').textContent='Keep contours and connected cycle groups intact; divide work around the structure instead of cutting it in half.';
  $('#badge').textContent='Structural grouping · feasibility unverified';
  $('#metrics').innerHTML=`<h2>Structure-preserving regions</h2>${metric('New contour cuts',String(data.join_nodes.length))}${metric('Protected units',String(data.structural_units.length))}${metric('Cycle blocks kept intact',String(data.cycle_blocks_preserved))}${metric('Total material',`${data.partitioned_length_m.toFixed(2)} m`)}${data.zones.map(z=>`<h3>${escape(z.id==='seam'?'Shared structural work':z.id.replace('_',' '))}</h3>${metric('Drawing length',`${z.length_m.toFixed(2)} m`)}${metric('Whole strokes',String(z.stroke_count))}`).join('')}${metric('Unsplit construction order',['GEOMETRY_PREVIEW','GROUNDED_INTACT_CONTOUR_ORDER'].includes(data.construction_status)?`${data.construction_actions} actions`:'Not found within budget')}<p class="description">Purple units cross the boundary or couple several contours. They keep one arm owner and reserve both arm resources. Grouping preserves geometry; it does not prove that the structure can stand.</p><label class="control">Inspect protected unit<select id="partition-unit" aria-label="Protected structural unit"><option value="">All units</option>${data.structural_units.map(u=>`<option value="${escape(u.id)}">${escape(u.id)} · ${u.work_zone==='seam'?'shared':escape(u.work_zone)} · ${(u.length_m*1000).toFixed(0)} mm</option>`).join('')}</select></label><button class="mode-button" id="partition-view">Show full robot cell</button><a class="download" href="${url('geometric_partition.json')}" target="_blank">Open partition artifact ↗</a>`;
  $('#partition-unit').addEventListener('change',()=>{const id=$<HTMLSelectElement>('#partition-unit').value;viewer.highlight(id?data.curves.filter(c=>c.allocation_group===id).map(c=>c.id):[]);});
  $('#partition-view').addEventListener('click',()=>{const full=$('#partition-view').textContent==='Show full robot cell';viewer.cell.visible=full;viewer.platform.visible=full;viewer.focus(full?'cell':'curves');$('#partition-view').textContent=full?'Focus sculpture':'Show full robot cell';});
  $('#note-text').textContent='There are no new cuts. Whole contours, cycle blocks and potential level rims stay together. The colored regions propose ownership only: a usable plan still needs supported intermediate shapes, sound joins and full-arm clearance.';
  $('#input-summary').textContent='Connected material graph, complete source contours and the two robot base positions.';
  $('#output-summary').textContent='Indivisible structural groups, arm-side ownership and shared-work reservations.';
  $('#next-summary').textContent='Establish a supported construction order, then schedule intact groups and validate both arms.';
  $('#legend').innerHTML='<span><i style="background:#327aa0"></i>Arm 1 region</span><span><i style="background:#c0813f"></i>Arm 2 region</span><span><i style="background:#8e648d"></i>Shared structural work</span>';
}

function renderMountAssessment(data:MountAssessment,arms:CellArmView[],cell:CellManifest,partition:GeometricPartition|null) {
  if(partition){
    const point=(p:number[])=>cell.world_from_build.slice(0,3).map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]+row[3]);
    viewer.setCurves(partition.curves.map(c=>({...c,color:0x8a765c,segments:c.segments.map(s=>s.map(point))})));
  }
  $('#legend').innerHTML='<span><i style="background:#327aa0"></i>Arm 1 tip</span><span><i style="background:#c0813f"></i>Arm 2 tip</span><span>Wireframe · target reference, not deposited material</span>';
  $('#title').textContent='Compare the robot base mounts.';
  $('#subtitle').textContent='Same Palace, pen-tip targets and right-angle tool geometry. These are static pose checks, not a coordinated motion plan.';
  $('#badge').textContent=data.status==='TIME_BUDGET'?'Partial mount assessment':'Sampled mount assessment';
  $('#metrics').innerHTML=`<h2>Base orientation</h2><div class="motion-controls"><label>Mount option<select id="mount-option" aria-label="Mount option">${data.candidates.map(c=>`<option value="${escape(c.id)}">${escape(c.label)}</option>`).join('')}</select></label><label>Pose pair<select id="mount-pair" aria-label="Mount pose pair"></select></label></div><div id="mount-result"></div><div class="table-wrap"><table class="mount-table"><thead><tr><th>Mount</th><th>Clear poses</th><th>Clear pairs</th><th>Joint margin</th></tr></thead><tbody>${data.candidates.map(c=>`<tr><td>${escape(c.label)}</td><td>${c.clear_pose_count}/${c.requested_pose_count}</td><td>${c.clear_pair_count}/${c.pose_pairs.length}</td><td>${c.minimum_best_joint_margin>0&&c.minimum_best_joint_margin<0.001?'<0.1':(100*c.minimum_best_joint_margin).toFixed(1)}%</td></tr>`).join('')}</tbody></table></div><p class="description">Joint margin is the minimum distance to a joint limit, divided by that joint's range, for the best retained configuration at each clear sample. Coverage uses a fixed set of points on the strokes. A clear pose pair is not proof of a path between poses.</p><p class="description">${data.limitations.map(escape).join('. ')}.</p><a class="download" href="${url('cell_mounting_assessment.json')}" target="_blank">Open mount assessment ↗</a>`;
  $('#input-summary').textContent='Three fixed base orientations, twelve sampled stroke points and the same provisional right-angle tool.';
  $('#output-summary').textContent='Per-arm static clearance, joint-limit margins and six paired-pose checks.';
  $('#next-summary').textContent='Use the viable mounting geometry to generate complete collision-checked trajectories.';
  const penAngle=data.drawing_pen_tilt_deg??0;
  $('#note-text').textContent=(penAngle?`The nozzle keeps its ${penAngle}° world-frame lean across base options. `:'The nozzle stays vertical in the world when the base tilts. ')+'Mount wedges, cable routing, payload and gravity setup still need physical design.';
  const draw=()=>{
    const candidate=data.candidates.find(c=>c.id===$<HTMLSelectElement>('#mount-option').value)!;
    const pair=candidate.pose_pairs[Number($<HTMLSelectElement>('#mount-pair').value)];
    for(const arm of arms){
      viewer.setCellArmBase(arm.id,candidate.world_from_base[arm.id]);
      const fallback=candidate.arms[arm.id].find(row=>row.clear_solutions.length)?.clear_solutions[0].positions??arm.initial_positions;
      viewer.setCellArmPose(arm.id,pair?.positions?.[arm.id]??fallback);
    }
    $('#mount-result').innerHTML=metric('Displayed pair',pair?.positions?'Both arms clear at this pose':'No clear paired solution; separate arm poses shown')+metric('Checked arm poses',`${candidate.pose_count}/${candidate.requested_pose_count}`);
    viewer.focus('cell');
  };
  const select=()=>{
    const candidate=data.candidates.find(c=>c.id===$<HTMLSelectElement>('#mount-option').value)!;
    $('#mount-pair').innerHTML=candidate.pose_pairs.map((p,i)=>`<option value="${i}">Pair ${i+1} · ${p.positions?'clear':'not established'}</option>`).join('');
    draw();
  };
  $('#mount-option').addEventListener('change',select);
  $('#mount-pair').addEventListener('change',draw);
  select();
}

function renderPenOrientationProbe(data:PenOrientationProbe, arms:CellArmView[]) {
  const available=data.arms.length===arms.length&&data.arms.every(arm=>arm.ik_found&&arm.positions);
  $('#title').textContent='Use pen orientation to separate the wrists.';
  $('#subtitle').textContent='One recorded pose comparison with the same pen-tip positions. This does not change the active trajectory.';
  $('#badge').textContent='One-pose diagnostic · full paths unverified';
  $('#metrics').innerHTML=`<h2>Pen-angle comparison</h2><label class="control">Recorded pose<select id="pen-angle-pose" aria-label="Pen-angle pose"><option value="original">Original pen orientation</option><option value="tilted" ${available?'selected':'disabled'}>${data.tilt_deg.toFixed(0)}° inward tilt</option></select></label><div id="pen-angle-result"></div>${metric('Original flange spacing',data.flange_separation_before_m===undefined?'—':(data.flange_separation_before_m*100).toFixed(1)+' cm')}${metric('Tilted flange spacing',data.flange_separation_after_m===undefined?'—':(data.flange_separation_after_m*100).toFixed(1)+' cm')}<p class="description">One fixed tilt and one IK seed per arm. A clear pose does not prove that an entire stroke, approach, or build can use this orientation.</p><a class="download" href="${url('pen_orientation_probe.json')}" target="_blank">Open pose diagnostic ↗</a>`;
  $('#input-summary').textContent='A recorded inter-arm conflict, the same two pen-tip positions and the saved robot geometry.';
  $('#output-summary').textContent='A static comparison of the original pose and one fixed inward pen tilt.';
  $('#next-summary').textContent='Build and validate a separate fixed motion catalogue before using the tilted policy.';
  $('#note-text').textContent=data.scope;
  $('#legend').innerHTML='<span><i style="background:#327aa0"></i>Arm 1 tip</span><span><i style="background:#c0813f"></i>Arm 2 tip</span><span>Red · recorded collision links</span>';
  const draw=()=>{
    const tilted=$<HTMLSelectElement>('#pen-angle-pose').value==='tilted';
    for(const [i,arm] of arms.entries()){
      const q=tilted?data.arms.find(row=>row.arm===arm.id)!.positions!:(data.original_witness.positions as number[][])[i];
      viewer.setCellArmPose(arm.id,q);
      viewer.highlightCellLinks(arm.id,tilted?[]:(data.original_witness.pairs??[]).map(pair=>(i===0?pair.first:pair.second).split(':')[0]));
    }
    viewer.focus('cell');
    const result=tilted?data.tilted_pair:data.original_pair;
    const contacts=tilted?data.arms.reduce((n,row)=>n+(row.static_contacts?.length??0),0):null;
    const movement=Math.max(...data.arms.map(row=>(row.tcp_displacement_m??0)*1000));
    $('#pen-angle-result').innerHTML=metric('Inter-arm pose check',result?.status==='CERTIFIED_INTER_ARM_INTERVAL'?'Clear at this pose':result?.status==='COLLISION'?'Collision':escape(result?.status??'Unavailable'))+(tilted?metric('Static contacts',String(contacts))+metric('Maximum tip movement',movement<.001?'< 0.001 mm':movement.toFixed(3)+' mm'):'')+metric('Full trajectory validation','Not performed for this policy');
  };
  $('#pen-angle-pose').addEventListener('change',draw);draw();
}

function renderCellAllocation(data:CellAllocation, construction:Construction, cell:CellManifest, arms:CellArmView[]) {
  const minutes=(seconds:number)=>`${(seconds/60).toFixed(1)} min`;
  $('#title').textContent='Assign the strokes. Minimize finish time.';
  $('#subtitle').textContent=arms.length===1?'Schedule all strokes for one arm using the fixed paths and construction dependencies.':'Offline allocation across two arms, using a fixed motion catalogue and construction dependencies.';
  $('#badge').textContent='Allocation only · not replay-certified';
  $('#metrics').innerHTML=`<h2>Offline resource allocation</h2>${metric('Solver','CP-SAT')}${metric('Fixed motion options',String(data.fixed_mode_count))}<label class="control">Arm resources<select id="allocation-resources" class="method-select" aria-label="Allocation arm count">${arms.length===2?'<option value="dual">Two arms</option>':''}<option value="single">${arms.length===1?'One arm':'One arm baseline'}</option></select></label><div id="allocation-detail"></div><p class="description">Drawing clocks use the stored joint paths. Travel durations are estimates. This schedule has not passed inter-arm, material or structural replay. ${construction.actions.some(a=>'work_zone' in a)?'Structural region constraints are imposed on ownership.':'Whole contours are assigned independently; the structural-region preview is not imposed on this allocation.'}</p>${data.motion_preparation?metric('Prepared selected motions',`${data.motion_preparation.prepared_strokes} / ${data.motion_preparation.requested_strokes}`)+`<p class="description">${escape(data.motion_preparation.status.replaceAll('_',' ').toLowerCase())}</p>`:''}<button id="allocation-view" class="mode-button">Show full robot cell</button><a class="download" href="${url('cell_allocation.json')}" target="_blank">Open allocation result ↗</a>`;
  $('#note-text').textContent=(data.support_first?'All added starting supports finish before the main contours. ':'')+'Colors show stroke ownership. The arms stay at their setup poses: allocation does not yet define validated robot movement.';
  $('#input-summary').textContent=`${data.requested_strokes} strokes; ${data.fixed_mode_count} existing motion options; anchor and cooling constraints.`;
  $('#output-summary').textContent='Arm assignments, sequence and estimated start/end times.';
  $('#next-summary').textContent='Construct selected travel paths and replay the shared schedule; refine only conflicts found there.';
  $('#legend').innerHTML=arms.map((arm,i)=>`<span><i style="background:${i===0?'#327aa0':'#c0813f'}"></i>Arm ${i+1} strokes</span>`).join('');
  const panel=$('#timing-panel');panel.classList.remove('hidden');
  const point=(p:number[])=>cell.world_from_build.slice(0,3).map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]+row[3]);
  const draw=()=>{
    const result=data.results[$<HTMLSelectElement>('#allocation-resources').value as 'dual'|'single'];
    $('#note-text').textContent=(data.support_first?'All added starting supports finish before the main contours. ':'')+'Colors show stroke ownership. The arms stay at their setup poses: allocation does not yet define validated robot movement.';
    for(const arm of arms){viewer.setCellArmPose(arm.id,data.initial_positions?.[arm.id]??arm.initial_positions);viewer.highlightCellLinks(arm.id,[]);}
    const assigned=new Map(result.actions.map(a=>[a.stroke_id,a]));
    viewer.setCurves(construction.actions.map(curve=>({...curve,
      segments:curve.segments.map(segment=>segment.map(point)),
      color:assigned.get(curve.id)?.arm==='arm_2'?0xc0813f:0x327aa0,
    })));
    viewer.cell.visible=false;
    viewer.focus('curves');
    $('#allocation-view').textContent='Show full robot cell';
    $('#allocation-detail').innerHTML=metric('Assigned strokes',`${result.actions.length} / ${data.requested_strokes}`)+metric('Solver status',result.status)+metric('Estimated finish',result.makespan_s===null?'No schedule':minutes(result.makespan_s))+metric('Model lower bound',result.objective_bound_s===null?'—':minutes(result.objective_bound_s))+metric('Solver wall time',`${result.wall_time_s.toFixed(1)} s`)+(arms.length===2?metric('Estimated one-/two-arm ratio',data.estimated_speedup===null?'—':`${data.estimated_speedup.toFixed(2)}×`):'');
    const assumptions=data.process_assumptions;
    if(assumptions&&result.actions.length){
      const selectedIds=new Set(result.actions.map(a=>a.stroke_id));
      const length=construction.actions.filter(a=>selectedIds.has(a.id)).reduce((sum,a)=>sum+a.length_m,0);
      const rawDrawing=length/assumptions.nominal_speed_m_s;
      const clockDrawing=result.actions.reduce((sum,a)=>sum+a.drawing_s,0);
      $('#allocation-detail').insertAdjacentHTML('beforeend',`<h3>Where the time goes</h3>${metric('Selected drawing length',`${length.toFixed(2)} m`)}${metric('Assumed tip-speed limit',`${(assumptions.nominal_speed_m_s*1000).toFixed(1)} mm/s`)}${metric('Uninterrupted drawing work',minutes(rawDrawing))}${metric('Current drawing-clock work',minutes(clockDrawing))}${metric('Assumed support cooling',`${assumptions.cooling_s.toFixed(1)} s per dependency`)}<p class="description">Work is summed across pens, not elapsed job time. ${assumptions.drawing_clock==='cubic_toppra'?'The drawing clock moves continuously through numerical samples, using shape-preserving cubic paths.':'The current clock stops at every sampled point along a curve.'} Cooling is a dependency delay and can overlap other work; its contribution to finish time has not been isolated.</p>`);
    }
    if(result===data.results[data.motion_preparation?.resource_mode??'dual']&&data.motion_preparation?.failures.length){
      const rejected=data.motion_preparation.failures[0];
      $('#allocation-detail').insertAdjacentHTML('beforeend',`<h3>Selected motion needs repair</h3><p>${escape(rejected.mode_id??'Selected motion')}</p>${(rejected.evidence??[]).map(item=>`<p>${escape(item.phase)} · ${escape(item.status.replaceAll('_',' ').toLowerCase())}</p>`).join('')}<p class="description">${escape(rejected.scope??'This diagnostic concerns the selected motion and material prefix.')}</p>`);
      const witness=rejected.evidence?.find(item=>item.positions&&item.pairs?.length);
      if(witness?.positions){
        $('#allocation-detail').insertAdjacentHTML('beforeend','<button class="mode-button" id="selected-motion-conflict">Inspect recorded collision pose</button>');
        $('#selected-motion-conflict').addEventListener('click',()=>{
          viewer.cell.visible=true;
          viewer.setCellArmPose(witness.arm,witness.positions!);
          viewer.focus('curves');
          $('#note-text').textContent=`${witness.arm} · ${witness.phase}: ${witness.status.replaceAll('_',' ').toLowerCase()}. ${witness.pairs?.map(pair=>pair.first+' / '+pair.second).join(', ')}. The other arm remains at its staging pose; this is an individual-motion diagnostic.`;
        });
      }
    }
    if(result===data.results[data.motion_preparation?.resource_mode??'dual']&&data.motion_preparation?.retimed_schedule){
      const prepared=data.motion_preparation;
      $('#allocation-detail').insertAdjacentHTML('beforeend',metric('Constructed-path finish',minutes(prepared.retimed_schedule!.makespan_s))+metric('Shared-clock replay',replayProgressLabel(prepared)));
      const failure=prepared.replay?.failures?.[0];
      if(failure?.positions&&Array.isArray(failure.positions[0])){
        $('#allocation-detail').insertAdjacentHTML('beforeend','<button class="mode-button" id="allocation-conflict">Inspect clearance conflict</button>');
        $('#allocation-conflict').addEventListener('click',()=>{
          viewer.cell.visible=true;
          arms.forEach((arm,i)=>viewer.setCellArmPose(arm.id,(failure.positions as number[][])[i]));
          viewer.focus('curves');
          $('#note-text').textContent=`Replay clearance conflict in interval starting ${(failure.time_s??0).toFixed(1)} s: ${failure.pairs?.map(p=>p.first+' / '+p.second).join(', ')}. These are the recorded joint poses.`;
        });
      }
    }
    const total=result.makespan_s??1;
    const colors:Record<string,string>={arm_1:'#327aa0',arm_2:'#c0813f'};
    panel.innerHTML=`<h2>Allocation timeline · estimated ${minutes(total)}</h2><p>Each block occupies one arm, including estimated incoming travel, pen events and drawing. Final return and cooling are included in the estimated finish.</p>${result.enabled_arms.map(arm=>`<div style="display:grid;grid-template-columns:65px 1fr;gap:10px;margin:18px 0"><strong>${escape(arm)}</strong><div style="position:relative;height:30px;background:#eeece6;border-radius:5px">${result.actions.filter(a=>a.arm===arm).map(a=>`<button class="allocation-block" data-stroke="${escape(a.stroke_id)}" title="${escape(a.stroke_id)}: ${a.start_s.toFixed(1)}–${a.end_s.toFixed(1)} s" style="position:absolute;top:0;bottom:0;left:${100*a.start_s/total}%;width:${100*(a.end_s-a.start_s)/total}%;background:${colors[arm]};border:1px solid white;min-width:2px;cursor:pointer"></button>`).join('')}</div></div>`).join('')}<div class="table-wrap"><table><thead><tr><th>Stroke</th><th>Arm</th><th>Start</th><th>End</th><th>Support</th></tr></thead><tbody>${result.actions.map(a=>`<tr><td><button class="allocation-block" data-stroke="${escape(a.stroke_id)}">${escape(a.stroke_id)}</button></td><td>${escape(a.arm)}</td><td>${a.start_s.toFixed(1)} s</td><td>${a.end_s.toFixed(1)} s</td><td>${escape(a.support_modes.map(s=>s.split('/')[1]).join(', ')||'Ground')}</td></tr>`).join('')}</tbody></table></div><p id="allocation-selected">Select a timeline block to highlight its stroke.</p>`;
    if(assumptions&&result.actions.length===data.requested_strokes){
      panel.insertAdjacentHTML('beforeend',`<h3>Continuous-drawing speed sensitivity</h3><p>Arithmetic lower bounds for this ${construction.total_length_m.toFixed(2)} m shape. These are uncalibrated scenarios, not measured human speeds or newly planned robot schedules. They exclude travel, joins, cooling, acceleration and collisions.</p><div class="table-wrap"><table><thead><tr><th>Assumed tip speed</th><th>One pen: drawing only</th><th>Two pens: ideal equal split</th></tr></thead><tbody>${[3,5,10].map(speed=>{const seconds=construction.total_length_m/(speed/1000);return `<tr><td>${speed} mm/s</td><td>${minutes(seconds)}</td><td>${minutes(seconds/2)}</td></tr>`;}).join('')}</tbody></table></div><p>Our support-cooling delay is an unmeasured development assumption. <a href="https://scrib3dpen.com/a/docs" target="_blank" rel="noopener noreferrer">SCRIB3D guidance</a> describes about 3 seconds for ABS and 4–6 seconds for PLA to harden; this does not establish load-bearing strength for a particular stroke.</p>`);
    }
    panel.querySelectorAll<HTMLButtonElement>('.allocation-block').forEach(button=>button.addEventListener('click',()=>{
      const action=result.actions.find(a=>a.stroke_id===button.dataset.stroke)!;
      viewer.selectById(action.stroke_id);
      $('#allocation-selected').textContent=`${action.stroke_id} · ${action.arm} · ${(action.end_s-action.start_s).toFixed(1)} s · anchor ${action.start_anchor}`;
    }));
    const repairs=result===data.results[data.motion_preparation?.resource_mode??'dual']?data.motion_preparation?.refinements:undefined;
    if(repairs?.length){
      const mode=(value:string)=>value.split('/').slice(0,2).join(' · ');
      panel.insertAdjacentHTML('afterbegin',`<section class="planner-checkpoint" aria-label="Scheduling repairs"><h2>Scheduling repairs (${repairs.length})</h2><p>Recorded conflicts from earlier schedule attempts. These explain the current constraints; they are not failures of the latest schedule.</p><ol>${repairs.map((repair,i)=>{
        const description=repair.kind==='NO_OVERLAP'?`Separate ${(repair.modes??[]).map(mode).join(' and ')} in time.`:repair.kind==='PARK_IDLE_ARM'?`Park ${repair.arm} after ${mode(repair.after_mode??'')} before its next task (${(repair.park_duration_s??0).toFixed(1)} s of added motion).`:repair.kind==='MATERIAL_ORDER'?`Order ${(repair.pairs??[]).map(pair=>pair.map(mode).join(' before ')).join('; ')} to avoid already deposited material.`:repair.kind.replaceAll('_',' ').toLowerCase();
        return `<li><p>${escape(description)}</p>${repair.witness?.positions?`<button class="mode-button repair-pose" data-repair="${i}">Inspect repair ${i+1}</button>`:''}</li>`;
      }).join('')}</ol><p id="repair-pose-label">Select a repair to inspect its recorded robot poses.</p></section>`);
      panel.querySelectorAll<HTMLButtonElement>('.repair-pose').forEach(button=>button.addEventListener('click',()=>{
        const repair=repairs[Number(button.dataset.repair)], witness=repair.witness!;
        viewer.cell.visible=true;
        for(const arm of arms)viewer.highlightCellLinks(arm.id,[]);
        if(Array.isArray(witness.positions?.[0]))arms.forEach((arm,i)=>{
          viewer.setCellArmPose(arm.id,(witness.positions as number[][])[i]);
          if(witness.phase==='INTER_ARM')viewer.highlightCellLinks(arm.id,(witness.pairs??[]).map(pair=>(i===0?pair.first:pair.second).split(':')[0]));
        });
        else if(witness.arm&&witness.positions)viewer.setCellArmPose(witness.arm,witness.positions as number[]);
        viewer.focus('cell');$('#allocation-view').textContent='Focus sculpture';
        const caption=`Earlier ${witness.phase??witness.arm??'replay'} conflict: ${witness.status}, interval starting ${(witness.time_s??0).toFixed(2)} s. Recorded poses may be inside that interval. The full wireframe is shown as a reference, not the material present at that time.`;
        $('#repair-pose-label').textContent=caption;$('#note-text').textContent=caption;
      }));
    }
  };
  $('#allocation-view').addEventListener('click',()=>{
    const full=$('#allocation-view').textContent==='Show full robot cell';
    viewer.cell.visible=full;
    viewer.focus(full?'cell':'curves');$('#allocation-view').textContent=full?'Focus sculpture':'Show full robot cell';
  });
  $<HTMLSelectElement>('#allocation-resources').value=data.motion_preparation?.resource_mode??run.resource_mode??'dual';
  $('#allocation-resources').addEventListener('change',draw);draw();
}

function renderMaterialOrders(problem:MaterialOrderProblem,allocation:CellAllocation,construction:Construction,cell:CellManifest) {
  const modes=new Map(problem.modes.map(mode=>[mode.id,mode]));
  const groups=new Map<string,{first:{id:string;arm:string;task:string};after:string;blockers:string[]}>();
  for(const [firstId,afterId] of problem.precedence_modes??[]){
    const first=modes.get(firstId),after=modes.get(afterId);
    if(!first||!after)continue;
    const key=JSON.stringify([firstId,after.task]);
    const group=groups.get(key)??{first,after:after.task,blockers:[]};
    group.blockers.push(afterId);groups.set(key,group);
  }
  const orders=[...groups.values()];
  $('#title').textContent='Keep access open while building.';
  $('#subtitle').textContent='Recorded material obstructions become conditional ordering constraints for the offline solver.';
  $('#badge').textContent='Ordering repair · full replay still required';
  $('#metrics').innerHTML=`<h2>Material ordering constraints</h2>${metric('Mode constraints',String(problem.precedence_modes?.length??0))}${metric('Grouped stroke orders',String(orders.length))}<label class="control">Recorded order<select id="material-order" class="method-select" aria-label="Material ordering constraint">${orders.map((order,i)=>`<option value="${i}">${escape(order.first.arm)}: ${escape(order.first.task)} before ${escape(order.after)}</option>`).join('')}</select></label><div id="material-order-detail"></div><p class="description">Blue is the stroke that needs access. Bronze is material that must be printed later for this motion option. The remaining wireframe stays visible for context. No curves are removed or split.</p><button id="material-order-cell" class="mode-button">Show full robot cell</button><a class="download" href="${url('allocation_problem.json')}" target="_blank">Open solver constraints ↗</a>`;
  $('#note-text').textContent='These are conservative repairs derived from recorded collisions or unresolved clearance. They preserve the original wireframe and do not certify a complete build.';
  $('#input-summary').textContent='A fixed motion catalogue and witnessed collisions with previously deposited strokes.';
  $('#output-summary').textContent='Conditional finish-before-start constraints, retained during final retiming.';
  $('#next-summary').textContent='Rebuild the selected motions and validate both arms against the evolving sculpture.';
  $('#legend').innerHTML='<span><i style="background:#327aa0"></i>Draw first</span><span><i style="background:#c0813f"></i>Draw later</span><span><i style="background:#c5c8ca"></i>Wireframe context</span>';
  viewer.cell.visible=false;
  const point=(p:number[])=>cell.world_from_build.slice(0,3).map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]+row[3]);
  const draw=()=>{
    const order=orders[Number($<HTMLSelectElement>('#material-order').value)];
    if(!order)return;
    viewer.setCurves(construction.actions.map(curve=>({...curve,color:curve.id===order.first.task?0x327aa0:curve.id===order.after?0xc0813f:0xc5c8ca,segments:curve.segments.map(segment=>segment.map(point))})));
    const result=allocation.results[allocation.motion_preparation?.resource_mode??run.resource_mode??'dual'];
    const first=result.actions.find(action=>action.mode_id===order.first.id);
    const after=result.actions.find(action=>order.blockers.includes(action.mode_id));
    const status=first&&after?(first.end_s<=after.start_s+1e-6?'Enforced in current allocation':'Current allocation needs regeneration'):'Inactive for current motion choices';
    $('#material-order-detail').innerHTML=metric('Draw first',order.first.task)+metric('Moving arm',order.first.arm)+metric('Draw later',order.after)+metric('Current allocation',status)+`<p class="description">Applies when ${escape(order.first.arm)} uses the recorded first-stroke motion and a listed blocker mode is selected. ${new Set(order.blockers.map(id=>modes.get(id)?.arm)).size} blocker-arm options are covered.</p>`;
  };
  $('#material-order').addEventListener('change',draw);
  $('#material-order-cell').addEventListener('click',()=>{
    viewer.cell.visible=!viewer.cell.visible;
    viewer.focus(viewer.cell.visible?'cell':'curves');
    $('#material-order-cell').textContent=viewer.cell.visible?'Focus sculpture':'Show full robot cell';
  });
  draw();viewer.focus('curves');
}

function renderScaleStudy(construction:Construction) {
  const bounds=manifest.bounds_m;
  const dimensions=bounds[1].map((v,i)=>v-bounds[0][i]);
  const sourceExtent=Math.max(...dimensions);
  const center=dimensions.map((v,i)=>bounds[0][i]+v/2);
  center[2]=bounds[0][2];
  const initial=Math.min(10,sourceExtent*100);
  viewer.cell.visible=false;viewer.platform.visible=false;
  viewer.grid.position.set(0,0,-.0003);viewer.grid.scale.setScalar(1);
  viewer.camera.zoom=2.3;
  $('#title').textContent='Keep the sculpture. Change its size.';
  $('#subtitle').textContent='The same whole contours and supports, uniformly scaled. Strand diameter stays unchanged.';
  $('#badge').textContent='Geometry study · needs new robot planning';
  $('#frame-label').textContent='Side-by-side size comparison · meters';
  $('#metrics').innerHTML=`<h2>Uniform scale preview</h2><label class="control">Maximum dimension <span id="study-size"></span><input id="study-scale" aria-label="Preview maximum dimension" type="range" min="5" max="${Math.max(10,sourceExtent*100)}" step="0.5" value="${initial}"></label><div id="study-detail"></div><p class="description">The blue reference keeps its original size. Bronze shows the selected size at the same camera scale. No curves are removed or split. The strand thickness is held fixed, so small openings become tighter.</p><p class="description">This view does not resize an existing robot program. Reach, tool clearance, joins and stability must be checked again for the chosen size.</p>`;
  $('#note-text').textContent='A smaller model reduces stroke length and unsupported spans. It also reduces the room available for the nozzle and for separate recognizable details.';
  $('#input-summary').textContent=`The current ${construction.actions.length}-action supported sculpture.`;
  $('#output-summary').textContent='Uniformly scaled geometry and drawing-only time scenarios.';
  $('#next-summary').textContent='Regenerate and validate robot paths at the selected size, then calibrate the pen.';
  $('#legend').innerHTML='<span><i style="background:#327aa0"></i>Original size</span><span><i style="background:#c0813f"></i>Selected size</span>';
  const panel=$('#timing-panel');panel.classList.remove('hidden');
  const draw=()=>{
    const extent=Number($<HTMLInputElement>('#study-scale').value)/100;
    const scale=extent/sourceExtent;
    const curves=[{prefix:'original',scale:1,shift:-sourceExtent*.65,color:0x327aa0},{prefix:'scaled',scale,shift:sourceExtent*.65,color:0xc0813f}].flatMap(copy=>construction.actions.map(curve=>({...curve,id:`${copy.prefix}/${curve.id}`,color:copy.color,length_m:curve.length_m*copy.scale,segments:curve.segments.map(segment=>segment.map(p=>p.map((v,i)=>(v-center[i])*copy.scale+(i===0?copy.shift:0))))})));
    viewer.setCurves(curves);
    const length=construction.total_length_m*scale;
    const dims=dimensions.map(v=>(v*scale*100).toFixed(1)).join(' × ');
    $('#study-size').textContent=`${(extent*100).toFixed(1)} cm`;
    $('#scale-label').textContent=`Preview: ${dims} cm`;
    $('#study-detail').innerHTML=metric('Preview dimensions',`${dims} cm`)+metric('Linear scale',`${(scale*100).toFixed(1)}%`)+metric('Drawing length',`${length.toFixed(2)} m`)+metric('Whole actions retained',String(construction.actions.length));
    panel.innerHTML=`<h2>Drawing-only time at ${dims} cm</h2><p>These are arithmetic lower bounds for ${length.toFixed(2)} m of strokes, not motion-planning results or measured human speeds. They exclude travel, joins, cooling, acceleration and collisions.</p><div class="table-wrap"><table><thead><tr><th>Assumed tip speed</th><th>One pen</th><th>Two pens: ideal equal split</th></tr></thead><tbody>${[3,5,10].map(speed=>{const seconds=length/(speed/1000);return `<tr><td>${speed} mm/s</td><td>${(seconds/60).toFixed(1)} min</td><td>${(seconds/120).toFixed(1)} min</td></tr>`;}).join('')}</tbody></table></div>`;
  };
  $('#study-scale').addEventListener('input',draw);draw();viewer.focus('curves');
}

function renderPreparedMotions(allocation:CellAllocation,arms:CellArmView[],restart?:RestartPreview) {
  const data=allocation.motion_preparation!;
  $('#title').textContent='Construct only the selected motions.';
  $('#subtitle').textContent='Inspect approach, pen events, drawing and withdrawal for each assigned stroke.';
  $('#badge').textContent=data.status==='SUPERSEDED_TIMING_FORMULATION'?'Timing diagnostic · superseded':'Individual motions · common-clock replay pending';
  $('#metrics').innerHTML=`<h2>Selected full motions</h2>${metric(restart?'Verified restarts':'Prepared strokes',restart?String(restart.restart_count):`${data.prepared_strokes} / ${data.requested_strokes}`)}<label class="control">Assigned motion<select class="method-select" id="prepared-motion" aria-label="Selected full motion">${data.motions.map((m,i)=>`<option value="${i}">${escape(m.arm)} · ${escape(m.stroke_id)}</option>`).join('')}</select></label><label class="control">Motion time<input id="prepared-time" aria-label="Selected motion time" type="range" min="0" step="0.05" value="0"></label><label class="control">Playback speed<select id="prepared-rate" aria-label="Robot playback speed"><option value="1">1×</option><option value="5">5×</option><option value="10" selected>10×</option><option value="20">20×</option></select></label><button class="mode-button" id="prepared-play">Play selected motion</button><div id="prepared-detail"></div><p class="description">This view plays one prepared motion. The complete build requires separate replay, including travel and growing material.</p><a class="download" href="${url('allocation_motions.json')}" target="_blank">Open prepared motions ↗</a>`;
  $('#note-text').textContent='These are the motions chosen by the offline scheduler, including pen-off travel. No new drawing candidates were generated.';
  $('#output-summary').textContent=`${data.prepared_strokes} selected full motions prepared; shared-clock validation is separate.`;
  if(restart){
    $('#title').textContent='One arm · automatic stroke restarts';
    $('#subtitle').textContent='Play the verified partial Palace contour, including pen-off withdrawal and same-point returns.';
    $('#badge').textContent='One arm · partial contour · simulation';
    $('#note-text').textContent=`${restart.restart_count} verified restarts. The remaining contour stopped at the structural release limit. Cooling and physical pen behavior are uncalibrated.`;
    $('#output-summary').textContent='153 mm of the 273 mm contour · full build unfinished';
    $('#metrics h2').textContent='Palace restart trial';
    $('#metrics .description').textContent='Plays the complete saved restart trial, not the full Palace build. At 10×, the 4.9-minute trial plays in about 29 seconds.';
    const link=$<HTMLAnchorElement>('#metrics a.download');link.href=url('stroke_restart_preview.json');link.textContent='Open restart trial ↗';
  }
  const slider=$<HTMLInputElement>('#prepared-time');
  // Duration is not necessarily a multiple of the input's nominal step.
  // Preserve its exact endpoint so playback can finish and restart cleanly.
  slider.step='any';
  const select=$<HTMLSelectElement>('#prepared-motion');
  const primary=$<HTMLButtonElement>('#robot-play');
  primary.classList.remove('hidden');
  primary.onclick=()=>$('#prepared-play').click();
  const stop=()=>{if(playback)clearInterval(playback);playback=null;$('#prepared-play').textContent='Play selected motion';primary.textContent=restart?`Play restart trial · ${$<HTMLSelectElement>('#prepared-rate').value}×`:'Play robot motion';};
  const draw=()=>{
    const m=data.motions[Number(select.value)];
    const time=Number(slider.value);
    const motion={...m,variant:m.mode_id,start_s:0,end_s:m.trajectory.duration_s};
    viewer.setCellArmPose(m.arm,coordinatedPose(motion,time));
    const edge=m.trajectory.edges.find(e=>e.end_s>time)??m.trajectory.edges.at(-1)!;
    if(restart){
      const drawn=m.trajectory.edges.filter(e=>e.kind==='DRAW'&&e.end_s<=time).flatMap(e=>[e.tcp0_m!,e.tcp1_m!]);
      viewer.setDepositedPaths([...restart.prior_paths,drawn]);
    }
    $('#prepared-detail').innerHTML=metric('Time',`${time.toFixed(1)} / ${m.trajectory.duration_s.toFixed(1)} s`)+metric('Operation',edge.kind)+metric('Drawing clock',m.trajectory.clock==='continuous_drawing_mixed_clock'?'Continuous through samples':'Stops at samples')+(m.trajectory.tracking_position_bound_m!==undefined?metric('Drawing tracking bound',`${(m.trajectory.tracking_position_bound_m*1000).toFixed(4)} mm`):'')+(m.trajectory.tracking_orientation_constraint==='free'?metric('Pen orientation','Adjusted to clear deposited material'):'');
  };
  const change=()=>{
    stop();
    for(const arm of arms)viewer.setCellArmPose(arm.id,allocation.initial_positions?.[arm.id]??arm.initial_positions);
    const m=data.motions[Number(select.value)];slider.max=String(m.trajectory.duration_s);slider.value='0';
    viewer.setActivePath(m.trajectory.edges.filter(e=>e.kind==='DRAW').flatMap(e=>[e.tcp0_m!,e.tcp1_m!]),true);
    draw();
  };
  $('#prepared-rate').addEventListener('change',()=>{if(!playback)stop();});
  select.addEventListener('change',change);slider.addEventListener('input',()=>{stop();draw();});
  $('#prepared-play').addEventListener('click',()=>{
    if(playback){stop();return;}
    if(Number(slider.value)>=Number(slider.max))slider.value='0';
    $('#prepared-play').textContent='Pause';
    primary.textContent='Pause robot motion';
    let previous=performance.now();
    playback=setInterval(()=>{const now=performance.now();const next=Math.min(Number(slider.max),Number(slider.value)+(now-previous)/1000*Number($<HTMLSelectElement>('#prepared-rate').value));previous=now;slider.value=String(next);draw();if(next>=Number(slider.max))stop();},50);
  });
  change();
}

function renderCellCandidates(data:CellDrawingVariants, arms:CellArmView[]) {
  if(!data.variants.length)return;
  $('#title').textContent=arms.length===1?'Follow the arm’s drawing path.':'Follow each arm’s drawing path.';
  $('#subtitle').textContent='Inspect the fixed drawing catalogue, including robot posture and pen orientation.';
  $('#note-text').textContent='One arm moves at a time in this preview. Its path is checked against itself and static surfaces; the shared schedule and growing material require separate validation.';
  $('#input-summary').textContent='The original supported strokes, robot models and configured pen geometry.';
  $('#legend').innerHTML=arms.map((arm,i)=>`<span><i style="background:${i===0?'#327aa0':'#c0813f'}"></i>Arm ${i+1} tip</span>`).join('')+'<span><i style="background:#295d48"></i>Selected drawing path</span>';
  $('#metrics').insertAdjacentHTML('afterbegin',`<h2>Per-arm drawing alternatives</h2>${metric('Candidates',String(data.variants.length))}${data.requested_groups!==undefined?metric('Arm/action groups checked',`${data.groups.length} / ${data.requested_groups}`):''}${metric('Search',data.status==='RUNNING'?'Running':data.status.replaceAll('_',' '))}<div class="motion-controls"><label>Individual candidate<select id="cell-candidate" aria-label="Arm drawing candidate">${data.variants.map((v,i)=>`<option value="${i}">${i+1}. ${escape(v.arm)} · ${escape(v.stroke_id)} · roll ${(v.roll_rad*180/Math.PI).toFixed(0)}°</option>`).join('')}</select></label><label>Drawing waypoint <span id="cell-waypoint-count"></span></label><input id="cell-waypoint" type="range" min="0" value="0" aria-label="Arm drawing waypoint"><button id="cell-play">Play individual path</button></div><div id="cell-candidate-detail"></div><p class="description">One candidate moves at a time. These paths pass arm/self and static-surface checks, with sampled tip tracking. Inter-arm collisions, growing material, travel and scheduling are not checked in this view.</p><a class="download" href="${url('cell_drawing_variants.json')}" target="_blank">Open drawing alternatives ↗</a>`);
  const select=$<HTMLSelectElement>('#cell-candidate');
  const seek=$<HTMLInputElement>('#cell-waypoint');
  const candidateKey=(v:CellDrawingVariants['variants'][number])=>`${v.arm}/${v.stroke_id}/${v.variant}`;
  const selectedIndex=data.variants.findIndex(v=>candidateKey(v)===selectedCellCandidate);
  select.value=String(Math.max(0,selectedIndex));
  const stop=()=>{if(playback)clearInterval(playback);playback=null;$('#cell-play').textContent='Play individual path';};
  const draw=()=>{
    const v=data.variants[Number(select.value)];
    const index=Math.min(Number(seek.value),v.positions.length-1);
    selectedCellWaypoint=index;
    viewer.setCellArmPose(v.arm,v.positions[index]);
    $('#cell-waypoint-count').textContent=`${index+1} / ${v.positions.length}`;
  };
  const change=(restoreWaypoint=false)=>{
    stop();
    for(const arm of arms)viewer.setCellArmPose(arm.id,arm.initial_positions);
    const v=data.variants[Number(select.value)];
    selectedCellCandidate=candidateKey(v);
    seek.max=String(v.positions.length-1);
    seek.value=String(restoreWaypoint?Math.min(selectedCellWaypoint,v.positions.length-1):0);
    viewer.setActivePath(v.tcp_targets_m,true);
    $('#cell-candidate-detail').innerHTML=metric('Start anchor',escape(v.start_anchor))+metric('Direction',v.reverse?'Reverse':'Forward')+metric('Pen roll',(v.roll_rad*180/Math.PI).toFixed(0)+'°')+metric('Drawing pen tilt',(v.pen_tilt_deg??0).toFixed(0)+'°')+metric('Sampled tip error',(v.maximum_sampled_interpolation_error_m*1000).toFixed(3)+' mm');
    draw();
  };
  select.addEventListener('change',()=>change());seek.addEventListener('input',()=>{stop();draw();});
  $('#cell-play').addEventListener('click',()=>{
    if(playback){stop();return;}
    if(Number(seek.value)>=Number(seek.max))seek.value='0';
    $('#cell-play').textContent='Pause individual path';
    playback=setInterval(()=>{seek.value=String(Math.min(Number(seek.max),Number(seek.value)+1));draw();if(seek.value===seek.max)stop();},60);
  });
  $('#badge').textContent=`${arms.length===1?'One arm':'Two arms'} · individual path candidates`;
  $('#output-summary').textContent=`${data.variants.length} arm-specific drawing alternatives. No coordinated schedule yet.`;
  $('#next-summary').textContent='Generate approach/withdrawal paths and validate assignments on the shared clock.';
  change(true);
}

function renderCellSchedule(plan:CoordinatedPlan,arms:CellArmView[],motionReuse?:MotionReuse|null) {
  if(!playbackReady(plan)){
    $('#title').textContent='Build planned · verification pending';
    $('#subtitle').textContent='Playback becomes available after separate full-robot verification passes.';
    $('#badge').textContent='Awaiting full-robot verification';
    $('#note-text').textContent='The viewer only animates saved trajectories. Verification runs separately.';
    $('#metrics').insertAdjacentHTML('afterbegin',metric('Scheduled strokes',`${plan.actions.filter(a=>!a.auxiliary).length} / ${plan.requested_strokes}`)+metric('Independent verification',escape(replayProgressLabel(plan))));
    $('#output-summary').textContent='Complete saved schedule; playback awaits verification.';
    $('#next-summary').textContent='Run the separate full-robot verifier, then reload this result.';
    return;
  }
  const single=plan.resource_mode==='single';
  const full=plan.full_schedule||plan.status==='COORDINATED_GEOMETRIC_PREVIEW';
  const playLabel=single?(full?'Play full one-arm build':'Play one-arm prefix'):'Play both arms';
  const pauseLabel=single?'Pause one-arm build':'Pause both arms';
  if(single)$('#title').textContent=full?'One pen. The complete sculpture.':'One pen · prepared prefix';
  const total=plan.modeled_duration_s, state=replayState(plan), complete=state==='full', verified=state!=='unverified';
  const replayLabel=replayProgressLabel(plan);
  const scheduled=new Set(plan.actions.filter(a=>!a.auxiliary).map(a=>a.stroke_id)).size;
  $('#subtitle').textContent='Inspect saved robot trajectories and pen commands on one clock, with their independent replay status.';
  $('#note-text').textContent=complete?'The saved complete trajectory passed modeled geometric replay. Physical fabrication remains unverified.':verified?'This prefix passed modeled geometric replay. The remaining sculpture is not yet planned.':'This is diagnostic playback of the saved trajectory. It does not establish collision clearance or physical feasibility.';
  $('#input-summary').textContent='Saved per-arm trajectories, independent pen events and the matching replay report when available.';
  const minutes=(value:number)=>(value/60).toFixed(1)+' min';
  const rows=plan.actions.flatMap(action=>action.trajectory.edges.map(edge=>({action,edge,start:action.start_s+edge.start_s,end:action.start_s+edge.end_s})));
  const deposition=rows.filter(row=>row.edge.kind==='DRAW'&&row.start<total).map(row=>({
    ...row,p0:row.edge.tcp0_m??viewer.setCellArmPose(row.action.arm,row.edge.q0),
    p1:row.edge.tcp1_m??viewer.setCellArmPose(row.action.arm,row.edge.q1),
  }));
  const events=deposition.flatMap(row=>[{time:row.start,delta:1},{time:Math.min(total,row.end),delta:-1}]).sort((a,b)=>a.time-b.time);
  let active=0, previous=0, concurrent=0;
  for(const event of events){if(active>1)concurrent+=event.time-previous;active+=event.delta;previous=event.time;}
  $('#metrics').insertAdjacentHTML('afterbegin',`<h2>Synchronized trajectory</h2>${metric('Scheduled strokes',`${scheduled} / ${plan.requested_strokes}`)}${metric('Replay completed strokes',`${plan.completed_strokes} / ${plan.requested_strokes}`)}${metric(full||complete?'Modeled duration':'Planned prefix duration',minutes(total))}${metric('Concurrent drawing',minutes(concurrent))}${metric('Replay intervals checked',String(plan.checked_intervals))}${metric('Independent replay',escape(replayLabel))}<p class="description">${complete?'Complete geometric replay passed; physical feasibility remains unverified.':verified?`Verified prefix: ${plan.completed_strokes} of ${plan.requested_strokes} drawing actions. The complete sculpture is not yet scheduled.`:full?'Full planned trajectory for inspection. Playback is not evidence that collision replay passed.':'Partial planned trajectory; the complete sculpture is not scheduled yet.'}</p><div class="motion-controls"><label>Shared job time <span id="dual-clock"></span><input type="range" id="dual-seek" aria-label="${single?'One-arm job time':'Two-arm job time'}" min="0" max="${total}" value="0" step="0.05"></label><button id="dual-play">Play both arms</button><label>Playback speed<select id="dual-rate" aria-label="${single?'One-arm playback speed':'Two-arm playback speed'}"><option value="1">1×</option><option value="10" selected>10×</option><option value="30">30×</option></select></label></div><div id="dual-state"></div><a class="download" href="${url(plan.file_name??'dual_arm_plan.json')}" target="_blank">Open coordinated plan ↗</a>`);
  $('#badge').textContent=complete?(single?'One-arm geometric replay passed':'Two-arm geometric replay passed'):verified?`Verified prefix · ${plan.completed_strokes}/${plan.requested_strokes} actions`:full?'Full schedule · replay '+(plan.replay_in_progress?'running':plan.replay?.status==='REPLAY_REJECTED'?'rejected':'unverified'):'Partial trajectory · replay unverified';
  $('#metrics').insertAdjacentHTML('beforeend','<button class="mode-button" id="schedule-view">Focus sculpture</button>');
  $('#schedule-view').addEventListener('click',()=>{
    const close=$('#schedule-view').textContent==='Focus sculpture';
    if(close)viewer.focusPoints(deposition.flatMap(row=>[row.p0,row.p1]));
    else viewer.focus('cell');
    $('#schedule-view').textContent=close?'Show full robot cell':'Focus sculpture';
  });
  const panel=$('#timing-panel');panel.classList.remove('hidden');
  const colors:Record<string,string>={DRAW:'#b77636',TRAVEL:'#367ca5',APPROACH:'#759aaa',WITHDRAW:'#759aaa',RETURN:'#367ca5',PARK:'#367ca5',JOIN:'#657c68',PEN_START_WAIT:'#a88a65',PEN_STOP_WAIT:'#a88a65',WAIT:'#d9dcd6',VERTEX_COOL:'#aac8b0',INDEX_TURN:'#8957a3',TABLE_SETTLE:'#ab91bd'};
  panel.innerHTML=`<h2>${single?'One-arm build timeline':'Two arms on one job clock'}</h2>${plan.initial_pose_repairs?.precondition?`<p><strong>Start pose:</strong> ${escape(arms.length===1?'Start the robot at its saved staged pose':plan.initial_pose_repairs.precondition)}. Moving into those poses is outside this preview.</p>`:''}<p>${arms.length===1?'The robot pose and growing strokes use the same timestamp.':'Both robot poses and growing strokes use the same timestamp.'} Select a timeline phase to inspect its start. Gray intervals hold the previous pose. Startup and shutdown remain unmeasured; cooling is a development assumption.</p>${arms.map(arm=>`<h3>${escape(arm.id)}</h3><div class="timing-track" role="group" aria-label="${escape(arm.id)} shared timeline" style="position:relative">${coordinatedTimeline(plan,arm.id).map(r=>{const duration=r.end_s-r.start_s;const durationLabel=duration<0.001?'under 1 ms':duration<1?`${(duration*1000).toFixed(1)} ms`:`${duration.toFixed(1)} s`;const label=`${r.kind.replaceAll('_',' ')} · ${r.stroke_id??'Holding pose'} · ${r.start_s.toFixed(3)}–${r.end_s.toFixed(3)} s (${durationLabel})`;return `<button class="timeline-phase" type="button" data-job-time="${r.start_s}" aria-label="${escape(label)}" title="${escape(label)}" style="position:absolute;top:0;bottom:0;border:0;padding:0;cursor:pointer;left:${100*r.start_s/Math.max(total,1e-9)}%;width:${100*(r.end_s-r.start_s)/Math.max(total,1e-9)}%;background:${colors[r.kind]??'#888'}"></button>`;}).join('')}</div>`).join('')}<div class="timing-legend">${['DRAW','TRAVEL','APPROACH','JOIN','WAIT',...(plan.actions.some(a=>a.trajectory.edges.some(e=>e.kind==='VERTEX_COOL'))?['VERTEX_COOL']:[])].map(kind=>`<span><i style="background:${colors[kind]}"></i>${kind.toLowerCase().replaceAll('_',' ')}</span>`).join('')}</div><p>${plan.actions.some(a=>a.trajectory.clock==='continuous_drawing_mixed_clock')?'Drawing stays connected through numerical samples. Required source corners and cooling holds stop motion without a pen withdrawal.':'Timing stops at geometric waypoints to preserve checked paths. Pen flow through those stops is uncalibrated.'} Pen/mount geometry and hot-strand behavior remain unmeasured.</p>${plan.failures.length?`<details><summary>Rejected motion alternatives (${plan.failures.length})</summary>${plan.failures.slice(0,12).map(f=>`<p>${escape(f.arm)} · ${escape(f.stroke_id)}: ${escape(f.phase)} / ${escape(f.status)}</p>`).join('')}</details>`:''}`;
  if(plan.comparison){
    const comparison=plan.comparison;
    panel.insertAdjacentHTML('beforeend',`<h3>One arm / two arms</h3><p>Same sculpture, staged poses, cell and process assumptions. The unused arm remains parked for the one-arm run.</p><table><thead><tr><th>Run</th><th>Completed</th><th>Modeled duration</th><th>Replay</th></tr></thead><tbody>${Object.entries(comparison.results).map(([name,row])=>`<tr><td>${name==='single_arm'?'One arm':'Two arms'}</td><td>${row.completed_strokes} / ${row.requested_strokes}</td><td>${row.modeled_duration_s===null?'Incomplete':minutes(row.modeled_duration_s)}</td><td>${escape(row.replay_status.replaceAll('_',' ').toLowerCase())}</td></tr>`).join('')}</tbody></table><p>${comparison.speedup===null?'A speedup comparison needs two complete, replay-checked schedules.':`Modeled speedup: ${comparison.speedup.toFixed(2)}×. Unknown setup times and uncalibrated pen behavior are excluded.`}</p>`);
  }
  if(plan.events?.length)panel.insertAdjacentHTML('beforeend',`<details><summary>Pen command events (${plan.events.length})</summary><table><thead><tr><th>Time</th><th>Arm</th><th>Command</th><th>Stroke</th></tr></thead><tbody>${plan.events.map(event=>`<tr><td>${event.time_s.toFixed(2)} s</td><td>${escape(event.arm)}</td><td>${escape(event.command)}</td><td>${escape(event.stroke_id)}</td></tr>`).join('')}</tbody></table></details>`);
  if(plan.trajectory_export_available)panel.insertAdjacentHTML('beforeend',`<h3>Trajectory export</h3><p>Exact timed joint position, velocity and acceleration polynomials, TCP endpoint poses and synchronized pen events. Simulation-only; this download does not command hardware.</p><a class="download" href="${url('trajectory.json')}" target="_blank">Open trajectory package ↗</a> <a class="download" href="${url('joint_path.json')}" target="_blank">Open source joint paths ↗</a>`);
  if(plan.search_failures?.length)panel.insertAdjacentHTML('beforeend',`<details><summary>Schedule conflicts (${plan.search_failures.length})</summary>${plan.search_failures.slice(0,12).map(f=>`<p>${f.time_s.toFixed(2)} s · ${escape(f.arm??'both arms')} · ${escape(f.phase)}: ${escape(f.status)}</p>`).join('')}</details>`);
  const seek=$<HTMLInputElement>('#dual-seek');
  seek.step='any';
  const primary=$<HTMLButtonElement>('#robot-play');
  primary.classList.remove('hidden');
  primary.textContent=playLabel;
  $('#dual-play').textContent=playLabel;
  if(single){
    $('#timing-panel h2').textContent=arms.length===1?'One-arm build timeline':'One active arm on the shared job clock';
    $('#metrics').insertAdjacentHTML('afterbegin',metric('Drawing resources',arms.length===1?'One robot and one pen':'One active arm; other arm parked'));
  }
  primary.onclick=()=>$('#dual-play').click();
  const draw=(time:number)=>{
    time=Math.max(0,Math.min(total,time));seek.value=String(time);
    const details:string[]=[];
    const tableAngle=tableAngleAt(plan,time);
    if(plan.turntable){
      viewer.setTurntableAngle(tableAngle);
      details.push(metric('Turntable angle',`<span id="table-angle" data-radians="${tableAngle}">${(tableAngle*180/Math.PI).toFixed(1)}°</span>`));
    }
    for(const arm of arms){
      const moves=plan.actions.filter(a=>a.arm===arm.id);
      const motion=moves.find(a=>a.start_s<=time&&time<a.end_s);
      const past=moves.filter(a=>a.end_s<=time).at(-1);
      viewer.setCellArmPose(arm.id,motion?coordinatedPose(motion,time):past?coordinatedPose(past,past.end_s):(plan.initial_positions?.[arm.id]??arm.initial_positions));
      const edge=motion?.trajectory.edges.find(e=>e.end_s>time-motion.start_s);
      const kind=edge?.kind??'WAIT';
      const feedEvent=plan.events?.filter(event=>event.arm===arm.id&&event.time_s<=time).at(-1);
      const feeding=plan.events?feedEvent?.command==='FEED_ON':['PEN_START_WAIT','JOIN','DRAW'].includes(kind);
      details.push(`<h2>${escape(arm.id)}</h2>${metric('Action',kind.replaceAll('_',' '))}${metric('Requested feed',feeding?'ON':'OFF')}<p class="description">${motion?escape(motion.stroke_id):'Holding previous pose'}</p>`);
      if(motion?.parent_curve)details.push(metric('Source section',`${escape(motion.parent_curve)} [${motion.source_interval?.join(', ')??''}]`)+metric('Build stage',escape(motion.stage??''))+metric('Material role',escape(motion.role??'')));
    }
    const rotateDeposit=(point:number[],index?:number)=>{
      if(!plan.turntable||index===undefined)return point;
      const axis=plan.turntable.world_from_axis;
      const x=point[0]-axis[0][3],y=point[1]-axis[1][3];
      const angle=tableAngle-index*Math.PI/2,c=Math.cos(angle),s=Math.sin(angle);
      return [axis[0][3]+c*x-s*y,axis[1][3]+s*x+c*y,point[2]];
    };
    viewer.setDepositedPaths(deposition.filter(r=>r.start<=time).map(r=>{
      if(r.end<=time)return [rotateDeposit(r.p0,r.edge.table_index),rotateDeposit(r.p1,r.edge.table_index)];
      return [rotateDeposit(r.p0,r.edge.table_index),viewer.setCellArmPose(r.action.arm,coordinatedPose(r.action,time))];
    }));
    $('#dual-clock').textContent=`${minutes(time)} / ${minutes(total)}`;
    $('#dual-state').innerHTML=details.join('');
  };
  const stop=()=>{if(playback)clearInterval(playback);playback=null;$('#dual-play').textContent=playLabel;primary.textContent=playLabel;};
  panel.querySelectorAll<HTMLButtonElement>('.timeline-phase').forEach(button=>button.addEventListener('click',()=>{stop();draw(Number(button.dataset.jobTime));}));
  seek.addEventListener('input',()=>{stop();draw(Number(seek.value));});
  $('#dual-play').addEventListener('click',()=>{
    if(playback){stop();return;}if(Number(seek.value)>=total)draw(0);
    $('#dual-play').textContent=pauseLabel;primary.textContent=pauseLabel;let last=performance.now();
    playback=setInterval(()=>{const now=performance.now();const t=Number(seek.value)+(now-last)/1000*Number($<HTMLSelectElement>('#dual-rate').value);last=now;draw(t);if(t>=total)stop();},100);
  });
  const conflict=plan.replay?.failures?.find(f=>f.time_s!==undefined);
  if(conflict){
    panel.insertAdjacentHTML('afterbegin',`<section class="planner-checkpoint"><strong>Recorded replay failure</strong><p>${escape(conflict.phase??conflict.arm??'Replay')} · ${escape(conflict.status)} at interval starting ${conflict.time_s!.toFixed(2)} s.</p><button id="replay-conflict">Inspect recorded conflict</button></section>`);
    $('#replay-conflict').addEventListener('click',()=>{
      stop();draw(conflict.time_s!);
      if(conflict.positions?.length){
        if(Array.isArray(conflict.positions[0]))arms.forEach((arm,i)=>viewer.setCellArmPose(arm.id,(conflict.positions as number[][])[i]));
        else if(conflict.arm)viewer.setCellArmPose(conflict.arm,conflict.positions as number[]);
      }
      viewer.focus('curves');
      $('#note-text').textContent=`Recorded failure pose: ${conflict.pairs?.map(p=>p.first+' / '+p.second).join(', ')??conflict.status}. The interval start time and interior witness pose can differ.`;
    });
  }
  $('#output-summary').textContent=`${scheduled} of ${plan.requested_strokes} strokes scheduled; ${complete?'geometric replay passed':full?'full trajectory, validation not passed':'partial trajectory'}.`;
  $('#next-summary').textContent=complete?'Calibrate pen flow, cooling and physical support before hardware execution.':'Complete scheduling and independent replay, then calibrate pen flow and physical support.';
  if(motionReuse) {
    $('#title').textContent=motionReuse.label;
    renderMotionReuse($('#motion-reuse-panel'),motionReuse,stroke=>{
      stop();draw(stroke.draw_start_s);
      viewer.setActivePath(stroke.source_points_m,true);
      viewer.focusPoints(stroke.source_points_m);
    });
  }
  draw(0);
}

function renderTiming(schedule:TimedSchedule, robot:RobotRig) {
  const moves=schedule.operations, total=schedule.modeled_duration_s;
  const minutes=(seconds:number)=>(seconds/60).toFixed(1)+' min';
  const colors:Record<string,string>={DRAW:'#b77636',TRAVEL:'#367ca5',RETURN:'#367ca5',APPROACH:'#759aaa',WITHDRAW:'#759aaa',JOIN:'#657c68',START_RESPONSE:'#a88a65',COOL:'#93a696',STARTUP:'#8b8b8b',SHUTDOWN:'#8b8b8b'};
  const totals=new Map<string,number>();
  for(const op of moves)totals.set(op.kind,(totals.get(op.kind)??0)+op.duration_s);
  $('#badge').textContent='Timing preview · unvalidated';
  $('#frame-label').textContent='Robot base · meters';
  $('#scale-label').textContent='Smooth paths require a new collision audit';
  $('#metrics').innerHTML=`<h2>Modeled schedule</h2><div class="metric big"><strong>${(total/60).toFixed(1)}<span style="font-size:15px"> min</span></strong><span class="metric-label">Subtotal with current assumptions</span></div>${metric('Target',minutes(schedule.profile.target_seconds))}${metric('Complete duration',schedule.unknown_durations.length?'Unknown overhead remains':minutes(total))}${metric('Feed timing conflicts',String(schedule.timing_conflicts.length))}<p class="description">${schedule.status==='RUNNING'?'Only processed operations are shown; retiming is still running.':'Includes motion, joins and assumed pen-response delays.'} Calibration, smooth-path collision checks and physical printing remain unvalidated.</p><label class="control">Schedule time <span id="timing-clock"></span></label><input type="range" id="timing-seek" aria-label="Schedule time" min="0" max="${total}" step="0.05" value="0"><div id="timing-current"></div><button id="timing-play">Play schedule</button><label class="control">Playback speed<select id="timing-rate" aria-label="Timing playback speed"><option value="1">1×</option><option value="10" selected>10×</option><option value="30">30×</option></select></label><a class="download" href="${url('timed_schedule.json')}" target="_blank">Open timed schedule ↗</a>`;
  const panel=$('#timing-panel');panel.classList.remove('hidden');
  const maxSpeed=moves.reduce((maximum,op)=>(op.tcp_speeds_m_s??[]).reduce((m,v)=>Math.max(m,v),maximum),.003);
  panel.innerHTML=`<h2>Motion and pen events on one clock</h2><p>This is a simulated schedule. Natural cubic joint paths pass through the planned waypoints without stopping at every sample. Smoothing changes the path between waypoints; the earlier linear-path collision audit does not transfer.</p><div class="timing-track" role="img" aria-label="Operation durations over the modeled schedule">${moves.filter(op=>op.duration_s>0).map(op=>`<span title="${escape(op.kind)} · ${op.duration_s.toFixed(2)} s" style="width:${100*op.duration_s/Math.max(total,1e-9)}%;background:${colors[op.kind]??'#888'}"></span>`).join('')}</div><div class="timing-legend">${[...totals].map(([kind,duration])=>`<span><i style="background:${colors[kind]??'#888'}"></i>${kind.replaceAll('_',' ')} · ${minutes(duration)}</span>`).join('')}</div><h3>Modeled tip speed</h3><svg id="timing-speed-chart" viewBox="0 0 900 160" role="img" aria-label="Modeled tip speed over time"><line x1="0" y1="140" x2="900" y2="140" stroke="#ccd4c9"/>${moves.filter(op=>op.times_s&&op.tcp_speeds_m_s).map(op=>{const step=Math.max(1,Math.floor(op.times_s!.length/120));const points=op.times_s!.map((t,i)=>i%step===0||i===op.times_s!.length-1?`${900*(t+op.start_s)/Math.max(total,1e-9)},${140-125*op.tcp_speeds_m_s![i]/maxSpeed}`:null).filter(Boolean).join(' ');return `<polyline points="${points}" fill="none" stroke="${colors[op.kind]??'#888'}" stroke-width="1.3"/>`;}).join('')}<line id="timing-cursor" x1="0" y1="5" x2="0" y2="140" stroke="#26352f" stroke-dasharray="4 3"/><text x="0" y="157" font-size="10">0 min</text><text x="800" y="157" font-size="10">${minutes(total)}</text><text x="0" y="12" font-size="10">${(maxSpeed*1000).toFixed(1)} mm/s</text></svg><h3>Unresolved inputs</h3><p>${schedule.unknown_durations.length?schedule.unknown_durations.map(name=>escape(name.replaceAll('_',' '))).join(', ')+' have no measured duration and contribute no time to this subtotal.':'All configured durations are included; configured values still require physical validation.'} Joint acceleration limits and pen settings are assumptions. Cooling, feed response, strand diameter and controller interpolation need calibration.</p>${schedule.timing_conflicts.length?`<h3>Feed timing conflicts</h3>${schedule.timing_conflicts.map(c=>`<p><strong>${escape(c.stroke_id)}</strong>: ${escape(c.message)}</p>`).join('')}`:''}<details><summary>Pen command events (${schedule.events.length})</summary><table><thead><tr><th>Time</th><th>Command</th><th>Stroke</th></tr></thead><tbody>${schedule.events.map(e=>`<tr><td>${e.time_s.toFixed(2)} s</td><td>${escape(e.command)}</td><td>${escape(e.stroke_id)}</td></tr>`).join('')}</tbody></table></details>`;
  const seek=$<HTMLInputElement>('#timing-seek');
  const draw=(time:number)=>{
    time=Math.max(0,Math.min(total,time));seek.value=String(time);
    const index=moves.findIndex(op=>op.end_s>time+1e-9);
    const op=moves[index<0?moves.length-1:index];
    if(!op)return;
    viewer.setRobotPose(timedPose(op,time),robot,schedule.flange_to_tcp);
    const earlier=moves.filter(row=>row.kind==='DRAW'&&row.end_s<=time).map(row=>row.tcp_path_m!);
    viewer.setDepositedPaths(earlier);
    if(op.tcp_path_m){
      const count=op.times_s!.filter(t=>t<=time-op.start_s).length;
      viewer.setActivePath(op.kind==='DRAW'?op.tcp_path_m.slice(0,Math.max(1,count)):op.tcp_path_m,op.kind==='DRAW');
    } else viewer.setActivePath([],false);
    const command=schedule.events.filter(event=>event.time_s<=time).at(-1)?.command??'STOP_FEED';
    $('#timing-clock').textContent=`${minutes(time)} / ${minutes(total)}`;
    $('#timing-current').innerHTML=metric('Operation',op.kind.replaceAll('_',' '))+metric('Feed command',command==='START_FEED'?'ON':'OFF')+metric('Expected flow',op.extrusion_expected?'Drawing · assumption':'OFF')+`<p class="description">${op.stroke_id?escape(op.stroke_id):'Job overhead'}</p>`;
    const x=900*time/Math.max(total,1e-9);$('#timing-cursor').setAttribute('x1',String(x));$('#timing-cursor').setAttribute('x2',String(x));
  };
  const stop=()=>{if(playback)clearInterval(playback);playback=null;$('#timing-play').textContent='Play schedule';};
  seek.addEventListener('input',()=>{stop();draw(Number(seek.value));});
  $('#timing-play').addEventListener('click',()=>{
    if(playback){stop();return;}
    if(Number(seek.value)>=total)draw(0);
    $('#timing-play').textContent='Pause schedule';let previous=performance.now();
    playback=setInterval(()=>{const now=performance.now();const time=Number(seek.value)+(now-previous)/1000*Number($<HTMLSelectElement>('#timing-rate').value);previous=now;draw(time);if(time>=total)stop();},100);
  });
  $('#note-text').textContent='The timeline is not a hardware program. Feed commands and expected plastic flow are distinct; their timing relationship must be measured.';
  $('#input-summary').textContent='Complete geometric sequence, URDF velocity limits and explicit timing assumptions.';
  $('#output-summary').textContent='Smooth joint paths, timestamps, velocity/acceleration bounds and synchronized feed events.';
  $('#next-summary').textContent='Recheck the smoothed geometry and calibrate the pen before physical execution.';
  $('#legend').innerHTML='<span><i></i>Modeled deposited material</span><span><i style="background:#367ca5"></i>Pen-off motion</span>';
  draw(0);
  return draw;
}

function renderSmoothAudit(audit:SmoothAudit, schedule:TimedSchedule, robot:RobotRig, seek:(time:number)=>void) {
  const rows=audit.operations;
  const clear=rows.filter(row=>row.status==='CERTIFIED_MODELED_PATH').length;
  const tracking=rows.filter(row=>row.tracking_status==='BOUNDED').length;
  const panel=$('#audit-panel');panel.classList.remove('hidden');
  panel.innerHTML=`<h2>Audit of the smoothed paths</h2><p>${clear} / ${audit.requested_operations} operations have bounded modeled arm clearance. ${tracking} / ${audit.requested_operations} meet the checked Cartesian tracking tolerances. ${audit.status==='RUNNING'?'The audit is still running.':''} These counts do not establish complete fabrication feasibility.</p><label class="control">Inspect operation<select id="audit-operation" aria-label="Audit operation">${rows.map((row,i)=>`<option value="${i}">${row.id} · ${row.kind} · ${row.status==='CERTIFIED_MODELED_PATH'?'arm clear':row.status}${row.tracking_status!=='BOUNDED'?' · tracking needs repair':''}</option>`).join('')}</select></label><div id="audit-detail"></div><h3>What is checked</h3><p>The saved cubic path is bounded between samples. Deposited strands grow in construction order and are enclosed by capsules, including the modeled deviation from straight chords. Configured non-contact tool bodies are included when supplied.</p><h3>Still outside this audit</h3><ul>${audit.limitations.map(text=>`<li>${escape(text)}</li>`).join('')}</ul><a class="download" href="${url('smooth_collisions.json')}" target="_blank">Open smooth-path audit ↗</a>`;
  const show=(index:number)=>{
    const row=rows[index];if(!row)return;
    const op=schedule.operations.find(op=>op.id===row.id)!;
    if(playback){clearInterval(playback);playback=null;$('#timing-play').textContent='Play schedule';}
    seek((op.start_s+op.end_s)/2);
    const witness=row.witnesses[0];
    if(witness?.positions)viewer.setRobotPose(witness.positions,robot,schedule.flange_to_tcp);
    viewer.highlightRobotLinks((witness?.pairs??[]).flatMap(pair=>[pair.first.split(':')[0],pair.second.split(':')[0]]),robot);
    $('#audit-detail').innerHTML=`${metric('Modeled arm clearance',escape(row.status.replaceAll('_',' ')))}${metric('Tracking',escape(row.tracking_status.replaceAll('_',' ')))}${metric('Position error bound',(row.position_error_bound_m*1000).toFixed(3)+' mm')}${metric('Orientation error bound',(row.orientation_error_bound_rad*180/Math.PI).toFixed(3)+'°')}<p>${witness?'The viewport shows a saved collision/clearance witness.':'The viewport shows this operation at its midpoint.'} A tracking bound above tolerance calls for path refinement; it is not a claim of physical fabrication failure.</p>`;
  };
  $('#audit-operation').addEventListener('change',()=>show(Number($<HTMLSelectElement>('#audit-operation').value)));
}

function renderSequence(sequence:SequencePreview, robot:RobotRig) {
  viewer.setLimit(0);
  viewer.setToolModel(sequence.tool_model);
  viewer.highlightRobotLinks([],robot);
  $('#title').textContent='Follow the whole construction.';
  $('#subtitle').textContent='Drawing, withdrawal, travel and approach, with the deposited structure growing in order.';
  $('#badge').textContent=sequence.status==='GEOMETRIC_SEQUENCE_PREVIEW' ? 'Sequence preview' : 'Partial sequence';
  const assumptions=$('#metrics').innerHTML;
  const moves=sequence.operations;
  $('#metrics').innerHTML=`<details class="setup-details"><summary>Robot & placement assumptions</summary>${assumptions}</details><button id="drawing-mode" class="mode-button">View individual drawing paths</button><h2>Construction + travel</h2>${metric('Strokes deposited',`${sequence.completed_strokes} / ${sequence.requested_strokes}`)}${metric('Withdrawals complete',String(moves.filter(op=>op.kind==='WITHDRAW').length))}${metric('Operations',String(moves.length))}${sequence.search ? metric('Order backtracks',String(sequence.search.backtracks))+metric('Transitions tried',String(sequence.search.attempts)) : ''}${metric('Deposited segments',number(sequence.material_segments))}${sequence.settings.contact_radius_m ? metric('Local join allowance',(sequence.settings.contact_radius_m*1000).toFixed(1)+' mm') : ''}<div class="motion-controls"><label>Operation<select id="sequence-operation" aria-label="Sequence operation">${moves.map((op,i)=>`<option value="${i}">${i+1}. ${escape(op.kind)} ${escape(op.stroke_id ?? '')}</option>`).join('')}</select></label><label>Waypoint <span id="sequence-count"></span></label>${metric('Extrusion','<span id="sequence-feed">—</span>')}<input id="sequence-sample" aria-label="Sequence waypoint" type="range" min="0" value="0"><button id="sequence-play">Play sequence</button><button id="sequence-refresh">Refresh results</button></div><p id="departure-detail" class="description"></p><p class="description">Bronze lines are deposited material. Blue is pen-off motion. Playback speed is schematic, not fabrication time. The robot starts at a planned parking pose.</p><h2>Validation scope</h2><p class="description">The arm is checked against the table, platform and growing strands. Free travel also checks an idealized tip sphere. ${sequence.settings.contact_radius_m ? 'Lifts also check the idealized tip, permitting contact only in the purple local join region. ' : ''}${sequence.tool_model?.configured ? 'Configured non-contact pen/mount bodies are included.' : 'Pen and mount bodies are not modeled.'} Drawing contact, process timing and physical stability remain unvalidated.</p>${sequence.failures.length ? `<p class="description">Last search result: ${escape(sequence.failures[0].phase)} / ${escape(sequence.failures[0].status)}</p>`:''}<a class="download" href="${url('sequence_motion.json')}" target="_blank">Open sequence artifact ↗</a>`;
  $('#metrics').insertAdjacentHTML('beforeend',toolSummary(sequence.tool_model));
  $('#drawing-mode').addEventListener('click',()=>{motionMode='drawings';selectStage('motion').catch(showError);});
  $('#sequence-refresh').addEventListener('click',()=>{const id=run.id;json<Run>(`/api/runs/${encodeURIComponent(id)}`).then(updated=>{if(run.id!==id)return;run=updated;return selectStage('motion');}).catch(showError);});
  $('#legend').innerHTML='<span><i></i>Deposited material</span><span><i style="background:#367ca5"></i>Pen-off path</span>';
  $('#note-text').textContent='This joins geometric drawing paths into a single sequence. A measured and verified tool model, valid deposition and calibrated timing are still required before execution.';
  $('#output-summary').textContent=`${sequence.completed_strokes} drawing strokes with explicit travel and lifts.`;
  $('#next-summary').textContent='Check the actual pen geometry and contact regions, then add calibrated timing.';
  if (!moves.length) return;
  let operation=0;
  const stop=()=>{if(playback)clearInterval(playback);playback=null;$('#sequence-play').textContent='Play sequence';};
  const sample=(index:number)=>{
    const op=moves[operation];
    index=Math.max(0,Math.min(op.positions.length-1,index));
    const slider=$<HTMLInputElement>('#sequence-sample');slider.max=String(op.positions.length-1);slider.value=String(index);
    $('#sequence-count').textContent=`${index+1} / ${op.positions.length}`;
    $('#sequence-feed').textContent=op.extrusion_enabled ? 'ON · drawing' : 'OFF';
    const actual=viewer.setRobotPose(op.positions[index],robot,sequence.flange_to_tcp);
    const error=Math.hypot(...actual.map((v,i)=>v-op.tcp_path_m[index][i]));
    if(error>.000001)throw new Error('Displayed sequence TCP does not match the generated path.');
    viewer.setActivePath(op.kind==='DRAW' ? op.tcp_path_m.slice(0,index+1) : op.tcp_path_m,op.kind==='DRAW');
  };
  const choose=(index:number)=>{
    operation=index;
    $<HTMLSelectElement>('#sequence-operation').value=String(index);
    viewer.setDepositedPaths(moves.slice(0,index).filter(op=>op.kind==='DRAW').map(op=>op.tcp_path_m));
    const op=moves[index];
    $('#departure-detail').textContent=op.departure_direction ? `Exit direction: ${op.departure_direction.map(v=>v.toFixed(2)).join(', ')}. ${op.departure_attempts?.length ?? 1} direction(s) tested with the same clearance checks.` : '';
    const contact=op.kind==='APPROACH' ? op.tcp_path_m.at(-1) : op.kind==='WITHDRAW' ? op.tcp_path_m[0] : undefined;
    viewer.setContactRegion(contact,sequence.settings.contact_radius_m);
    sample(0);
  };
  $('#sequence-operation').addEventListener('change',()=>{stop();choose(Number($<HTMLSelectElement>('#sequence-operation').value));});
  $('#sequence-sample').addEventListener('input',()=>{stop();sample(Number($<HTMLInputElement>('#sequence-sample').value));});
  $('#sequence-play').addEventListener('click',()=>{
    if(playback){stop();return;}
    $('#sequence-play').textContent='Pause sequence';
    playback=setInterval(()=>{
      const next=Number($<HTMLInputElement>('#sequence-sample').value)+Math.max(1,Math.floor(moves[operation].positions.length/120));
      if(next>=moves[operation].positions.length){
        sample(moves[operation].positions.length-1);
        if(operation+1<moves.length)choose(operation+1);else stop();
      }else sample(next);
    },60);
  });
  choose(0);
}

async function loadRun(id: string) {
  const request = ++runRequest;
  ++stageRequest;
  const loadedRun = await json<Run>(`/api/runs/${encodeURIComponent(id)}`);
  const loadedManifest = await json<Manifest>(`/api/runs/${encodeURIComponent(id)}/artifacts/asset_manifest.json`);
  if (request !== runRequest) return;
  if(!run||run.id!==id){
    cellReferenceVisible=false;
    selectedCellCandidate=null;
    selectedCellWaypoint=0;
  }
  run = loadedRun;
  $<HTMLSelectElement>('#run-select').value = id;
  try { localStorage.setItem('pen-art-selected-run', id); } catch { /* Storage may be disabled. */ }
  manifest = loadedManifest;
  method=defaultCurveMethod(run);
  structureVariant='backbone';
  $('#stage-nav').innerHTML = stages.map((stage, index) => `<button class="stage-button" data-stage="${stage.id}"><span class="stage-number mono">${String(index + 1).padStart(2, '0')}</span><span><span class="stage-label">${stage.name}</span><span class="stage-status">${statusText(run.stages[stage.id === 'curves' ? curveStageKey() : stage.id]?.status ?? 'pending')}</span></span></button>`).join('');
  document.querySelectorAll<HTMLButtonElement>('.stage-button[data-stage]').forEach(button => button.addEventListener('click', () => { selectStage(button.dataset.stage!).catch(showError); }));
  $('#scale-label').textContent = `Object: ${manifest.bounds_m[1].map((v,i)=>((v-manifest.bounds_m[0][i])*100).toFixed(1)).join(' × ')} cm`;
  await selectStage('asset');
  await loadModel(modelMode);
  if(request===runRequest && (run.workbench_collection==='motion_reuse'||(run.workbench_primary&&run.artifacts.includes('offline_dual_arm_plan.json'))))await selectStage('motion');
}

document.querySelectorAll<HTMLButtonElement>('[data-mode]').forEach(button => button.addEventListener('click', () => {
  modelMode = button.dataset.mode!;
  document.querySelectorAll('[data-mode]').forEach(b => b.classList.toggle('selected', b === button));
  loadModel(modelMode).catch(showError);
}));
$('#run-select').addEventListener('change', () => loadRun($<HTMLSelectElement>('#run-select').value).catch(showError));
function displayedRuns() {
  return runScope==='all' ? allRuns : allRuns.filter(value=>runScope==='reuse'?value.workbench_collection==='motion_reuse':value.workbench_primary&&value.workbench_collection!=='motion_reuse');
}
function renderRunChoices(selected?:string, includeOpened=false) {
  const currentCount=allRuns.filter(value=>value.workbench_primary&&value.workbench_collection!=='motion_reuse').length;
  const reuseCount=allRuns.filter(value=>value.workbench_collection==='motion_reuse').length;
  $<HTMLSelectElement>('#run-scope').innerHTML=`${currentCount?`<option value="current">Current builds (${currentCount})</option>`:''}${reuseCount?`<option value="reuse">Motion cache (${reuseCount})</option>`:''}<option value="all">All runs (${allRuns.length})</option>`;
  $<HTMLSelectElement>('#run-scope').value=runScope;
  const visible=displayedRuns();
  const opened=includeOpened && runScope==='current' ? allRuns.find(value=>value.id===selected && !value.workbench_primary) : undefined;
  $<HTMLSelectElement>('#run-select').innerHTML=visible.map(value=>`<option value="${escape(value.id)}">${escape(runLabel(value))}</option>`).join('')
    +(opened?`<optgroup label="Opened older run"><option value="${escape(opened.id)}">${escape(runLabel(opened))}</option></optgroup>`:'');
  if(selected && (opened || visible.some(value=>value.id===selected)))$<HTMLSelectElement>('#run-select').value=selected;
}
$('#run-scope').addEventListener('change',()=>{
  runScope=$<HTMLSelectElement>('#run-scope').value as 'current'|'reuse'|'all';
  const selected=run?.id;
  renderRunChoices(selected);
  const next=$<HTMLSelectElement>('#run-select').value;
  if(next && next!==selected)loadRun(next).catch(showError);
});
async function start() {
  allRuns = (await json<Run[]>('/api/runs')).filter(r => r.stages.asset.status === 'complete').sort((a,b) => Number(!!b.workbench_primary)-Number(!!a.workbench_primary) || b.created_at.localeCompare(a.created_at));
  if (!allRuns.length) { renderRunChoices();empty('No model runs yet', 'Run the asset pipeline to create the first model.'); return; }
  let savedId: string | null = null;
  try { savedId = localStorage.getItem('pen-art-selected-run'); } catch { /* Storage may be disabled. */ }
  const requested=new URLSearchParams(location.search);
  const explicit=allRuns.find(value=>value.id===requested.get('run'));
  if(!allRuns.some(value=>value.workbench_primary))runScope='all';
  if(explicit?.workbench_collection==='motion_reuse')runScope='reuse';
  const initial=explicit ?? displayedRuns().find(value=>value.id===savedId) ?? displayedRuns()[0];
  renderRunChoices(initial.id,true);
  await loadRun(initial.id);
  const requestedStage=requested.get('stage');
  if(requestedStage && requestedStage!==activeStage && stages.some(stage=>stage.id===requestedStage))await selectStage(requestedStage);
  setInterval(async () => {
    if (!run || stageLoads) return;
    const id = run.id;
    try {
      const latest = await json<Run>(`/api/runs/${encodeURIComponent(id)}`);
      if (run.id !== id || (latest.updated_at === run.updated_at && latest.artifact_revision === run.artifact_revision)) return;
      // Keep the user's current playback intact; consume the update after it stops.
      if(playback || stageLoads) return;
      const key = activeStage === 'curves' ? curveStageKey() : activeStage;
      const liveCellUpdate=['cell','motion','timing','validation'].includes(activeStage)&&latest.artifact_revision!==run.artifact_revision;
      const refreshStage = liveCellUpdate || JSON.stringify(latest.stages[key]) !== JSON.stringify(run.stages[key]) ||
        (activeStage === 'curves' && JSON.stringify(latest.stages.comparison) !== JSON.stringify(run.stages.comparison)) ||
        (['curves','cell','motion','timing','validation'].includes(activeStage) && JSON.stringify(latest.artifacts) !== JSON.stringify(run.artifacts)) ||
        (['cell','motion','timing','validation'].includes(activeStage) && JSON.stringify(latest.stages.validation)!==JSON.stringify(run.stages.validation));
      run = latest;
      document.querySelectorAll<HTMLButtonElement>('.stage-button[data-stage]').forEach(button => {
        const key = button.dataset.stage === 'curves' ? curveStageKey() : button.dataset.stage!;
        button.querySelector('.stage-status')!.textContent = statusText(run.stages[key]?.status ?? 'pending');
      });
      if (activeStage !== 'asset' && refreshStage) await selectStage(activeStage);
    } catch (error) { showError(error); }
  }, 15000);
}
start().catch(showError);
