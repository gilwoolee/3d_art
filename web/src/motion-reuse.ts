export interface ReuseStroke {
  section:string; parent_curve:string; origin:string; record_id:string;
  source_points_m:number[][]; length_mm:number; table_index:number;
  start_s:number; end_s:number; draw_start_s:number;
}
interface BenchmarkMode { verified:boolean; seconds:number; accepted_source:string|null; generated_count:number }
export interface MotionReuse {
  label:string; source_run:string; phase:number;
  runtime:{counts:Record<string,number>;clearance_hits:number;clearance_misses:number};
  strokes:ReuseStroke[];
  library_entries:{record_id:string;section:string;origin:string;length_mm:number;
    table_index:number;joint_samples:number;source_run:string;has_exact_clock:boolean}[];
  continuous_actions?:boolean;
  computation?:{seconds:number;planning_seconds?:number;export_replay_seconds?:number}|null;
  execution?:{modeled_duration_s:number;seconds_by_activity:Record<string,number>;
    drawing_actions:number;mean_drawing_speed_mm_s:number;draw_tcp_chord_length_m:number};
  benchmark?:{scope:string;objects:string[];stroke_families:string[];budget_s:number;training_records:number;
    summary:Record<string,{verified:number;requests:number;median_seconds:number}>;
    requests:{id:string;variation:string;scene_kind:string;modes:Record<string,BenchmarkMode>}[];
    failure_diagnostics:{recorded_seconds:number;planner_detail:{status:string;failed_sample:number};
      first_waypoint_diagnostic:{ik_solutions:number;clear_solutions:number}[]}[]}|null;
  notes:string[];
}

const escape=(value:unknown)=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]!));
const origin=(value:string)=>({cache:'Cached + adapted',planner:'Local planner',diffusion:'Diffusion + refined'}[value]??value);

export function renderMotionReuse(
  panel:HTMLElement,data:MotionReuse,inspect:(stroke:ReuseStroke)=>void,
) {
  panel.classList.remove('hidden');
  const benchmark=data.benchmark;
  const modeNames:Record<string,string>={teacher:'Local IK baseline',cache:'Cache + fallback',cache_diffusion:'Cache + diffusion + fallback'};
  const result=(value:BenchmarkMode)=>`${value.verified?'Pass':'No path'} · ${value.seconds.toFixed(3)} s`;
  const origins=data.strokes.reduce((counts,stroke)=>{
    counts[stroke.origin]=(counts[stroke.origin]??0)+1;return counts;
  },{} as Record<string,number>);
  panel.innerHTML=`<h2>${escape(data.label)}</h2>
    <p>${data.strokes.length} successful drawing actions in this verified ${data.continuous_actions?'continuous':'reference'} build:
    ${Object.entries(origins).map(([key,count])=>`${count} ${escape(origin(key).toLowerCase())}`).join(', ')}.
    Select a stroke to inspect its drawing start and stored source curve, then use the playback controls above.</p>
    <div class="reuse-stats"><span><strong>${data.library_entries.length}</strong> stored motion records</span>
    <span><strong>${data.runtime.clearance_hits.toLocaleString()}</strong> self/static bound reuses</span>
    <span><strong>${data.runtime.clearance_misses.toLocaleString()}</strong> fresh self/static queries</span></div>
    <p>Record count includes variants saved across the first and repeated builds. A bound reuse is a collision-query event, not a separate trajectory. Current printed material is checked again.</p>
    <div class="comparison-table-wrap"><table id="reuse-strokes"><thead><tr><th>Inspect drawing</th><th>Motion source</th><th>Length</th><th>Table</th></tr></thead><tbody>
    ${data.strokes.map((stroke,i)=>`<tr><td><button data-reuse-stroke="${i}">${escape(stroke.section)}</button><small>${escape(stroke.parent_curve)}</small></td><td>${escape(origin(stroke.origin))}</td><td>${stroke.length_mm.toFixed(1)} mm</td><td>${stroke.table_index*90}°</td></tr>`).join('')}
    </tbody></table></div><p id="reuse-selected" aria-live="polite">No stored stroke selected.</p>
    <details><summary>What is stored in the motion library?</summary>
    <p>Source curve, table pose, joint-path seeds, and the exact accepted clock, with model and validation context. ${data.notes.map(escape).join(' ')}</p>
    <div class="comparison-table-wrap"><table><thead><tr><th>Stroke / record</th><th>Created by</th><th>Length</th><th>Joint samples</th><th>Source build</th></tr></thead><tbody>
    ${data.library_entries.map(entry=>`<tr><td>${escape(entry.section)}<small>${escape(entry.record_id.slice(0,12))}</small></td><td>${escape(origin(entry.origin))}</td><td>${entry.length_mm.toFixed(1)} mm</td><td>${entry.joint_samples}${entry.has_exact_clock?' + exact clock':''}</td><td>${escape(entry.source_run.split('/').slice(-2).join('/'))}</td></tr>`).join('')}
    </tbody></table></div></details>
    ${data.computation?`<p>Planning and verification wall time: ${data.computation.seconds.toFixed(2)} s${data.computation.planning_seconds!==undefined?` (planning ${data.computation.planning_seconds.toFixed(2)} s; export/replay ${data.computation.export_replay_seconds?.toFixed(2)} s)`:''}.</p>`:''}
    ${data.execution?`<h2>Execution time</h2>
    <p>${(data.execution.modeled_duration_s/60).toFixed(1)} min modeled total ·
    ${(data.execution.draw_tcp_chord_length_m).toFixed(2)} m drawn ·
    ${data.execution.mean_drawing_speed_mm_s.toFixed(1)} mm/s average while drawing.</p>
    <table><thead><tr><th>Activity</th><th>Minutes</th></tr></thead><tbody>
    ${Object.entries(data.execution.seconds_by_activity).map(([kind,seconds])=>`<tr><td>${escape(kind.replaceAll('_',' '))}</td><td>${(seconds/60).toFixed(2)}</td></tr>`).join('')}
    </tbody></table><p>Modeled execution time is separate from planning computation. Physical extrusion speed is uncalibrated.</p>`:''}
    ${benchmark?`<h2>What “15 held-out results” means</h2>
    <p><strong>${escape(benchmark.scope)}.</strong> All come from ${benchmark.objects.length} test object
    (${benchmark.objects.map(escape).join(', ')}) and ${benchmark.stroke_families.length} source stroke family.
    Variants change length, placement, curvature, table index, arm posture, or obstacle scene.
    Cube and bottle supply training examples; prism is validation; torus is test.
    The benchmark starts with ${benchmark.training_records} verified training trajectories in its cache,
    separate from the ${data.library_entries.length}-record prism demonstration library above.</p>
    <div class="comparison-table-wrap"><table id="reuse-benchmark"><thead><tr><th>Method</th><th>Verified local requests</th><th>Median planning time</th></tr></thead><tbody>
    ${Object.entries(benchmark.summary).map(([key,row])=>`<tr><td>${escape(modeNames[key])}</td><td>${row.verified} / ${row.requests}</td><td>${row.median_seconds.toFixed(3)} s</td></tr>`).join('')}
    </tbody></table></div>
    <p><strong>The three baseline failures were not timeouts.</strong> With ${benchmark.budget_s} seconds available,
    they stopped after ${Math.min(...benchmark.failure_diagnostics.map(row=>row.recorded_seconds)).toFixed(3)}–${Math.max(...benchmark.failure_diagnostics.map(row=>row.recorded_seconds)).toFixed(3)} seconds at the first waypoint.
    Each of the four IK candidates collided with the synthetic arm obstacle. The small local search stopped;
    cached paths supplied a different posture that passed continuous validation.</p>
    <p>This compares a bounded local IK helper, not an exhaustive run of the full construction planner.
    All 15 hybrid successes came from the cache: diffusion was never called on these test requests.
    The local baseline was faster on easy requests; no general speedup or added diffusion benefit is established.</p>
    <details><summary>Inspect the 15 request-level results</summary><div class="comparison-table-wrap"><table><thead><tr><th>Variant / scene</th><th>Local IK baseline</th><th>Cache</th><th>Cache + diffusion</th></tr></thead><tbody>
    ${benchmark.requests.map(row=>`<tr><td>${escape(row.variation)}<small>${escape(row.scene_kind)} · ${escape(row.id.slice(0,8))}</small></td>${['teacher','cache','cache_diffusion'].map(mode=>`<td>${result(row.modes[mode])}<small>${escape(row.modes[mode].accepted_source??'No accepted candidate')}</small></td>`).join('')}</tr>`).join('')}
    </tbody></table></div></details>`:''}`;
  panel.querySelectorAll<HTMLButtonElement>('[data-reuse-stroke]').forEach(button=>{
    button.addEventListener('click',()=>{
      const stroke=data.strokes[Number(button.dataset.reuseStroke)];
      panel.querySelectorAll('#reuse-strokes tr').forEach(row=>row.classList.remove('selected'));
      button.closest('tr')!.classList.add('selected');
      panel.querySelector('#reuse-selected')!.textContent=`${stroke.section}: ${origin(stroke.origin)}. Drawing starts at ${stroke.draw_start_s.toFixed(2)} s. Green shows its stored source curve; use Play full one-arm build to continue.`;
      inspect(stroke);
    });
  });
}
