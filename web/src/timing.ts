export interface TimedOperation {
  id?:string; kind:string; stroke_id:string|null; start_s:number; end_s:number;
  duration_s:number; unknown_duration?:boolean; extrusion_expected:boolean;
  hold_positions?:number[]; times_s?:number[]; positions?:number[][];
  tcp_path_m?:number[][]; tcp_speeds_m_s?:number[];
  geometric_path?:{knots:number[];coefficients:number[][][]};
  path_clock?:{s:number[];t_s:number[];speeds:number[]};
}
export interface TimedSchedule {
  status:string; executable:false; modeled_duration_s:number; within_modeled_budget?:boolean;
  unknown_durations:string[]; operations:TimedOperation[]; flange_to_tcp:number[][];
  events:{time_s:number;command:string;stroke_id:string}[];
  timing_conflicts:{code:string;stroke_id:string;message:string}[];
  geometry_validation:{status:string}; profile:{target_seconds:number;calibrated:boolean};
}

export function interval(values:number[], value:number):number {
  let lo=0, hi=values.length-1;
  while(lo+1<hi){const middle=Math.floor((lo+hi)/2);if(values[middle]<=value)lo=middle;else hi=middle;}
  return Math.min(values.length-2,lo);
}

export function timedPose(operation:TimedOperation, jobTime:number):number[] {
  if(operation.hold_positions)return operation.hold_positions;
  const clock=operation.path_clock!, path=operation.geometric_path!;
  const t=Math.max(0,Math.min(operation.duration_s,jobTime-operation.start_s));
  const clockIndex=interval(clock.t_s,t), dt=t-clock.t_s[clockIndex];
  const acceleration=(clock.speeds[clockIndex+1]-clock.speeds[clockIndex])/(clock.t_s[clockIndex+1]-clock.t_s[clockIndex]);
  const s=Math.max(path.knots[0],Math.min(path.knots.at(-1)!,clock.s[clockIndex]+clock.speeds[clockIndex]*dt+acceleration*dt*dt/2));
  const i=interval(path.knots,s), u=s-path.knots[i];
  return path.coefficients[0][i].map((_,j)=>((path.coefficients[0][i][j]*u+path.coefficients[1][i][j])*u+path.coefficients[2][i][j])*u+path.coefficients[3][i][j]);
}

export interface CoordinatedEdge {
  start_s:number;end_s:number;q0:number[];q1:number[];kind:string;stroke_id:string|null;
  tcp0_m?:number[];tcp1_m?:number[];
  coefficients?:number[][];
}
export interface CoordinatedAction {
  arm:string;stroke_id:string;variant:string;start_s:number;end_s:number;
  auxiliary?:boolean;
  trajectory:{clock:string;duration_s:number;edges:CoordinatedEdge[];
    tracking_position_bound_m?:number;tracking_orientation_bound_rad?:number;
    tracking_orientation_constraint?:'fixed'|'free';axis_repair_artifact?:string;
    tracking_subdivisions?:number};
}
export interface CoordinatedPlan {
  resource_mode?:'single'|'dual';
  active_arms?:string[];
  status:string;completed_strokes:number;requested_strokes:number;modeled_duration_s:number;
  checked_intervals:number;actions:CoordinatedAction[];
  file_name?:string;
  full_schedule?:boolean;
  trajectory_export_available?:boolean;
  initial_positions?:Record<string,number[]>;initial_pose_repairs?:{precondition?:string};
  failures:{arm:string;stroke_id:string;phase:string;status:string}[];
  search_failures?:{phase:string;time_s:number;status:string;arm?:string}[];
  limitations:string[];
  events?:{time_s:number;arm:string;stroke_id:string;command:string}[];
  replay_in_progress?:boolean;
  replay?:{status:string;completed_strokes?:number;checked_intervals?:number;source_plan_sha256:string;
    phase?:string;checked_actions?:number;requested_actions?:number;
    checked_edges?:number;requested_edges?:number;active_action?:{arm:string;stroke_id:string};
    reused_path_checks?:number;resumed_from_s?:number;
    failures?:{status:string;phase?:string;arm?:string;time_s?:number;positions?:number[]|number[][];pairs?:{first:string;second:string}[]}[]};
  comparison?:{
    status:string;speedup:number|null;
    results:Record<string,{
      completed_strokes:number;requested_strokes:number;modeled_duration_s:number|null;
      validated_prefix_s:number;concurrent_drawing_s:number;replay_status:string;
    }>;
  };
}

export interface TimelinePhase {
  kind:string; stroke_id:string|null; start_s:number; end_s:number;
}

export function coordinatedTimeline(plan:CoordinatedPlan,arm:string):TimelinePhase[] {
  // Collapse numerical subdivisions only inside the same action. Playback still
  // evaluates every original polynomial; this is a display-only representation.
  const phases:TimelinePhase[]=[];
  let cursor=0;
  for(const action of plan.actions.filter(a=>a.arm===arm).sort((a,b)=>a.start_s-b.start_s)){
    const local:TimelinePhase[]=[];
    for(const edge of action.trajectory.edges){
      const start=Math.max(0,action.start_s+edge.start_s);
      const end=Math.min(plan.modeled_duration_s,action.start_s+edge.end_s);
      if(end<=start)continue;
      const previous=local.at(-1);
      if(previous?.kind===edge.kind&&Math.abs(previous.end_s-start)<1e-9)previous.end_s=end;
      else local.push({kind:edge.kind,stroke_id:action.stroke_id,start_s:start,end_s:end});
    }
    for(const phase of local){
      if(phase.start_s>cursor+1e-9)phases.push({kind:'WAIT',stroke_id:null,start_s:cursor,end_s:phase.start_s});
      phases.push(phase);
      cursor=Math.max(cursor,phase.end_s);
    }
  }
  if(cursor<plan.modeled_duration_s)phases.push({kind:'WAIT',stroke_id:null,start_s:cursor,end_s:plan.modeled_duration_s});
  return phases;
}

export function replayProgressLabel(plan:Pick<CoordinatedPlan,'replay'|'replay_in_progress'>):string {
  const replay=plan.replay;
  if(replay?.status==='RUNNING'){
    return replay.phase==='PATH_CHECKS'
      ? `Checking motion limits · ${replay.checked_actions??0}/${replay.requested_actions??0} actions · ${replay.checked_edges??0}/${replay.requested_edges??0} intervals${replay.reused_path_checks?` · ${replay.reused_path_checks} unchanged paths reused`:''}`
      : 'Checking both arms against the growing sculpture'+(replay.resumed_from_s!==undefined?` · resumed at ${(replay.resumed_from_s/60).toFixed(1)} min`:'');
  }
  if(plan.replay_in_progress)return 'Running'+(replay?`; previous result: ${replay.status.replaceAll('_',' ').toLowerCase()}`:'');
  if(replay?.status==='REPLAY_TIME_BUDGET')return 'Compute budget reached · verification incomplete';
  return replay?.status.replaceAll('_',' ').toLowerCase()??'Not checked';
}

export function coordinatedPose(action:CoordinatedAction,time:number):number[] {
  const relative=Math.max(0,Math.min(action.trajectory.duration_s,time-action.start_s));
  const edges=action.trajectory.edges;
  let low=0,high=edges.length-1;
  while(low<high){const mid=Math.floor((low+high)/2);if(edges[mid].end_s<=relative)low=mid+1;else high=mid;}
  const edge=edges[low];
  const u=Math.max(0,Math.min(1,(relative-edge.start_s)/(edge.end_s-edge.start_s)));
  if(edge.coefficients)return edge.coefficients.map(c=>c.reduceRight((sum,value)=>sum*u+value,0));
  const h=u*u*u*(10+u*(-15+6*u));
  return edge.q0.map((value,i)=>value+(edge.q1[i]-value)*h);
}
