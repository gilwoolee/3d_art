from pathlib import Path
import shutil
from time import monotonic
import numpy as np
from pen_art.io import read_json,write_json,sha256
from pen_art.planning.section_build import SectionBuild
from pen_art.planning.scene import MaterialScene
from pen_art.planning.constant_axis import constant_axis_candidates,constant_axis_path
from pen_art.planning.dual_arm import ScheduledMotion
source=Path('runs/palace_hybrid_spatial_v1');run=Path('runs/palace_constant_axis_probe_v1')
assert not run.exists()
shutil.copytree(source,run,ignore=shutil.ignore_patterns('axis_repairs','.*'))
build=SectionBuild(run,Path('research/planning_cell/cell.yaml'),600,gpu=True)
index=read_json(run/'forward_checkpoint.json');ids=set();q=build.initial;scene=MaterialScene()
for i,ref in enumerate(index['witnesses'][:9]):
 record=read_json(run/ref['file']);row=build.actions[record['section']]
 checked=build.checkpoint_proposal(i,row,build.state(ids,q,scene),q)
 assert checked is not None
 move,drawing,scene,_=checked;ids.add(row['id']);q=np.asarray(move.end_configuration)
row=build.actions['section-00005'];state=build.state(ids,q,scene)
drawing=next(build.drawings(row,state.ready_nodes(),q))
start=monotonic();rotations=constant_axis_candidates(build.planner,drawing,scene)
print('Clear pen rotations',len(rotations),'screen seconds',monotonic()-start,flush=True)
report={'status':'NO_ACCEPTED_CONSTANT_AXIS','source_run':str(source),'section':row['id'],'candidate_rotations':len(rotations),'attempts':[],'inputs':{str(source/p):sha256(source/p) for p in ['stroke_graph.json','construction_plan.json','cell_manifest.json']}}
for i,rotation in enumerate(rotations):
 if monotonic()>=build.planner.deadline:break
 started=monotonic();proposal=constant_axis_path(build.planner,drawing,scene,rotation)
 result={'index':i,'seconds':monotonic()-started,'rotation':rotation.tolist(),'sampled_path':proposal is not None}
 print('Candidate',i,'sampled path',proposal is not None,flush=True)
 if proposal:
  repaired={**drawing,**proposal}
  move=build.planner.prepare_drawing(state,build.arm.identity,repaired,q)
  result['prepared']=move is not None
  if move:
   after,failure=build.planner.validate_interval(state,(ScheduledMotion(move,0.),),move.duration_s)
   result['replay_failure']=failure
   if not failure:
    report.update(status='ACCEPTED_CONSTANT_AXIS_SECTION',proposal=repaired,trajectory=move.payload.record(),duration_s=move.duration_s)
 if build.planner.attempts:result['last_failure']={k:build.planner.attempts[-1].get(k) for k in ['phase','status']}
 report['attempts'].append(result);write_json(run/'comparison.json',report)
 if report['status']=='ACCEPTED_CONSTANT_AXIS_SECTION':break
report['seconds']=monotonic()-start;write_json(run/'comparison.json',report)
print(report['status'],flush=True)
