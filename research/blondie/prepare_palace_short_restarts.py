from pathlib import Path
import shutil
from copy import deepcopy
from pen_art.io import read_json,write_json,sha256,now
from pen_art.planning.sections import section_actions,split_member_for_restart
from pen_art.planning.section_build import SectionBuild,prepare_candidates,write_section_artifacts
from pen_art.provenance import audit_source_provenance
source=Path('runs/palace_hybrid_spatial_v1');run=Path('runs/palace_hybrid_short_restarts_v1');assert not run.exists()
original=read_json(source/'stroke_graph.json');graph=deepcopy(original)
old_actions=read_json(source/'construction_plan.json')['actions'];nodes={n['id']:n for n in original['nodes']}
cuts=[]
for key in ['section-00005','section-00006']:
 old=next(r for r in old_actions if r['id']==key)
 reverse=nodes[old['start']]['position'][2]>nodes[old['end']]['position'][2]
 fractions=sorted(1-f if reverse else f for f in [.125,.25,.375])
 for fraction in fractions:
  target=old['source_interval'][0]+fraction*(old['source_interval'][1]-old['source_interval'][0])
  candidates=[e for e in graph['edges'] if e.get('parent_curve')==old['parent_curve'] and e['source_interval'][0]+1e-10<target<e['source_interval'][1]-1e-10]
  if candidates:
   assert len(candidates)==1
   member=candidates[0];lo,hi=member['source_interval']
   graph=split_member_for_restart(graph,member['id'],(target-lo)/(hi-lo))
  else:
   at_boundary=[e for e in graph['edges'] if e.get('parent_curve')==old['parent_curve'] and abs(e['source_interval'][1]-target)<1e-10]
   assert len(at_boundary)==1
   at_boundary[0]['stop_after']=True
 cuts.append({'source_section':key,'fractions':fractions,'preferred_reverse':reverse})
construction=section_actions(graph,group_sources=True)
order=sorted(old_actions,key=lambda r:(r['stage']!='framework',r['priority'],min(nodes[r[e]]['position'][2] for e in ('start','end')),r['id']))
seen=set();chosen=[]
for rank,old in enumerate(order):
 lo,hi=old['source_interval'];reverse=nodes[old['start']]['position'][2]>nodes[old['end']]['position'][2]
 pieces=sorted([r for r in construction['actions'] if r['parent_curve']==old['parent_curve'] and lo-1e-10<=r['source_interval'][0] and r['source_interval'][1]<=hi+1e-10],key=lambda r:r['source_interval'][0],reverse=reverse)
 assert pieces,old['id']
 for i,row in enumerate(pieces):
  assert id(row) not in seen;seen.add(id(row))
  row['id']=old['id'] if len(pieces)==1 else old['id']+f'-part-{i}'
  row['priority']=rank+i/len(pieces);chosen.append(row['id'])
assert len(seen)==len(construction['actions']) and len(set(chosen))==len(chosen)
construction['proposed_order_note']='Short exact restarts near two observed posture transitions; preserve unsplit source order and preferred upward direction.'
audit=audit_source_provenance(construction['actions'],graph)
shutil.copytree(source,run,ignore=shutil.ignore_patterns('.*','axis_repairs'))
metadata=read_json(run/'run.json');metadata.update(id=run.name,updated_at=now());write_json(run/'run.json',metadata)
old_index=read_json(run/'forward_checkpoint.json');(run/'forward_checkpoint.json').rename(run/'imported_forward_checkpoint.json')
write_json(run/'construction_plan.json',construction)
write_json(run/'section_build.json',{'status':'PREPARED_PROPOSALS_PENDING_REVALIDATION','completed_sections':0,'required_sections':len(chosen),'best_prefix':[]})
write_section_artifacts(run,graph,construction);prepare_candidates(run)
build=SectionBuild(run,Path('research/planning_cell/cell.yaml'),7200,gpu=False)
assert build.stable(build.actions)
first_ids=set(chosen[:9]);members={m for k in first_ids for m in build.actions[k]['members']}
checks=[]
for key in chosen[9:17]:
 members.update(build.actions[key]['members'])
 from pen_art.structure import analyze_structure
 r=analyze_structure(graph,build.material,edge_ids=members)
 checks.append({'section':key,'modeled_pass':r['modeled_pass'],'maximum_displacement_m':r.get('maximum_displacement_m')})
 assert r['modeled_pass'],checks[-1]
write_json(run/'restart_preflight.json',{'status':'FINAL_AND_EARLY_RELEASES_PASSED','final':build.structure[frozenset(build.actions)],'early_releases':checks})
write_json(run/'forward_checkpoint.json',{'status':'PROPOSALS_REQUIRE_REVALIDATION','binding':build.checkpoint_binding,'witnesses':old_index['witnesses'][:9],'imported_from':str(source),'note':'Only the unchanged preceding 9 motions are proposed; all are revalidated.'})
write_json(run/'trial_reason.json',{'at':now(),'source_run':str(source),'cuts':cuts,'new_restart_nodes':len(graph['nodes'])-len(original['nodes']),'new_ground_anchors':0,'new_support_material':False,'source_provenance':audit,'sections':len(chosen),'order':chosen,'inputs':{str(source/p):sha256(source/p) for p in ['stroke_graph.json','construction_plan.json']},'outputs':{str(run/p):sha256(run/p) for p in ['stroke_graph.json','construction_plan.json','restart_preflight.json']}})
print('Prepared',run,len(chosen),'sections; exact cuts and early structural releases pass',flush=True)
