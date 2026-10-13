from pathlib import Path
import shutil
from pen_art.io import read_json,write_json,sha256,now
from pen_art.planning.section_build import SectionBuild,prepare_candidates,write_section_artifacts
from pen_art.provenance import audit_source_provenance
source=Path('runs/palace_hybrid_restart_v1');original=Path('runs/palace_hybrid_spatial_v1');run=Path('runs/palace_hybrid_restart_order_v1')
assert not run.exists()
construction=read_json(source/'construction_plan.json');base=read_json(original/'construction_plan.json')['actions'];graph=read_json(source/'stroke_graph.json')
nodes={n['id']:n for n in graph['nodes']}
order=sorted(base,key=lambda r:(r['stage']!='framework',r['priority'],min(nodes[r[end]]['position'][2] for end in ('start','end')),r['id']))
ordered=[];seen=set()
for rank,old in enumerate(order):
 lo,hi=old['source_interval']
 pieces=sorted([r for r in construction['actions'] if r['parent_curve']==old['parent_curve'] and lo-1e-10<=r['source_interval'][0] and r['source_interval'][1]<=hi+1e-10],key=lambda r:r['source_interval'][0])
 assert pieces,old['id']
 for part,row in enumerate(pieces):
  assert row['id'] not in seen
  row['priority']=rank+part/len(pieces)
  seen.add(row['id']);ordered.append(row['id'])
assert len(seen)==len(construction['actions'])
assert ordered.index('section-00006')==ordered.index('section-00005')+1
construction['proposed_order_note']='Preserve unsplit framework order, keeping the two restart pieces adjacent; remaining source curves and base-first stages unchanged.'
audit=audit_source_provenance(construction['actions'],graph)
shutil.copytree(source,run,ignore=shutil.ignore_patterns('.*'))
metadata=read_json(run/'run.json');metadata.update(id=run.name,updated_at=now());write_json(run/'run.json',metadata)
old_index=read_json(run/'forward_checkpoint.json');(run/'forward_checkpoint.json').rename(run/'imported_forward_checkpoint.json')
write_json(run/'construction_plan.json',construction)
write_json(run/'section_build.json',{'status':'PREPARED_PROPOSALS_PENDING_REVALIDATION','completed_sections':0,'required_sections':len(construction['actions']),'best_prefix':[],'simulation_only':True})
write_section_artifacts(run,graph,construction,preserve_graph=True);prepare_candidates(run)
build=SectionBuild(run,Path('research/planning_cell/cell.yaml'),7200,gpu=False)
write_json(run/'forward_checkpoint.json',{'status':'PROPOSALS_REQUIRE_REVALIDATION','binding':build.checkpoint_binding,'witnesses':old_index['witnesses'],'imported_from':str(source),'note':'These are imported motion proposals only; every motion is freshly revalidated.'})
assert sha256(run/'stroke_graph.json')==sha256(source/'stroke_graph.json')
write_json(run/'trial_reason.json',{'at':now(),'source':str(source),'unchanged_graph_sha256':sha256(run/'stroke_graph.json'),'order':ordered,'split_pieces_adjacent':True,'source_provenance':audit,'original_checkpoint_sha256':sha256(run/'imported_forward_checkpoint.json'),'construction_sha256':sha256(run/'construction_plan.json'),'simulation_only':True})
print('Prepared',len(ordered),'sections with adjacent restart pieces and unchanged physical graph')
