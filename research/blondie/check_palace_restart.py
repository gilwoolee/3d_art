from pathlib import Path
import numpy as np
from pen_art.curves import bezier_at
from pen_art.io import read_json, write_json, sha256
from pen_art.provenance import audit_source_provenance

run = Path('runs/palace_hybrid_restart_v1')
reason = read_json(run / 'trial_reason.json')
source = Path(reason['source_run'])
for name, digest in reason['inputs'].items():
    assert sha256(Path(name)) == digest, name
original = read_json(source / 'stroke_graph.json')
actual = read_json(run / 'stroke_graph.json')
member_id = reason['member']
old_edges = {row['id']: row for row in original['edges']}
new_edges = {row['id']: row for row in actual['edges']}
for key, row in old_edges.items():
    if key != member_id:
        assert new_edges[key] == row, key
original_nodes = {row['id']: row for row in original['nodes']}
actual_nodes = {row['id']: row for row in actual['nodes']}
assert all(actual_nodes[key] == row for key, row in original_nodes.items())
boundary = reason['boundary']
assert len(actual_nodes) == len(original_nodes) + 1
assert not actual_nodes[boundary['node']]['ground']
left, right = [new_edges[key] for key in boundary['pieces']]
old = old_edges[member_id]
fraction = boundary['fraction']
error = max(float(np.linalg.norm(bezier_at(piece['segments'][0], t) - bezier_at(old['segments'][0], start + t * span))) for piece, start, span in [(left, 0, fraction), (right, fraction, 1-fraction)] for t in np.linspace(0, 1, 101))
assert error < 1e-12
assert left['source_interval'][0] == old['source_interval'][0]
assert right['source_interval'][1] == old['source_interval'][1]
assert left['source_interval'][1] == right['source_interval'][0]
assert left['stop_after']
construction = read_json(run / 'construction_plan.json')
audit = audit_source_provenance(construction['actions'], actual)
preflight = read_json(run / 'restart_preflight.json')
assert preflight['status'] == 'READY_FOR_ORDER_SEARCH'
write_json(Path('research/blondie/palace_restart_split.json'), {
    'status': 'EXACT_SOURCE_SPLIT_VERIFIED', 'run': str(run),
    'unchanged_original_members': len(old_edges)-1,
    'original_members': len(old_edges), 'new_members': len(new_edges),
    'fraction': fraction, 'maximum_sampled_curve_difference_m': error,
    'added_ground_anchors': 0, 'added_support_material': False,
    'sections': len(construction['actions']), 'source_provenance': audit,
    'final_preflight': preflight['status'], 'robot_build_complete': False,
    'inputs': {str(p): sha256(p) for p in [run / 'trial_reason.json', source / 'stroke_graph.json', run / 'stroke_graph.json', run / 'construction_plan.json', run / 'restart_preflight.json', Path('src/pen_art/planning/sections.py')]}
})
print('Exact Palace split and unchanged remainder verified; robot build remains pending')
