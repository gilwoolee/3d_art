"""Profile unchanged Palace geometry with the current continuous checker."""
import cProfile
import io
import json
import pstats
from pathlib import Path
from time import monotonic

import numpy as np
from pen_art.io import write_json, sha256
from pen_art.planning.cell_planner import tracked_drawing_path
from pen_art.planning.section_build import SectionBuild

run = Path('runs/palace_hybrid_backward_diverse_v1')
build = SectionBuild(run, Path('research/planning_cell/cell.yaml'), 120)
key = 'section-01210'
state = build.state(set(build.actions) - {key}, build.initial)
repair_path = run / 'axis_repairs' / f'arm_1_{key}.json'
repair = json.loads(repair_path.read_text())
drawing = {'stroke_id': key, 'positions': repair['positions'],
           'tcp_targets_m': repair['tcp_targets_m'], 'tcp_rotation': np.eye(3).tolist(),
           'free_tool_orientation': True}
profile = cProfile.Profile()
profile.enable()
tracked = tracked_drawing_path(build.arm, drawing, True, build.planner.deadline)
assert tracked['status'] == 'BOUNDED', tracked
results = []
scene = state.payload.copy()
for index, piece in enumerate(tracked['pieces']):
    build.planner.add_piece(scene, build.arm.identity, piece,
                            build.actions[key]['diameter_m'] / 2, f'probe/{index}')
    started = monotonic()
    result = build.arm.collision.check_bounded_path(
        piece.position, piece.deviation, build.planner.obstacles, build.planner.margin,
        build.planner.settings['maximum_subdivision_depth'], scene, build.planner.deadline)
    results.append({'segment': index, 'seconds': monotonic() - started, **result})
    print(results[-1], flush=True)
    if result['status'] != 'CERTIFIED_MODELED_EDGE': break
profile.disable()
profile.dump_stats('runs/palace_collision_profile.prof')
output = io.StringIO()
pstats.Stats(profile, stream=output).sort_stats('cumtime').print_stats(30)
Path('runs/palace_collision_profile.txt').write_text(output.getvalue())
write_json(Path('research/blondie/palace_collision_profile.json'), {
    'status': 'DIAGNOSTIC_ONLY', 'source_run': str(run), 'section': key,
    'material_capsules': len(state.payload.objects), 'results': results,
    'inputs': {str(repair_path): sha256(repair_path),
               'src/pen_art/planning/collision.py': sha256(Path('src/pen_art/planning/collision.py'))}})
print(output.getvalue(), flush=True)
