from pathlib import Path
import numpy as np

from pen_art.curves import curve_coordinate_bounds
from pen_art.io import read_json, write_json, sha256, now
from pen_art.planning.build_order_preflight import preflight
from pen_art.provenance import audit_source_provenance

rows = []
for name in ['cube_sections_v1', 'prism_sections_v1', 'bottle_sections_v1',
             'torus_sections_v1', 'cube_wireprint_spatial_v1',
             'bottle_hybrid_v1', 'palace_hybrid_spatial_v1']:
    run = Path('runs') / name
    graph = read_json(run / 'stroke_graph.json')
    construction = read_json(run / 'construction_plan.json')
    result = preflight(run)
    bounds = [curve_coordinate_bounds(edge) for edge in graph['edges']]
    lower = np.min([a for a, b in bounds], axis=0)
    upper = np.max([b for a, b in bounds], axis=0)
    extents = upper - lower
    tube_lower = np.min([a - e['diameter_m']/2 for (a,b),e in zip(bounds,graph['edges'],strict=True)],axis=0)
    tube_upper = np.max([b + e['diameter_m']/2 for (a,b),e in zip(bounds,graph['edges'],strict=True)],axis=0)
    tube_extents = tube_upper - tube_lower
    assert np.all(tube_extents <= 0.15 + 1e-9), (name, tube_extents)
    assert result['status'] == 'READY_FOR_ORDER_SEARCH', (name, result['status'])
    result.update(run=str(run), centerline_extent_m=extents.tolist(), material_extent_bound_m=tube_extents.tolist(),
                  material_assumptions=read_json(run / 'fabrication_config.json')['structure'],
                  source_provenance=audit_source_provenance(construction['actions'],graph))
    output = Path('research/blondie/final_graph_preflights') / (name + '.json')
    write_json(output, result)
    rows.append({'run': str(run), 'status': result['status'], 'material_extent_bound_m':tube_extents.tolist(),
                 'report':str(output), 'report_sha256':sha256(output)})
    print(rows[-1], flush=True)
write_json(Path('research/blondie/final_graph_preflights.json'), {
    'status':'PASSED', 'at':now(), 'simulation_only':True,
    'scope':'Final graph structure, size and source coverage; construction prefixes and robot motion require their own validation',
    'runs':rows})
