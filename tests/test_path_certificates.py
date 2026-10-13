from copy import deepcopy

import numpy as np

from pen_art.planning.path_certificates import SCOPE_STATUS, PathCertificateStore


def test_local_drawing_proofs_survive_travel_but_keep_scopes_and_changes(tmp_path):
    from pen_art.planning.clock_path import ClockEdge, ClockPath

    draw = ClockEdge(0., 2., np.array([0.]), np.array([1.]), 'DRAW', 'curve',
                     table_index=0)
    local = ClockPath([draw]).record()
    full = deepcopy(local)
    full['edges'][0]['start_s'], full['edges'][0]['end_s'] = 8., 10.
    full['edges'].insert(0, ClockEdge(0., 8., np.array([2.]), np.array([0.]),
                                    'TRAVEL', None, table_index=0).record())
    full['duration_s'] = 10.
    store = PathCertificateStore(tmp_path)
    context = {'physical': 'bound robot, tool, source, speed, table'}
    for scope in ('source_coverage', 'drawing_orientation', 'drawing_speed'):
        store.admit(context, local, scope, {'status': SCOPE_STATUS[scope]})
        assert store.get(context, full, scope)
        changed = deepcopy(full)
        changed['edges'][-1]['q1'][0] += .01
        assert store.get(context, changed, scope) is None
        assert store.get({'physical': 'changed'}, full, scope) is None
    changed = deepcopy(full)
    changed['edges'][-1]['end_s'] += .01
    assert store.get(context, changed, 'drawing_speed') is None
    store.admit(context, local, 'invariant_path', {'status': 'CERTIFIED_MODELED_PATH'})
    assert store.get(context, full, 'invariant_path') is None
    assert store.get(context, full, 'pen_invariant_path') is None
