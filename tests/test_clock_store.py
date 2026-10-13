import json

import numpy as np
import pytest

from pen_art.io import sha256
from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.clock_store import _digest, load_plan, plain, write_plan


def plan():
    edge = ClockEdge(0., 2., np.array([1., 2.]), np.array([1., 2.]), 'DRAW', 'a',
                     coefficients=[[1., 0., 0., 1e-25], [2., 0., 0., 0., 0., 0.]])
    return {'actions': [{'trajectory': ClockPath([edge]).record(), 'start_s': 11.}],
            'events': [{'time_s': 11.}], 'inputs': {}, 'status': 'test'}


def test_lossless_mapped_roundtrip(tmp_path):
    source = plan()
    path = tmp_path / 'plan.json'
    write_plan(path, source)
    packed, _ = load_plan(path)
    assert plain(packed) == json.loads(path.read_text())
    assert {key: value for key, value in plain(packed).items()
            if key != 'packed_clock_binding'} == source
    clock = ClockPath.from_record(packed['actions'][0]['trajectory'])
    assert clock.record() == source['actions'][0]['trajectory']
    assert not clock.edges[0].q0.flags.writeable
    assert isinstance(packed['actions'][0]['trajectory']['edges'].values, np.memmap)


def test_present_corrupt_or_stale_sidecar_fails_closed(tmp_path):
    for target in ('json', 'manifest', 'array', 'manifest_rehashed', 'array_rehashed',
                   'legacy_rehashed', 'missing_anchor', 'invalid_table'):
        case = tmp_path / target
        case.mkdir()
        path = case / 'plan.json'
        source = plan()
        if target == 'invalid_table':
            source['actions'][0]['trajectory']['edges'][0]['table_index'] = 0.5
            with pytest.raises(ValueError):
                write_plan(path, source)
            continue
        write_plan(path, source)
        manifest = path.with_suffix('.clocks.json')
        row = json.loads(manifest.read_text())
        if target == 'json':
            path.write_text(path.read_text() + ' ')
        elif target in ('manifest', 'manifest_rehashed', 'legacy_rehashed'):
            row['actions'][0]['start_s'] += 1
        elif target in ('array', 'array_rehashed'):
            data = next(case.glob('*.npy'))
            values = np.load(data)
            values[0] += 0.01
            with data.open('wb') as stream:
                np.save(stream, values, allow_pickle=False)
            if target == 'array_rehashed':
                row['data_sha256'] = sha256(data)
        if target in ('legacy_rehashed', 'missing_anchor'):
            path.write_text(json.dumps(source))
            row['plan_sha256'] = sha256(path)
            if target == 'legacy_rehashed':
                row['version'] = 1
        if target in ('manifest_rehashed', 'array_rehashed', 'legacy_rehashed', 'missing_anchor'):
            row['checksum'] = _digest({k: v for k, v in row.items() if k != 'checksum'})
        manifest.write_text(json.dumps(row))
        with pytest.raises(ValueError):
            load_plan(path)
