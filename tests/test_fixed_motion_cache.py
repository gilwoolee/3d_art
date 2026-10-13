"""Fixed TCP orientation is an actual continuous admission contract."""


from types import SimpleNamespace

import numpy as np
import pytest

from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.motion_library import DrawingRequest, MotionLibrary


def arm():
    def forward(q, offset):
        c, s = np.cos(q[1]), np.sin(q[1])
        pose = np.eye(4)
        pose[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        pose[0, 3] = q[0]
        return pose

    return SimpleNamespace(
        robot=SimpleNamespace(forward=forward, joint_data=[]), flange_to_tcp=np.eye(4)
    )


def drawing():
    return {
        "positions": [[0, 0], [0.01, 0]],
        "tcp_targets_m": [[0, 0, 0], [0.01, 0, 0]],
        "tcp_rotation": np.eye(3).tolist(),
        "free_tool_orientation": False,
    }


def test_fixed_cache_rejects_interior_excursion_despite_matching_endpoint_rotation(tmp_path):
    model, path = arm(), drawing()
    request = DrawingRequest(path["tcp_targets_m"], context_hash="fixed")
    # R(0) == R(1), but the middle rotates .01 rad, above the .003 tolerance.
    bad = ClockPath(
        [
            ClockEdge(
                0,
                1,
                np.array([0, 0]),
                np.array([0.01, 0]),
                "DRAW",
                "line",
                coefficients=[[0, 0, 0, 0.1, -0.15, 0.06], [0, 0, 0, 0.64, -1.92, 1.92, -0.64]],
            )
        ]
    )
    library = MotionLibrary(tmp_path)
    with pytest.raises(ValueError, match="orientation excursion"):
        library.remember(
            request, path, {"status": "VERIFIED_VALID", "clock_record": bad.record()}, arm=model
        )
    assert not library.records
    with pytest.raises(ValueError, match="robot model"):
        library.remember(request, path, {"status": "VERIFIED_VALID"})
    with pytest.raises(ValueError, match="fixed TCP rotation"):
        library.remember(
            request,
            {**path, "free_tool_orientation": True},
            {"status": "VERIFIED_VALID"},
            arm=model,
        )
    varied = [np.eye(3).tolist(), [[0, -1, 0], [1, 0, 0], [0, 0, 1]], np.eye(3).tolist()]
    with pytest.raises(ValueError, match="Per-point"):
        library.remember(
            request, {**path, "tcp_rotations": varied}, {"status": "VERIFIED_VALID"}, arm=model
        )


def test_lazy_index_keeps_original_identity_and_rejects_changed_selection(tmp_path, monkeypatch):
    model, path = arm(), drawing()
    request = DrawingRequest(path['tcp_targets_m'], context_hash='fixed')
    writer = MotionLibrary(tmp_path)
    key = writer.remember(request, path, {'status': 'VERIFIED_VALID'},
                          scene={'capsules': [{'historical': True}]}, arm=model)
    eager = writer.retrieve(request)
    reader = MotionLibrary(tmp_path, lazy=True)
    assert '_lazy' in reader.records[0] and 'trajectory' not in reader.records[0]
    selected = reader.retrieve(request)
    assert selected[0]['trajectory'] == eager[0]['trajectory']
    assert reader.record_key(selected[0]) == key
    assert 'scene' not in selected[0]
    # A second library reads only the admitted index during initialization.
    with monkeypatch.context() as patch:
        patch.setattr(MotionLibrary, '_validate_record', lambda r: (_ for _ in ()).throw(
            AssertionError('initialization reparsed a selected clock')))
        indexed = MotionLibrary(tmp_path, lazy=True)
    assert len(indexed.records) == 1
    published = tmp_path / 'motions' / f'{key}.json'
    published.write_text(published.read_text() + ' ')
    assert not indexed.retrieve(request)
    assert not reader.retrieve(request)
