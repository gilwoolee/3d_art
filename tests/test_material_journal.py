import numpy as np
import pytest

from pen_art.planning.material_journal import load_journal, write_journal
from pen_art.planning.parallel_build import MaterialPrefix, deposit_key


def test_shared_journal_preserves_geometry_prefix_and_replacement(tmp_path):
    operations = [('add', [{'start': [0., 0., 0.], 'end': [0., 0., .01],
                           'radius_m': .0005, 'name': 'wire'}])]
    path = tmp_path / 'journal.json'
    write_journal(path, operations)
    packed = load_journal(path)
    assert deposit_key(packed) == deposit_key(operations)
    assert isinstance(packed[0][1].values, np.memmap)
    assert not packed[0][1].values.flags.writeable
    original = MaterialPrefix(None, lambda key: operations).scene_for([('first', 0)])
    mapped = MaterialPrefix(None, lambda key: packed)
    assert mapped.scene_for([('first', 0)]).geometry_prefix() == original.geometry_prefix()
    assert len(mapped.scene_for([]).objects) == 0
    assert mapped.scene_for([('replacement', 0)]).geometry_prefix() == original.geometry_prefix()
    with path.with_suffix('.npy').open('r+b') as stream:
        stream.seek(-1, 2)
        stream.write(b'x')
    with pytest.raises(ValueError, match='Corrupt'):
        load_journal(path)
    # Merged journals stand in for earlier actions' wire in later checks.
    _merged_deposits_enclose_every_original_capsule()


def _merged_deposits_enclose_every_original_capsule():
    import numpy as np

    from pen_art.planning.material_journal import merge_deposits

    rng = np.random.default_rng(7)

    def leg(start, end, count, wobble):
        points = np.linspace(start, end, count + 1)
        points[1:-1] += rng.normal(scale=wobble, size=(count - 1, 3))
        return [
            {"start": a.tolist(), "end": b.tolist(), "radius_m": 0.0005 + 1e-6 * (i % 3),
             "name": f"{start[0]:.3f}/{i}"}
            for i, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True))
        ]

    corner = np.array([0.013, 0.0, 0.005])
    first = leg(np.zeros(3), corner, 110, 2e-6)
    second = leg(corner, np.array([0.026, 0.0, 0.0]), 110, 2e-6)
    detached = leg(np.array([0.1, 0.0, 0.0]), np.array([0.11, 0.0, 0.0]), 5, 0.0)
    operations = [("add", first[:40]), ("add", first[40:] + second), ("turn", 0, 1),
                  ("add", detached)]
    budget = 20e-6
    merged = merge_deposits(operations, budget)
    assert [operation[0] for operation in merged] == ["add", "turn", "add"]
    # Two straight legs and one detached run become three capsules; the corner
    # and the turn each end a run.
    assert [len(operation[1]) for operation in merged if operation[0] == "add"] == [2, 1]
    originals = {0: first + second, 2: detached}
    for position, rows in originals.items():
        for row in rows:
            ends = np.array([row["start"], row["end"]])
            covered = False
            for capsule in merged[position][1]:
                a, b = np.array(capsule["start"]), np.array(capsule["end"])
                along = np.clip((ends - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)
                reach = np.linalg.norm(ends - (a + along[:, None] * (b - a)), axis=1).max()
                if reach + row["radius_m"] <= capsule["radius_m"] + 1e-15:
                    covered = True
                    assert capsule["radius_m"] <= 0.000502 + budget
            assert covered
    # A sharper wobble than the budget keeps more capsules, never fewer checks.
    rough = merge_deposits([("add", leg(np.zeros(3), corner, 110, 3e-5))], budget)
    assert len(rough[0][1]) > 10
