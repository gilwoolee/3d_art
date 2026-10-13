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
