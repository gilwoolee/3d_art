"""Journals of deposited wire: shared read-only capsule arrays, their keys and
the material they rebuild. Each worker constructs its own FCL objects."""

import hashlib
import os
from collections import OrderedDict
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from ..io import read_json, sha256, write_json
from .scene import MaterialScene


class CapsuleRows(Sequence):
    def __init__(self, values, names):
        self.values, self.names = values, names

    def __len__(self):
        return len(self.names)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        row = self.values[index]
        return {"start": row[:3], "end": row[3:6], "radius_m": float(row[6]),
                "name": self.names[index]}


def write_journal(path, operations):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays, descriptors, offset = [], [], 0
    for _, rows in operations:
        values = np.asarray([[*r['start'], *r['end'], r['radius_m']] for r in rows],
                            dtype='<f8').reshape(-1, 7)
        arrays.append(values)
        descriptors.append(['add', offset, len(values), [r['name'] for r in rows]])
        offset += len(values)
    data_path = path.with_suffix('.npy')
    # Two workers can journal identical deposits (strokes that lay nothing new
    # share one key) at the same moment; each writes its own temporary file.
    temporary = data_path.with_suffix(f'.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        np.save(stream, np.concatenate(arrays) if arrays else np.empty((0, 7)), allow_pickle=False)
    temporary.replace(data_path)
    write_json(path, {'data_sha256': sha256(data_path), 'operations': descriptors}, compact=True)


def load_journal(path):
    path = Path(path)
    manifest = read_json(path)
    data_path = path.with_suffix('.npy')
    if sha256(data_path) != manifest['data_sha256']:
        raise ValueError('Corrupt capsule journal')
    values = np.load(data_path, mmap_mode='r', allow_pickle=False)
    if values.dtype != np.float64 or values.ndim != 2 or values.shape[1] != 7:
        raise ValueError('Malformed capsule array')
    operations, end = [], 0
    for entry in manifest['operations']:
        if entry[0] == 'add':
            _, offset, count, names = entry
            if offset != end or count != len(names) or offset + count > len(values):
                raise ValueError('Malformed capsule journal prefix')
            operations.append(('add', CapsuleRows(values[offset:offset + count], names)))
            end += count
        else:
            raise ValueError('Unknown material operation')
    if end != len(values) or not np.isfinite(values).all():
        raise ValueError('Incomplete or nonfinite capsule journal')
    return operations


class JournalFolder:
    """The deposit journals in one folder, as one process uses them: each kept
    once under its key, checked against it when read, the last four read held,
    and the material they name rebuilt by one ``MaterialPrefix``."""

    def __init__(self, folder):
        self.folder, self.cache = Path(folder), OrderedDict()
        self.prefix = MaterialPrefix(self.load)

    def keep(self, operations):
        """Write ``operations`` under their key unless already there; returns the key."""
        key = deposit_key(operations)
        path = self.folder / f"{key}.json"
        if not path.exists():
            write_journal(path, operations)
        return key

    def load(self, key):
        if key not in self.cache:
            operations = load_journal(self.folder / f"{key}.json")
            if deposit_key(operations) != key:
                raise ValueError("Changed material journal")
            self.cache[key] = operations
            while len(self.cache) > 4:
                self.cache.popitem(last=False)
        self.cache.move_to_end(key)
        return self.cache[key]


def _distance_to_segment(points, first, last):
    axis = last - first
    squared = float(axis @ axis)
    if squared < 1e-24:
        return np.linalg.norm(points - first, axis=1)
    along = np.clip((points - first) @ axis / squared, 0.0, 1.0)
    return np.linalg.norm(points - (first + along[:, None] * axis), axis=1)


def merge_deposits(operations, budget_m):
    """Replace runs of connected capsules by one capsule that contains them.

    A merged capsule spans a run's first start to its last end. Its radius is
    the run's largest radius plus the largest distance of any member endpoint
    from the new centre line; a capsule's centre line lies no further from that
    line than its endpoints do, so every member is enclosed. A run ends where
    this growth would exceed ``budget_m`` or at a gap. The result only ever
    over-covers the journaled material.
    """
    if not np.isfinite(budget_m) or budget_m <= 0:
        raise ValueError("Deposit merging needs a positive radius budget")
    merged, rows = [], []

    def flush():
        if not rows:
            return
        values = np.asarray([[*r["start"], *r["end"], r["radius_m"]] for r in rows], float)
        output, first = [], 0
        while first < len(values):
            last, radius = first, float(values[first, 6])
            while last + 1 < len(values):
                following = last + 1
                if np.linalg.norm(values[following, :3] - values[last, 3:6]) > 1e-9:
                    break
                members = values[first : following + 1]
                reach = _distance_to_segment(
                    members[:, :6].reshape(-1, 3), values[first, :3], values[following, 3:6]
                ).reshape(-1, 2).max(axis=1)
                if float(reach.max()) > budget_m:
                    break
                last, radius = following, float((members[:, 6] + reach).max())
            name = rows[first]["name"]
            output.append(
                {
                    "start": values[first, :3].tolist(),
                    "end": values[last, 3:6].tolist(),
                    "radius_m": radius,
                    "name": name if last == first else f"{name}+{last - first}",
                }
            )
            first = last + 1
        merged.append(("add", output))
        rows.clear()

    for _, added in operations:
        rows.extend(added)
    flush()
    return merged


def _merged(planner, operations):
    """Deposits as later actions will see them; an action's own wire stays fine."""
    budget = planner.settings.get("merge_deposits_m", 0.0)
    if not budget:
        return operations
    return merge_deposits(operations, budget)


def deposit_key(operations):
    """Bind deposited geometry and its order, independently of segment names."""
    state = hashlib.sha256(b"material-operations-v1")
    for _, rows in operations:
        state.update(b"add" + len(rows).to_bytes(8, "little"))
        state.update(
            np.asarray(
                [[*row["start"], *row["end"], row["radius_m"]] for row in rows], dtype="<f8"
            ).tobytes()
        )
    return state.hexdigest()


def deposit_difference(first, second):
    """Largest endpoint/radius difference between two deposit journals, or None."""
    first = [row for _, rows in first for row in rows]
    second = [row for _, rows in second for row in rows]
    if len(first) != len(second):
        return None
    if not first:
        return 0.0
    values = [
        np.asarray([[*row["start"], *row["end"], row["radius_m"]] for row in rows])
        for rows in (first, second)
    ]
    return float(np.max(np.abs(values[0] - values[1])))


class MaterialPrefix:
    """Rebuild the material preceding an action from journaled deposits.

    One append-only scene is kept per process. A longer chain extends it; a
    shorter or diverging one shares its leading capsules. Insertions repeat
    the validator's own sequence, so the result is the scene sequential
    checking would hold.
    """

    def __init__(self, load):
        self.load = load
        self.rebuilds = 0
        self.reset()

    def reset(self):
        self.scene = MaterialScene()
        self.chain, self.counts = [], []

    def _apply(self, key):
        for _, rows in self.load(key):
            for row in rows:
                self.scene.add(row["start"], row["end"], row["radius_m"], row["name"])
        self.chain.append(key)
        self.counts.append(len(self.scene.objects))

    def scene_for(self, chain):
        """The material of the deposit journals named by ``chain``, in order."""
        common = 0
        while (
            common < min(len(chain), len(self.chain)) and chain[common] == self.chain[common]
        ):
            common += 1
        if common == len(self.chain):
            for key in chain[common:]:
                self._apply(key)
            return self.scene
        base = self.scene.prefix(self.counts[common - 1]) if common else MaterialScene()
        if common == len(chain):
            return base  # Serve a pure prefix; keep the longer scene held.
        self.rebuilds += 1
        self.scene = base
        del self.chain[common:], self.counts[common:]
        for key in chain[common:]:
            self._apply(key)
        return self.scene
