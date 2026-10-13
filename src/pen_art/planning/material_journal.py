"""Shared read-only capsule arrays; each worker constructs its own FCL objects."""

from collections.abc import Sequence
from pathlib import Path

import numpy as np

from ..io import read_json, sha256, write_json


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
    for operation in operations:
        if operation[0] == 'turn':
            descriptors.append(list(operation))
        else:
            rows = operation[1]
            values = np.asarray([[*r['start'], *r['end'], r['radius_m']] for r in rows],
                                dtype='<f8').reshape(-1, 7)
            arrays.append(values)
            descriptors.append(['add', offset, len(values), [r['name'] for r in rows]])
            offset += len(values)
    data_path = path.with_suffix('.npy')
    temporary = data_path.with_suffix('.tmp')
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
        if entry[0] == 'turn':
            operations.append(tuple(entry))
        elif entry[0] == 'add':
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
    this growth would exceed ``budget_m``, at a gap, or at a table turn. The
    result only ever over-covers the journaled material.
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

    for operation in operations:
        if operation[0] == "turn":
            flush()
            merged.append(tuple(operation))
        else:
            rows.extend(operation[1])
    flush()
    return merged
