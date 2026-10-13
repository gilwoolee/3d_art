"""Lossless memory-mapped computational clocks, bound to the viewer plan.

The JSON plan remains the interchange format. The optional sidecar packs only
numeric edge fields; labels, missing fields and all other metadata stay in its
checksummed manifest. A present but invalid sidecar is rejected, never ignored.
"""

import json
import re
import struct
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from ..io import sha256, write_json

FIELDS = {"start_s", "end_s", "q0", "q1", "contact_m", "material_radius_m",
          "tcp0_m", "tcp1_m", "coefficients", "table_index", "table_to_index"}
VERSION = 2


def _digest(value):
    import hashlib

    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


class PackedEdges(Sequence):
    def __init__(self, descriptors, values):
        self.descriptors, self.values = descriptors, values

    def __len__(self):
        return len(self.descriptors)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        entry = self.descriptors[index]
        row = dict(entry["labels"])
        for name, layout in entry["numbers"].items():
            if layout is None:
                row[name] = None
                continue
            start, count, shape = layout
            data = self.values[start:start + count]
            if name == "coefficients":
                offset, rows = 0, []
                for length in shape:
                    rows.append(data[offset:offset + length])
                    offset += length
                row[name] = rows
            elif shape == []:
                value = float(data[0])
                row[name] = int(value) if name.startswith("table_") else value
            else:
                row[name] = data.reshape(shape)
        return row

    def tolist(self):
        return [plain(self[i]) for i in range(len(self))]


def plain(value):
    if isinstance(value, (np.ndarray, PackedEdges)):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def write_plan(path, plan):
    """Publish compact viewer JSON and an atomically committed packed sidecar."""
    path = Path(path)
    # The viewer file anchors the complete packed representation; a manifest
    # cannot be relabeled with another clock while retaining that viewer file.
    plan = {key: value for key, value in plan.items() if key != "packed_clock_binding"}
    chunks, offset = [], 0
    header = {key: value for key, value in plan.items() if key != "actions"}
    actions = []
    for action in plan["actions"]:
        row = {key: value for key, value in action.items() if key != "trajectory"}
        trajectory = action["trajectory"]
        descriptors = []
        for edge in trajectory["edges"]:
            entry = {"labels": {key: value for key, value in edge.items() if key not in FIELDS},
                     "numbers": {}}
            for key in sorted(FIELDS & edge.keys()):
                value = edge[key]
                if value is None:
                    entry["numbers"][key] = None
                    continue
                if key.startswith("table_") and value not in (0, 1, 2, 3):
                    raise ValueError("Packed table indices must be quarter-turn indices")
                if key == "coefficients":
                    shape = [len(coefficients) for coefficients in value]
                    data = np.concatenate([np.asarray(c, dtype=np.float64) for c in value])
                else:
                    raw = np.asarray(value)
                    if raw.dtype.kind not in "iuf":
                        raise ValueError("Non-numeric clock payload")
                    array = np.asarray(value, dtype=np.float64)
                    if raw.dtype.kind in "iu" and (np.any(raw > 2**53) or np.any(raw < -(2**53))):
                        if not np.array_equal(raw.astype(object), array.astype(object)):
                            raise ValueError("Clock integer cannot be represented losslessly")
                    shape, data = list(array.shape), array.ravel()
                if not np.isfinite(data).all():
                    raise ValueError("Nonfinite clock payload")
                chunks.append(data)
                entry["numbers"][key] = [offset, len(data), shape]
                offset += len(data)
            descriptors.append(entry)
        row["trajectory"] = {key: value for key, value in trajectory.items() if key != "edges"}
        row["trajectory"]["edges"] = descriptors
        actions.append(row)
    values = np.concatenate(chunks) if chunks else np.empty(0, dtype=np.float64)
    # Content-addressed data names leave existing manifests valid during export.
    import hashlib

    payload_hash = hashlib.sha256(values.tobytes()).hexdigest()
    data_name = f"{path.stem}.{payload_hash}.npy"
    data_path = path.with_name(data_name)
    temporary = data_path.with_suffix(".tmp")
    with temporary.open("wb") as stream:
        np.save(stream, values, allow_pickle=False)
    temporary.replace(data_path)
    manifest = {"version": VERSION, "data": data_name,
                "data_sha256": sha256(data_path), "count": len(values),
                "header": header, "actions": actions}
    binding = _digest(manifest)
    write_json(path, {"packed_clock_binding": binding, **plan}, compact=True)
    manifest["plan_sha256"] = sha256(path)
    write_json(path.with_suffix(".clocks.json"), {**manifest, "checksum": _digest(manifest)},
               compact=True)


def load_plan(path):
    path = Path(path)
    plan_hash = sha256(path)
    manifest_path = path.with_suffix(".clocks.json")
    if not manifest_path.exists():
        return json.loads(path.read_bytes()), plan_hash
    manifest = json.loads(manifest_path.read_bytes())
    contents = {key: value for key, value in manifest.items() if key != "checksum"}
    if (manifest.get("version") not in {1, VERSION} or manifest.get("plan_sha256") != plan_hash
            or manifest.get("checksum") != _digest(contents)):
        raise ValueError("Stale or corrupt packed clock manifest")
    binding = None
    if manifest["version"] == VERSION:
        with path.open("rb") as stream:
            anchor = re.match(rb'^\{\s*"packed_clock_binding"\s*:\s*"([0-9a-f]{64})"',
                              stream.read(256))
        binding = _digest({key: value for key, value in contents.items() if key != "plan_sha256"})
        if anchor is None or anchor[1].decode("ascii") != binding:
            raise ValueError("Packed clocks disagree with the viewer plan binding")
    name = manifest["data"]
    if Path(name).name != name:
        raise ValueError("Packed clock data must be adjacent to its manifest")
    data_path = path.with_name(name)
    if sha256(data_path) != manifest["data_sha256"]:
        raise ValueError("Corrupt packed clock data")
    values = np.load(data_path, mmap_mode="r", allow_pickle=False)
    if (values.dtype != np.float64 or values.shape != (manifest["count"],)
            or not np.isfinite(values).all()):
        raise ValueError("Malformed packed clock array")
    for action in manifest["actions"]:
        descriptors = action["trajectory"]["edges"]
        for entry in descriptors:
            for key, layout in entry["numbers"].items():
                if key not in FIELDS:
                    raise ValueError("Unknown packed numeric field")
                if layout is None:
                    continue
                start, count, shape = layout
                size = sum(shape) if key == "coefficients" else int(np.prod(shape))
                if start < 0 or count < 1 or start + count > len(values) or size != count:
                    raise ValueError("Malformed packed clock layout")
        action["trajectory"]["edges"] = PackedEdges(descriptors, values)
    packed = {**manifest["header"], "actions": manifest["actions"]}
    if binding is not None:
        packed["packed_clock_binding"] = binding
    else:
        # Legacy files have no authoritative anchor. Verify their contents
        # against the actual viewer JSON before consuming any packed clocks.
        original = json.loads(path.read_bytes())
        if not _exact_equal(original, packed):
            raise ValueError("Legacy packed clocks differ from the viewer plan")
    return packed, plan_hash


def _exact_equal(first, second):
    """Compare legacy content without materializing a second complete plan."""
    if isinstance(first, dict):
        return (isinstance(second, dict) and first.keys() == second.keys()
                and all(_exact_equal(value, second[key]) for key, value in first.items()))
    if isinstance(first, list):
        if not isinstance(second, (list, np.ndarray, PackedEdges)) or len(first) != len(second):
            return False
        try:
            a, b = np.asarray(first), np.asarray(second)
            if a.dtype.kind not in "iuf" or b.dtype.kind not in "iuf":
                raise ValueError("Compare nonnumeric metadata recursively")
            for values in (a, b):
                if values.dtype.kind in "iu" and (np.any(values > 2**53)
                                                 or np.any(values < -(2**53))):
                    raise ValueError("Compare large integers without float conversion")
            return (a.shape == b.shape and
                    a.astype(np.float64).tobytes() == b.astype(np.float64).tobytes())
        except (TypeError, ValueError):
            return all(_exact_equal(a, b) for a, b in zip(first, second, strict=True))
    if isinstance(second, np.generic):
        second = second.item()
    if isinstance(first, bool) or isinstance(second, bool):
        return type(first) is type(second) and first == second
    if isinstance(first, (int, float)) and isinstance(second, (int, float, np.generic)):
        return first == second and struct.pack("d", float(first)) == struct.pack("d", float(second))
    return first == second
