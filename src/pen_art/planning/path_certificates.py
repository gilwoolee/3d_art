"""Exact checked-path proofs; growing-material validation is never implied."""

import json
from collections import Counter
from pathlib import Path
from time import perf_counter

import numpy as np

from ..io import read_json, write_json
from .replay_checkpoint import IMPLEMENTATION, digest

POLICY = "exact-path-invariants-v2"
SCOPE_STATUS = {
    "invariant_path": "CERTIFIED_MODELED_PATH",
    "pen_invariant_path": "CERTIFIED_PEN_MODELED_PATH",
    "source_coverage": "BOUNDED",
    "joint_speed": "CHECKED",
    "drawing_speed": "CHECKED",
    "drawing_orientation": "BOUNDED",
    # The pen and tool against the cell and the wire of a named preceding chain.
    "pen_material_path": "CHECKED",
}


def _canonical(value):
    def convert(item):
        if hasattr(item, "tolist"):
            return item.tolist()
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        if isinstance(item, Path):
            return str(item)
        raise TypeError(f"Unsupported certificate value: {type(item).__name__}")

    return json.loads(json.dumps(value, default=convert, allow_nan=False, sort_keys=True))


def drawing_binding(path_record, *, durations=True):
    """Exact local DRAW geometry; travel, waits and schedule offsets are excluded.

    Speed proofs additionally bind each exact floating-point duration. Rounding
    from adding a travel prefix therefore causes a safe miss, not relaxed reuse.
    Context still binds physical settings and source geometry independently.
    """
    from .clock_store import plain

    edges = []
    for row in path_record["edges"]:
        if row["kind"] != "DRAW":
            continue
        local = {key: plain(value) for key, value in row.items()
                 if key not in {"start_s", "end_s"}}
        if durations:
            local["duration_s"] = row["end_s"] - row["start_s"]
        edges.append(local)
    return {"edges": edges, "fixed_tcp_rotation": path_record.get("fixed_tcp_rotation"),
            "drawing_table_index": path_record.get("drawing_table_index"),
            "tracking_orientation_constraint": path_record.get("tracking_orientation_constraint")}


class PathCertificateStore:
    """Persist successful, scope-specific proofs under an exact immutable key.

    The caller supplies the complete physical/source/process context. This
    store additionally binds verifier implementation and the entire path record,
    including its timing and orientation metadata. It certifies no material scene.
    """

    def __init__(self, directory, *, enabled=True, fallback=None):
        self.directory = Path(directory)
        # A frozen library's proofs stay readable while new ones are written
        # elsewhere, so concurrent or repeated runs see identical inputs.
        self.fallback = None if fallback is None else Path(fallback)
        self.enabled = enabled
        self.stats = Counter()
        self.seconds = Counter()
        self._memory = {}

    def _binding(self, context, path_record, scope):
        if scope not in SCOPE_STATUS:
            raise ValueError(f"Unknown path certificate scope: {scope}")
        return {
            "policy": POLICY,
            "implementation": IMPLEMENTATION,
            "scope": scope,
            "context_sha256": digest(_canonical(context)),
            "path_sha256": digest(_canonical(
                drawing_binding(path_record, durations=False)
                if scope in {"source_coverage", "drawing_orientation"} and "edges" in path_record
                else drawing_binding(path_record)
                if scope == "drawing_speed" and "edges" in path_record
                else path_record
            )),
        }

    def key(self, context, path_record, scope):
        return digest(self._binding(context, path_record, scope))

    def get(self, context, path_record, scope):
        started = perf_counter()
        try:
            if not self.enabled:
                self.stats["disabled"] += 1
                return None
            binding = self._binding(context, path_record, scope)
            key = digest(binding)
            path = self.directory / f"{key}.json"
            if self.fallback is not None and not path.exists():
                path = self.fallback / f"{key}.json"
            try:
                stat = path.stat()
                signature = (stat.st_mtime_ns, stat.st_size)
                cached = self._memory.get(key)
                row = cached[1] if cached and cached[0] == signature else read_json(path)
                checksum = row.get("checksum")
                contents = {name: value for name, value in row.items() if name != "checksum"}
                if (
                    row.get("kind") != "exact_path_certificate"
                    or row.get("binding") != binding
                    or checksum != digest(contents)
                    or not isinstance(row.get("proof"), dict)
                    or row["proof"].get("status") != SCOPE_STATUS[scope]
                ):
                    self.stats[f"{scope}_rejected"] += 1
                    self._memory.pop(key, None)
                    return None
            except FileNotFoundError:
                self.stats[f"{scope}_misses"] += 1
                return None
            except (OSError, ValueError, KeyError, TypeError, AttributeError):
                self.stats[f"{scope}_rejected"] += 1
                self._memory.pop(key, None)
                return None
            self._memory[key] = (signature, row)
            self.stats[f"{scope}_hits"] += 1
            self.stats["memory_hits" if cached and cached[0] == signature else "disk_hits"] += 1
            return _canonical(row["proof"])
        finally:
            self.seconds["lookup"] += perf_counter() - started

    def admit(self, context, path_record, scope, proof):
        binding = self._binding(context, path_record, scope)
        proof = _canonical(proof)
        if not isinstance(proof, dict) or proof.get("status") != SCOPE_STATUS[scope]:
            raise ValueError(f"Only a successful {scope} proof can be admitted")
        if not self.enabled:
            self.stats["disabled"] += 1
            return None
        key = digest(binding)
        contents = {"kind": "exact_path_certificate", "binding": binding, "proof": proof}
        row = {**contents, "checksum": digest(contents)}
        path = self.directory / f"{key}.json"
        write_json(path, row)
        stat = path.stat()
        self._memory[key] = ((stat.st_mtime_ns, stat.st_size), row)
        self.stats[f"{scope}_admitted"] += 1
        return key

    def report(self):
        return {"counts": dict(self.stats), "seconds": dict(self.seconds), "enabled": self.enabled}
