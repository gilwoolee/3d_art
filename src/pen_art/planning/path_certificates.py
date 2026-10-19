"""Exact checked-path proofs; growing-material validation is never implied."""

import hashlib
import json
import sys
from collections import Counter
from importlib.metadata import version
from pathlib import Path
from time import perf_counter

import numpy as np

from ..io import read_json, sha256, write_json


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


# Captured when this process imports it: a later source edit must not label an
# already running planner or verifier with the new implementation hash.
IMPLEMENTATION = digest(
    {
        "sources": {
            str(path.relative_to(Path(__file__).parents[1])): sha256(path)
            for path in sorted(Path(__file__).parents[1].rglob("*.py"))
        },
        "python": sys.version,
        "packages": {name: version(name) for name in ("numpy", "scipy", "python-fcl")},
    }
)


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


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


class FrozenContext(dict):
    """A proof context its owner will not change, so its text is formed once."""

    text = None


def _context_text(context):
    """Exactly ``json.dumps(_canonical(context), sort_keys=True)``, reusing frozen parts."""
    if isinstance(context, FrozenContext):
        if context.text is None:
            context.text = json.dumps(_canonical(context), sort_keys=True, allow_nan=False)
        return context.text
    if (
        isinstance(context, dict)
        and all(isinstance(key, str) for key in context)
        and any(isinstance(value, FrozenContext) for value in context.values())
    ):
        return "{" + ", ".join(
            f"{json.dumps(key)}: {_context_text(context[key])}" for key in sorted(context)
        ) + "}"
    return json.dumps(_canonical(context), sort_keys=True, allow_nan=False)


def drawing_binding(path_record, *, durations=True):
    """Exact local DRAW geometry; travel, waits and schedule offsets are excluded.

    Speed proofs additionally bind each exact floating-point duration. Rounding
    from adding a travel prefix therefore causes a safe miss, not relaxed reuse.
    Context still binds physical settings and source geometry independently.
    """
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
            "tracking_orientation_constraint": path_record.get("tracking_orientation_constraint")}


class PathCertificateStore:
    """Persist successful, scope-specific proofs under an exact immutable key.

    The caller supplies the complete physical/source/process context. This
    store additionally binds verifier implementation and the entire path record,
    including its timing and orientation metadata. It certifies no material scene.
    """

    def __init__(self, directory):
        self.directory = Path(directory)
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
            "context_sha256": hashlib.sha256(_context_text(context).encode()).hexdigest(),
            "path_sha256": digest(_canonical(
                drawing_binding(path_record, durations=False)
                if scope in {"source_coverage", "drawing_orientation"} and "edges" in path_record
                else drawing_binding(path_record)
                if scope == "drawing_speed" and "edges" in path_record
                else path_record
            )),
        }

    def get(self, context, path_record, scope):
        started = perf_counter()
        try:
            binding = self._binding(context, path_record, scope)
            key = digest(binding)
            path = self.directory / f"{key}.json"
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
        return {"counts": dict(self.stats), "seconds": dict(self.seconds)}
