"""Leakage-resistant curation of witnessed drawing attempts."""

import hashlib
import json
from pathlib import Path

import numpy as np

STATUSES = {"VERIFIED_VALID", "COLLISION", "CONSTRAINT_VIOLATION", "UNRESOLVED"}
REPRESENTATION = "piecewise_linear_joint_v1"


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def read_episodes(paths):
    """Read JSONL episodes or checksummed MotionLibrary attempt envelopes.

    A library root selects only its attempts directory, so the same successful
    record is not read twice from attempts and motions. JSON envelope checksums
    use the owning motion library's canonical serialization.
    """
    from ..planning.motion_library import digest as record_digest

    def unwrap(value, location):
        if not isinstance(value, dict):
            raise ValueError(f"Episode must be an object: {location}")
        motion_envelope = "record" in value
        if motion_envelope:
            record = value["record"]
            if not isinstance(record, dict) or value.get("digest") != record_digest(record):
                raise ValueError(f"Motion attempt envelope digest mismatch: {location}")
            value = record
        version = value.get("schema_version")
        if motion_envelope and version == 2:
            # The checksummed cache schema changes admission policy, not the
            # learning episode layout. Normalize after validating its checksum.
            return {**value, "schema_version": 1, "motion_record_schema_version": 2}
        if version != 1:
            raise ValueError(f"Unsupported episode schema: {location}")
        return value

    seen_files, seen_envelopes = set(), set()
    for input_path in paths:
        path = Path(input_path)
        if path.is_dir():
            directory = path / "attempts" if (path / "attempts").is_dir() else path
            files = sorted(directory.glob("*.json"))
            if not files:
                raise ValueError(f"No motion attempt envelopes found: {directory}")
        else:
            files = [path]
        for file in files:
            resolved = file.resolve()
            if resolved in seen_files:
                continue
            seen_files.add(resolved)
            if file.suffix.lower() == ".json":
                value = json.loads(file.read_text())
                if (
                    path.is_dir()
                    and isinstance(value, dict)
                    and not (
                        "record" in value
                        or "digest" in value
                        or ("request" in value and "validation" in value)
                    )
                ):
                    continue  # Directory-level manifests are not attempt records.
                episode = unwrap(value, str(file))
                if "record" in value:
                    checksum = value["digest"]
                    if checksum in seen_envelopes:
                        continue
                    seen_envelopes.add(checksum)
                yield episode
            else:
                with file.open() as stream:
                    for number, line in enumerate(stream, 1):
                        if line.strip():
                            yield unwrap(json.loads(line), f"{file}:{number}")


def curve_samples(points, count):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError("A source curve needs at least two 3D points")
    if not np.isfinite(points).all():
        raise ValueError("Source curve is non-finite")
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if distances[-1] <= 1e-12:
        raise ValueError("Zero-length source curve")
    keep = np.r_[True, np.diff(distances) > 1e-12]
    samples = np.linspace(0.0, distances[-1], count)
    return np.column_stack([np.interp(samples, distances[keep], points[keep, i]) for i in range(3)])


def normalized_request(request, count):
    reference = request.get("source_reference_points_m", request["source_points_m"])
    source = curve_samples(reference, count)
    table = request["table_index"]
    if isinstance(table, bool) or table not in (0, 1, 2, 3):
        raise ValueError("table_index must be one of 0, 1, 2, 3")
    physical_length = float(
        np.linalg.norm(np.diff(np.asarray(reference, float), axis=0), axis=1).sum()
    )
    values = [*source.ravel(), physical_length]
    for field in ("q_start", "q_goal"):
        q = request.get(field)
        if q is None:
            values.extend([0.0] * 7 + [0.0])
        else:
            q = np.asarray(q, dtype=float)
            if q.shape != (7,) or not np.isfinite(q).all():
                raise ValueError(f"Invalid {field}")
            values.extend([*q, 1.0])
    values.extend(float(table == index) for index in range(4))
    fixed = request.get("fixed_tcp_rotation")
    rotations = np.asarray(
        [fixed] if fixed is not None else request.get("allowed_orientations") or [], dtype=float
    )
    if rotations.size:
        rotations = rotations.reshape(-1, 9)
        if not np.isfinite(rotations).all():
            raise ValueError("Invalid orientation policy")
        values.extend(np.r_[rotations.mean(0), rotations.min(0), rotations.max(0), len(rotations)])
    else:
        values.extend([0.0] * 28)
    return np.asarray(values, dtype=np.float32)


def split_groups(episodes, seed=17):
    """Keep all connected object/family relatives together before augmentation.

    A family shared by several objects joins their groups transitively. Missing
    source identity is an error rather than silently splitting augmented twins.
    """
    parent = {}

    def find(value):
        parent.setdefault(value, value)
        if parent[value] != value:
            parent[value] = find(parent[value])
        return parent[value]

    identities = []
    for item in episodes:
        request = item["request"]
        obj, family = request.get("source_object"), request.get("family_id")
        if not obj or not family:
            raise ValueError("Every episode needs source_object and family_id")
        a, b = f"object:{obj}", f"family:{family}"
        parent[find(b)] = find(a)
        identities.append(a)
    explicit = [item.get("split") for item in episodes]
    if any(value is not None for value in explicit):
        if any(value not in (None, "train", "validation", "test") for value in explicit):
            raise ValueError("Invalid explicit split assignment")
        groups = {}
        for identity, value in zip(identities, explicit, strict=True):
            if value is None:
                continue
            root = find(identity)
            if root in groups and groups[root] != value:
                raise ValueError("Related object/family episodes cross explicit splits")
            groups[root] = value
        # Existing object/family groups retain their frozen split. Feedback from
        # genuinely new groups is training-only; it cannot alter held-out groups.
        return [groups.get(find(identity), "train") for identity in identities]
    roots = sorted({find(value) for value in identities}, key=lambda x: digest([seed, x]))
    if len(roots) < 3:
        raise ValueError("Need at least three independent object/family groups")
    test_count = max(1, round(len(roots) * 0.15))
    val_count = max(1, round(len(roots) * 0.15))
    mapping = {
        group: "test" if i < test_count else "validation" if i < test_count + val_count else "train"
        for i, group in enumerate(roots)
    }
    return [mapping[find(value)] for value in identities]


def curate(paths, output, *, knots=32, probes=32, seed=17, signature_fn=None, fit_validator=None):
    """Write frozen splits; unknown attempts never become negative feasibility labels.

    fit_validator receives (episode, q_knots, source_knots), and must independently
    validate precisely this piecewise-linear joint path including deposition.
    """
    if knots < 2 or probes < 1:
        raise ValueError("Need at least two knots and one collision probe")
    episodes = list(read_episodes(paths))
    splits = split_groups(episodes, seed)
    contexts = {
        e["validation"].get("context_hash")
        or (e["request"].get("context_hash") if e["validation"]["status"] == "UNRESOLVED" else None)
        for e in episodes
    }
    if len(contexts) != 1 or None in contexts:
        raise ValueError("Dataset requires one explicit model/static context_hash")
    for episode in episodes:
        requested = episode["request"].get("context_hash")
        if requested and requested not in contexts:
            raise ValueError("Request and validation model/static contexts differ")
    rows, outcomes, unresolved, rejected = [], [], [], []
    for episode, split in zip(episodes, splits, strict=True):
        status = episode["validation"]["status"]
        if status not in STATUSES:
            raise ValueError(f"Invalid outcome {status}")
        row = {"episode": episode, "split": split, "id": digest(episode)}
        if status == "UNRESOLVED":
            unresolved.append(row)
        elif status != "VERIFIED_VALID":
            outcomes.append(row)
        else:
            trajectory = episode["trajectory"]
            seed_only = trajectory.get("q_path_validation") == "SEED_ONLY"
            if seed_only and fit_validator is None:
                rejected.append({"id": row["id"], "reason": "seed path needs fit validation"})
                continue
            q = np.asarray(trajectory["q_path"], dtype=float)
            source = np.asarray(trajectory["source_points_m"], dtype=float)
            if q.shape != (len(source), 7) or len(q) < 2 or not np.isfinite(q).all():
                raise ValueError("Malformed witnessed trajectory")
            progress = np.r_[0, np.cumsum(np.linalg.norm(np.diff(source, axis=0), axis=1))]
            if progress[-1] <= 1e-12 or np.any(np.diff(progress) <= 0):
                rejected.append({"id": row["id"], "reason": "nonmonotone source progress"})
                continue
            grid = np.linspace(0, progress[-1], knots)
            fitted = np.column_stack([np.interp(grid, progress, q[:, j]) for j in range(7)])
            fitted_source = curve_samples(source, knots)
            exact = q.shape == fitted.shape and np.allclose(q, fitted, rtol=0, atol=1e-12)
            witnessed = (
                not seed_only
                and exact
                and episode["validation"].get("representation") == REPRESENTATION
                and episode["validation"].get("fitted_path_verified") is True
            )
            if not witnessed and (
                fit_validator is None or not fit_validator(episode, fitted, fitted_source)
            ):
                rejected.append({"id": row["id"], "reason": "fitted path has no valid witness"})
                continue
            row.update(q_path=fitted.tolist(), source_points_m=fitted_source.tolist())
            row["fit_validation"] = {
                "status": "VERIFIED_VALID",
                "representation": REPRESENTATION,
                "method": "inherited_exact_path" if witnessed else "independent_refit_validation",
                "validator": (
                    episode["validation"].get("validator")
                    if witnessed
                    else f"{fit_validator.__module__}:{fit_validator.__qualname__}"
                ),
                "path_hash": digest(
                    {"q_path": fitted.tolist(), "source_points_m": fitted_source.tolist()}
                ),
            }
            rows.append(row)
            outcomes.append(row)
    train = [r for r in rows if r["split"] == "train"]
    if not train:
        raise ValueError("No verified training paths after fit validation")
    pool = np.concatenate([np.asarray(r["q_path"]) for r in train])
    # Farthest-point selection represents distinct postures, fitted on train only.
    selected = [0]
    distances = np.full(len(pool), np.inf)
    for _ in range(min(probes, len(pool)) - 1):
        distances = np.minimum(distances, ((pool - pool[selected[-1]]) ** 2).sum(axis=1))
        next_index = int(distances.argmax())
        if distances[next_index] < 1e-12:
            break
        selected.append(next_index)
    probe_q = pool[selected]
    if signature_fn is None:
        raise ValueError("A robot/material collision signature oracle is required")
    for row in rows:
        row["signature"] = np.asarray(signature_fn(row["episode"], probe_q), float).tolist()
        signature = np.asarray(row["signature"])
        if signature.shape != (len(probe_q),) or not np.isin(signature, (0, 1)).all():
            raise ValueError("Collision signature must be a binary vector matching probes")
    manifest = {
        "schema_version": 1,
        "representation": REPRESENTATION,
        "knots": knots,
        "seed": seed,
        "context_hash": next(iter(contexts)),
        "probe_q": probe_q.tolist(),
        "probe_training_ids": [r["id"] for r in train],
        "split_members": {
            name: [r["id"] for r in rows if r["split"] == name]
            for name in ("train", "validation", "test")
        },
        "valid_counts": {
            name: sum(r["split"] == name for r in rows) for name in ("train", "validation", "test")
        },
        "signature_patterns": {
            name: len({tuple(row["signature"]) for row in rows if row["split"] == name})
            for name in ("train", "validation", "test")
        },
        "outcome_count": len(outcomes),
        "unresolved_count": len(unresolved),
        "rejected": rejected,
    }
    manifest["dataset_hash"] = digest({"manifest": manifest, "rows": rows})
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    for name, values in (("motions", rows), ("outcomes", outcomes), ("unresolved", unresolved)):
        (output / f"{name}.jsonl").write_text("".join(json.dumps(v) + "\n" for v in values))
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def load_dataset(path):
    """Verify the frozen valid-motion set before training or evaluation."""
    path = Path(path)
    manifest = json.loads((path / "manifest.json").read_text())
    rows = [json.loads(line) for line in (path / "motions.jsonl").read_text().splitlines()]
    expected = dict(manifest)
    stored = expected.pop("dataset_hash")
    if digest({"manifest": expected, "rows": rows}) != stored:
        raise ValueError("Dataset content differs from its frozen manifest")
    for split in ("train", "validation", "test"):
        if [row["id"] for row in rows if row["split"] == split] != manifest["split_members"][split]:
            raise ValueError("Dataset split differs from its frozen manifest")
    return manifest, rows
