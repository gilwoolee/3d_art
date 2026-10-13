"""Check exact polynomial bounds and full replay with cached stationary roots."""

import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from time import perf_counter

import numpy as np
from numpy.polynomial import Chebyshev, Polynomial

import pen_art.timing as timing
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_replay import replay_cell
from pen_art.planning.clock_path import ClockPath

source = Path("runs/staged_beam_cache_v1")
saved = read_json(source / "offline_dual_arm_plan.json")
cases = []
for action in saved["actions"]:
    edges = ClockPath.from_record(action["trajectory"]).edges
    for edge in edges[:: max(1, len(edges) // 12)]:
        for polynomial in edge.piece().polynomials:
            for order in range(3):
                p = polynomial.deriv(order)
                cases.extend(
                    (p, a, b) for a, b in [(0.0, 1.0), (0.1, 0.9), (0.2, 0.3), (0.499, 0.501)]
                )
cases.extend(
    [
        (Polynomial([0.0, 0.0, 0.0, 10.0, -15.0, 6.0]), 0.0, 1.0),
        (Polynomial([3.0]), 0.2, 0.8),
        (Polynomial([0.0, 6.4, -6.4], domain=[2.0, 5.0]), 2.1, 4.4),
        (Chebyshev([0.3, 0.1, -0.8, 0.2], domain=[-3.0, 7.0]), -2.0, 6.0),
    ]
)
helper = timing._stationary_roots
helper.cache_clear()
results = {}
reference = None
for label, function in [("uncached", helper.__wrapped__), ("cached", helper)]:
    timing._stationary_roots = function
    began = perf_counter()
    values = [timing.polynomial_range(p, a, b) for p, a, b in cases]
    seconds = perf_counter() - began
    if reference is None:
        reference = values
    else:
        assert values == reference
    results[label] = {
        "seconds": seconds,
        "result_sha256": hashlib.sha256(json.dumps(values).encode()).hexdigest(),
    }
timing._stationary_roots = helper
# Mutable coefficient/domain arrays must not retain roots from a prior value.
default_domain = Polynomial.domain.copy()
p = Polynomial([0.0, 1.0, -1.0], domain=default_domain.copy())
for coefficient, domain in [(-1.0, [-1.0, 1.0]), (-2.0, [-1.0, 1.0]), (-2.0, [0.0, 2.0])]:
    p.coef[-1] = coefficient
    p.domain[:] = domain
    actual = timing.polynomial_range(p, 0.1, 0.9)
    timing._stationary_roots = helper.__wrapped__
    expected = timing.polynomial_range(p, 0.1, 0.9)
    timing._stationary_roots = helper
    assert actual == expected
assert np.array_equal(Polynomial.domain, default_domain)
replays = {}
baseline = None
for label, function in [("uncached", helper.__wrapped__), ("cached", helper)]:
    run = Path(f"runs/staged_root_cache_{label}_v3")
    assert not run.exists(), run
    shutil.copytree(source, run)
    metadata = read_json(run / "run.json")
    metadata["id"] = run.name
    metadata["name"] = f"Stationary-root cache comparison: {label}"
    write_json(run / "run.json", metadata)
    (run / "offline_dual_arm_replay_checkpoint.json").unlink(missing_ok=True)
    timing._stationary_roots = function
    began = perf_counter()
    validation = replay_cell(
        run, Path("research/planning_cell/cell.yaml"), 600, "offline_dual_arm_plan"
    )
    seconds = perf_counter() - began
    assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
    assert not validation.get("resumed_from_s")
    if baseline is None:
        baseline = validation
    else:
        assert validation == baseline
    replays[label] = {
        "run": str(run),
        "seconds": seconds,
        "checked_intervals": validation["checked_intervals"],
        "validation_sha256": sha256(run / "offline_dual_arm_validation.json"),
    }
    print(label, "full replay", seconds, validation["checked_intervals"], flush=True)
timing._stationary_roots = helper
write_json(
    Path("research/blondie/polynomial_root_cache.json"),
    {
        "status": "REPLAY_COMPARISON_PASSED_BROWSER_PENDING",
        "range_cases": len(cases),
        "ranges": results,
        "replays": replays,
        "cache": helper.cache_info()._asdict(),
        "simulation_only": True,
        "scope": (
            "Exact bounds and complete replay match; timings are local under concurrent jobs, "
            "not a full-build speed claim."
        ),
        "inputs": {
            str(p): sha256(p)
            for p in [
                source / "offline_dual_arm_plan.json",
                Path("src/pen_art/timing.py"),
                Path(__file__),
            ]
        },
    },
)
subprocess.run(
    [
        "tools/node/bin/node",
        "web/scripts/check-replay.mjs",
        "staged_root_cache_cached_v3",
        "research/blondie/root_cache_browser",
    ],
    check=True,
)
receipt_path = Path("research/blondie/polynomial_root_cache.json")
receipt = read_json(receipt_path)
receipt["status"] = "PASS"
receipt["browser_receipt"] = "research/blondie/root_cache_browser_browser_replay.json"
write_json(receipt_path, receipt)
