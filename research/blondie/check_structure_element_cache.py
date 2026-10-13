"""Compare every structural result field with and without element reuse."""

import hashlib
import json
from copy import deepcopy
from math import cos, sin
from pathlib import Path
from time import perf_counter, process_time

import pen_art.structure as structure
from pen_art.io import read_json, sha256, write_json

large = Path("runs/palace_hybrid_spaced_1200um_source")
small = Path("runs/cube_sections_v1")
graph = read_json(large / "stroke_graph.json")
material = read_json(large / "fabrication_config.json")["structure"]
rows = {r["id"]: r for r in read_json(large / "construction_plan.json")["actions"]}
order = read_json(Path("research/blondie/palace_supported_order.json"))["order"][:169]
prefix = {m for key in order for m in rows[key]["members"]}
small_graph = read_json(small / "stroke_graph.json")
small_material = read_json(small / "fabrication_config.json")["structure"]
helper = structure._cached_beam_element
helper.cache_clear()
cases = [
    ("palace_full", graph, material, {}),
    ("palace_prefix_169", graph, material, {"edge_ids": prefix}),
    ("cube_baseline", small_graph, small_material, {}),
]
for key, factor in [("young_modulus_pa", 1.1), ("poisson_ratio", 0.9), ("density_kg_m3", 1.1)]:
    changed = {**small_material, key: small_material[key] * factor}
    cases.append((key, small_graph, changed, {}))
changed = deepcopy(small_graph)
for edge in changed["edges"]:
    edge["diameter_m"] *= 1.02
cases.append(("changed_diameter", changed, small_material, {}))
changed = deepcopy(small_graph)
for node in changed["nodes"]:
    x, y, z = node["position"]
    node["position"] = [cos(0.1) * x - sin(0.1) * y, sin(0.1) * x + cos(0.1) * y, z]
for edge in changed["edges"]:
    for segment in edge["segments"]:
        for point in segment:
            x, y, z = point
            point[:] = [cos(0.1) * x - sin(0.1) * y, sin(0.1) * x + cos(0.1) * y, z]
cases.append(("rotated_geometry", changed, small_material, {}))
free_node = next(n["id"] for n in small_graph["nodes"] if not n["ground"])
cases.append(
    (
        "changed_nodal_load",
        small_graph,
        small_material,
        {"nodal_loads": {free_node: [0.001, 0, 0, 0, 0, 0]}},
    )
)
cases.append(("changed_boundary", small_graph, small_material, {"fixed_translations": [free_node]}))
results = []
for name, shape, settings, kwargs in cases:
    row = {"case": name}
    outputs = []
    for mode, function in [
        ("uncached", helper.__wrapped__),
        ("cached", helper),
        ("warm_cached", helper),
    ]:
        structure._cached_beam_element = function
        wall, cpu = perf_counter(), process_time()
        result = structure.analyze_structure(shape, settings, **kwargs)
        row[mode] = {"wall_seconds": perf_counter() - wall, "cpu_seconds": process_time() - cpu}
        encoded = json.dumps(result, sort_keys=True, allow_nan=False).encode()
        row[mode]["result_sha256"] = hashlib.sha256(encoded).hexdigest()
        outputs.append(encoded)
    assert outputs[0] == outputs[1] == outputs[2], name
    row.update(status="EXACT_MATCH", modeled_pass=result["modeled_pass"])
    results.append(row)
    print(
        name,
        "EXACT_MATCH",
        row["uncached"]["cpu_seconds"],
        row["warm_cached"]["cpu_seconds"],
        flush=True,
    )
structure._cached_beam_element = helper
assert helper.cache_info().hits > 0
write_json(
    Path("research/blondie/structure_element_cache.json"),
    {
        "status": "PASS",
        "cases": results,
        "cache": helper.cache_info()._asdict(),
        "scope": (
            "Identical full result serialization, including all nodes and edges. "
            "Local timings under concurrent jobs; not a full-build speed claim."
        ),
        "simulation_only": True,
        "inputs": {
            str(p): sha256(p)
            for p in [
                large / "stroke_graph.json",
                large / "fabrication_config.json",
                small / "stroke_graph.json",
                small / "fabrication_config.json",
                Path("src/pen_art/structure.py"),
                Path(__file__),
            ]
        },
    },
)
