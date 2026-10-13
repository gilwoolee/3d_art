from pathlib import Path

from pen_art.io import sha256, write_json

source_path = Path("scripts/check_shared_source.py")
source = source_path.read_text()
results = []
for mode in ("forward", "backward"):
    run = Path(f"runs/staged_compact_cache_{mode}_v1")
    assert not run.exists(), run
    receipt = Path(f"research/blondie/staged_compact_cache_{mode}.json")
    code = (
        source.replace("runs/staged_shared_regression_v1", str(run))
        .replace("research/blondie/staged_shared_regression.json", str(receipt))
        .replace("regression_only=True", f"regression_only={mode == 'backward'}")
    )
    scope = {"__name__": "__main__"}
    exec(compile(code, str(source_path), "exec"), scope)
    build = scope["build"]
    full = build.structure[frozenset(build.actions)]
    assert "nodes" in full and "edges" in full
    compact = [v for k, v in build.structure.items() if len(k) < len(build.actions)]
    assert all("nodes" not in v and "edges" not in v for v in compact)
    assert compact if mode == "forward" else build.search_metadata["status"] == "FOUND"
    results.append(
        {
            "mode": mode,
            "run": str(run),
            "validation": str(receipt),
            "status": "PASS",
            "compact_prefix_entries": len(compact),
            "full_graph_geometry_retained": True,
            "checked_intervals": scope["validation"]["checked_intervals"],
            "regression": build.search_metadata,
        }
    )
    print("CURRENT CODE CACHE AND REPLAY PASS", mode, flush=True)
write_json(
    Path("research/blondie/compact_structure_cache.json"),
    {
        "status": "PASS",
        "simulation_only": True,
        "cases": results,
        "inputs": {
            str(p): sha256(p)
            for p in [
                source_path,
                Path("src/pen_art/planning/section_build.py"),
                Path("src/pen_art/planning/regression.py"),
                Path(__file__),
            ]
        },
    },
)
