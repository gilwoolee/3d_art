"""Exercise element reuse through the real staged robot build and browser."""

import subprocess
from pathlib import Path

from pen_art.io import sha256, write_json
from pen_art.structure import _cached_beam_element

run = "staged_beam_cache_v1"
source = Path("scripts/check_shared_source.py")
code = (
    source.read_text()
    .replace("runs/staged_shared_regression_v1", f"runs/{run}")
    .replace(
        "research/blondie/staged_shared_regression.json",
        "research/blondie/staged_beam_cache.json",
    )
)
assert not Path(f"runs/{run}").exists()
_cached_beam_element.cache_clear()
exec(compile(code, str(source), "exec"), {"__name__": "__main__"})
info = _cached_beam_element.cache_info()
assert info.hits > 0 and info.currsize > 0
subprocess.run(
    [
        "tools/node/bin/node",
        "web/scripts/check-replay.mjs",
        run,
        "research/blondie/beam_cache_browser",
    ],
    check=True,
)
write_json(
    Path("research/blondie/structure_element_cache_e2e.json"),
    {
        "status": "PASS",
        "run": f"runs/{run}",
        "simulation_only": True,
        "cache": info._asdict(),
        "inputs": {
            str(p): sha256(p) for p in [source, Path(__file__), Path("src/pen_art/structure.py")]
        },
    },
)
