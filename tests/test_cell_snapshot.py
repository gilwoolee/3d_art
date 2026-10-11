import json

import pytest

from pen_art.cell import snapshot_backbone
from pen_art.io import read_json, sha256, write_json


def test_supported_cell_snapshot_binds_order_to_matching_graph(tmp_path):
    source, output = tmp_path / "source", tmp_path / "cell"
    source.mkdir()
    output.mkdir()
    write_json(output / "run.json", {"stages": {}})
    # Different source formatting must not change the canonical input binding.
    (source / "backbone_graph.json").write_text(json.dumps({"edges": [{"id": "post"}]}))
    for name in ("backbone_pen_strokes", "backbone_structure", "backbone_process"):
        write_json(source / f"{name}.json", {})
    construction = {"actions": [{"members": ["post"]}]}
    write_json(source / "backbone_construction.json", construction)
    snapshot_backbone(output, source)
    exported = read_json(output / "construction_plan.json")
    assert exported["inputs"] == {"stroke_graph.json": sha256(output / "stroke_graph.json")}
    assert exported["actions"] == construction["actions"]
    for name, digest in read_json(output / "cell_source_bundle.json")["inputs"].items():
        assert sha256(output / name) == digest
    construction["actions"].append({"members": ["wrong-graph-member"]})
    write_json(source / "backbone_construction.json", construction)
    with pytest.raises(ValueError, match="exact graph members"):
        snapshot_backbone(output, source)
