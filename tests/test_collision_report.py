import hashlib
import json

import pytest

from pen_art.io import sha256, write_json
from pen_art.planning.collision_report import audit_drawing_paths


def test_audit_binds_results_to_snapshot_and_rejects_stale_inputs(tmp_path):
    robot = tmp_path / "robot.urdf"
    robot.write_text("""<robot name="test">
      <link name="base"/>
      <link name="flange"><collision><origin xyz="0 0 0.5"/>
        <geometry><sphere radius="0.1"/></geometry></collision></link>
      <joint name="joint" type="revolute"><parent link="base"/><child link="flange"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/>
      </joint></robot>""")
    write_json(
        tmp_path / "placement_search.json",
        {
            "selected": {"center_xy_m": [1, 0], "height_above_table_m": 0.1},
            "assumptions": {"table_z_m": 0},
        },
    )
    write_json(tmp_path / "asset_manifest.json", {"bounds_m": [[0, 0, 0], [0.1, 0.1, 0.1]]})
    write_json(
        tmp_path / "drawing_motion.json",
        {
            "requested_strokes": 1,
            "inputs": {"robot.urdf": sha256(robot)},
            "flange_to_tcp": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "settings": {"interpolation_tolerance_m": 0.0003, "interpolation_tolerance_rad": 0.003},
            "strokes": [
                {
                    "id": "one",
                    "source_stroke": "one",
                    "status": "SAMPLED_DRAWING_PATH",
                    "positions": [[0], [0]],
                    "tcp_targets_m": [[0, 0, 0], [0, 0, 0]],
                    "tcp_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                }
            ],
        },
    )
    config = tmp_path / "collision.yaml"
    config.write_text(
        json.dumps(
            {
                "tool_geometry": None,
                "table_size_m": [4, 4, 0.1],
                "platform_padding_m": 0.04,
                "clearance_m": 0.001,
                "maximum_subdivision_depth": 8,
            }
        )
    )
    report = audit_drawing_paths(tmp_path, config)
    assert report["status"] == "COMPLETE_ARM_STATIC_AUDIT"
    assert report["counts"]["CERTIFIED_MODELED_PATH"] == 1
    assert (
        report["source_motion_sha256"]
        == hashlib.sha256((tmp_path / "drawing_motion.json").read_bytes()).hexdigest()
    )
    settings = json.loads(config.read_text())
    settings["tool_geometry"] = {
        "schema_version": 1,
        "units": "m",
        "flange_to_tcp": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        "tip_radius_m": 0.0005,
        "bodies": [
            {
                "name": "pen",
                "origin": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.02], [0, 0, 0, 1]],
                "shape": {"type": "sphere", "radius_m": 0.025},
            }
        ],
    }
    config.write_text(json.dumps(settings))
    tool_report = audit_drawing_paths(tmp_path, config)
    assert tool_report["counts"]["COLLISION"] == 1
    assert tool_report["tool_model"]["configured"]
    assert any(
        pair["first"] == "tool/pen" and pair["second"] == "table"
        for pair in tool_report["strokes"][0]["witnesses"][0]["pairs"]
    )
    robot.write_text(robot.read_text() + "\n")
    with pytest.raises(ValueError, match="Motion input is stale"):
        audit_drawing_paths(tmp_path, config)
