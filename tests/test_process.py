import yaml

from pen_art.graph import straight_edge
from pen_art.process import evaluate_action


def test_two_final_anchors_do_not_support_an_unfinished_bridge():
    profile = yaml.safe_load(open("configs/fabrication.yaml"))["process"]
    profile.update(
        calibrated=True, maximum_free_tip_length_m=0.012, maximum_horizontal_reach_m=0.008
    )
    nodes = {
        "a": {"position": [0.0, 0, 0.02], "ground": False},
        "b": {"position": [0.03, 0, 0.02], "ground": False},
    }
    bridge = straight_edge("a", "b", nodes, "bridge", "artistic", 0.001)
    result = evaluate_action(bridge, nodes, profile, {"a", "b"}, {"a": 10.0, "b": 10.0})
    assert "UNSUPPORTED_LENGTH" in result["failures"]
    assert "HORIZONTAL_REACH" in result["failures"]
    assert not result["modeled_pass"]


def test_mesh_contact_does_not_create_a_start_anchor():
    profile = yaml.safe_load(open("configs/fabrication.yaml"))["process"]
    profile["cooled_anchor_age_s"] = 5.0
    nodes = {
        "a": {"position": [0.0, 0, 0.02], "ground": False},
        "b": {"position": [0.0, 0, 0.025], "ground": False},
    }
    edge = straight_edge("a", "b", nodes, "rod", "artistic", 0.001)
    result = evaluate_action(edge, nodes, profile)
    assert result["failures"] == ["NO_EXISTING_START_ANCHOR"]
    ready = evaluate_action(edge, nodes, profile, {"a"}, {"a": 1.0})
    assert ready["modeled_pass"]
    assert ready["cooling_wait_s"] == 4.0
    assert ready["physical_status"] == "requires_calibration"


def test_unknown_pen_limits_do_not_reject_a_long_stroke():
    profile = yaml.safe_load(open("configs/fabrication.yaml"))["process"]
    nodes = {
        "a": {"position": [0.0, 0, 0.02], "ground": False},
        "b": {"position": [0.05, 0, 0.02], "ground": False},
    }
    edge = straight_edge("a", "b", nodes, "bridge", "artistic", 0.001)
    result = evaluate_action(edge, nodes, profile, {"a"})
    assert result["modeled_pass"]
    assert result["physical_status"] == "requires_calibration"
    assert result["cooling_wait_s"] is None
    assert "maximum_free_tip_length_m" in result["unknown_limits"]
    profile["maximum_free_tip_length_m"] = 0.012
    assumed = evaluate_action(edge, nodes, profile, {"a"})
    assert assumed["modeled_pass"]
    assert assumed["advisories"] == ["UNSUPPORTED_LENGTH"]
