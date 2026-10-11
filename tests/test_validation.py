from pen_art.io import read_json, sha256, write_json
from pen_art.validation import validation_report


def test_summary_keeps_uncalibrated_structure_and_stale_replay_separate(tmp_path):
    write_json(
        tmp_path / "structural_screen.json",
        {"calibrated": False, "modeled_pass": False, "failure": "ILL_CONDITIONED_STRUCTURE"},
    )
    plan = tmp_path / "offline_dual_arm_plan.json"
    write_json(plan, {"inputs": {}, "modeled_duration_s": 2000})
    write_json(
        tmp_path / "offline_dual_arm_validation.json",
        {"status": "CERTIFIED_GEOMETRIC_REPLAY", "source_plan_sha256": sha256(plan)},
    )
    result = validation_report(tmp_path)
    assert result["geometry_status"] == "REPLAY_PASSED"
    assert result["physical_status"] == "NOT_TESTED"
    assert result["checks"][0]["status"] == "UNCALIBRATED"
    assert result["modeled_duration_s"] == 2000  # Advisory target is not a failure gate.
    write_json(plan, {"inputs": {}, "modeled_duration_s": 2100})
    result = validation_report(tmp_path)
    assert result["geometry_status"] == "NOT_VALIDATED"
    assert (
        next(c for c in result["checks"] if c["name"] == "Shared-clock geometric replay")["detail"]
        == "STALE"
    )
    assert read_json(tmp_path / "validation_report.json") == result
    write_json(
        tmp_path / "offline_dual_arm_validation.json",
        {
            "status": "REPLAY_REJECTED",
            "source_plan_sha256": sha256(plan),
            "failures": [{"status": "TIME_BUDGET"}],
        },
    )
    result = validation_report(tmp_path)
    assert (
        next(c for c in result["checks"] if c["name"] == "Shared-clock geometric replay")["status"]
        == "INCOMPLETE"
    )
