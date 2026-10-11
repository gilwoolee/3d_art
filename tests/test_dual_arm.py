from pen_art.planning.dual_arm import (
    MotionProposal,
    StrokeTask,
    search_schedule,
)


def task(name, anchor="ground", node=None, cooling=0):
    return StrokeTask(name, frozenset([anchor]), frozenset([node or name]), cooling)


def run(tasks, durations, *, validate=None, arms=("left", "right"), offsets=(), **kwargs):
    def prepare(state, task, arm):
        value = durations.get((task.identity, arm))
        if value is not None:
            yield MotionProposal(
                task.identity,
                arm,
                "default",
                value,
                sorted(task.anchors & state.ready_nodes())[0],
                (0.0,),
            )

    return search_schedule(
        tasks,
        dict.fromkeys(arms, (0.0,)),
        ["ground"],
        prepare,
        validate or (lambda state, new, end: (state.payload, None)),
        finish=lambda state: (state, None),
        start_offsets_s=offsets,
        **kwargs,
    )


def test_independent_strokes_overlap_and_outperform_same_serial_work():
    tasks = [task("a"), task("b")]
    durations = {(t.identity, a): 4 for t in tasks for a in ("left", "right")}
    parallel = run(tasks, durations)
    serial = run(tasks, durations, arms=("left",))
    assert parallel["status"] == serial["status"] == "FOUND"
    assert parallel["state"].time_s == 4
    assert serial["state"].time_s == 8
    assert {m.proposal.arm for m in parallel["state"].history} == {"left", "right"}


def test_support_released_only_after_other_arm_completion_and_cooling():
    tasks = [task("support", node="junction", cooling=3), task("upper", anchor="junction")]
    result = run(tasks, {("support", "left"): 2, ("upper", "right"): 1})
    assert result["status"] == "FOUND"
    support, upper = result["state"].history
    assert support.start_s == 0
    assert upper.start_s == 5
    assert result["state"].time_s == 6


def test_alternative_ownership_avoids_forcing_fixed_partition():
    tasks = [task("a"), task("b")]
    result = run(tasks, {("a", "left"): 20, ("a", "right"): 2, ("b", "left"): 2})
    assigned = {m.proposal.task: m.proposal.arm for m in result["state"].history}
    assert assigned == {"a": "right", "b": "left"}
    assert result["state"].time_s == 2


def test_temporal_conflict_is_resolved_by_offset_not_only_serialization():
    def validate(state, new, end):
        active = state.active + new
        if len(active) == 2 and abs(active[0].start_s - active[1].start_s) < 0.5:
            return None, {"status": "SHARED_JUNCTION_BUSY"}
        return None, None

    result = run(
        [task("a"), task("b")],
        {("a", "left"): 2, ("b", "right"): 2},
        validate=validate,
        offsets=(0.5,),
    )
    assert result["status"] == "FOUND"
    assert result["state"].time_s == 2.5
    assert any(f["status"] == "SHARED_JUNCTION_BUSY" for f in result["failures"])
