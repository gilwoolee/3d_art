import numpy as np

from pen_art.planning.curve_motion import search_layers, tracking_edge


def test_layer_search_retains_branch_needed_by_later_target():
    choices = {0: [[0.0], [10.0]], 1: [[1.0], [11.0]], 2: [[12.0]]}

    def solve(target, seeds):
        return choices[target]

    def edge(previous, candidate, p0, p1):
        return 0.0 if abs(candidate[0] - previous[0]) <= 1 else None

    greedy = search_layers([0, 1, 2], solve, edge, [[0.0]], beam_width=1)
    assert greedy["status"] == "NO_CONNECTED_IK_PATH"
    result = search_layers([0, 1, 2], solve, edge, [[0.0]], beam_width=2)
    assert result["status"] == "SAMPLED_DRAWING_PATH"
    assert result["positions"] == [[10.0], [11.0], [12.0]]


def test_matching_endpoints_do_not_hide_intermediate_tcp_drift():
    class NonlinearArm:
        def forward(self, positions, offset):
            pose = np.eye(4)
            pose[0, 3] = positions[0] ** 2
            return pose

    arm = NonlinearArm()
    p0, p1 = np.eye(4), np.eye(4)
    p1[0, 3] = 1
    assert tracking_edge(arm, np.eye(4), np.array([0]), np.array([1]), p0, p1, 0.1, 0.01, 2) is None


def test_retained_endpoint_alternatives_include_their_complete_parent_paths():
    result = search_layers(
        [0, 1, 2],
        lambda target, seeds: [[target], [target + 10]],
        lambda a, b, p0, p1: 0 if abs(a[0] - b[0]) < 2 else None,
        [[0], [10]],
        beam_width=2,
    )
    assert [p["positions"] for p in result["path_alternatives"]] == [
        [[0], [1], [2]],
        [[10], [11], [12]],
    ]
