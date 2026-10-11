import numpy as np

from pen_art.planning.rrt import connect


def test_rrt_detours_around_obstacle_with_every_edge_checked():
    def valid(a, b):
        delta = b - a
        alpha = np.clip(-a @ delta / max(delta @ delta, 1e-15), 0, 1)
        return np.linalg.norm(a + alpha * delta) > 0.3

    result = connect(
        [-1, 0], [1, 0], [-1.5, -1.5], [1.5, 1.5], valid, seed=17, step_rad=0.2, maximum_seconds=2
    )
    assert result["status"] == "CONNECTED"
    assert result["method"] == "rrt_connect"
    path = np.array(result["positions"])
    np.testing.assert_array_equal(path[0], [-1, 0])
    np.testing.assert_array_equal(path[-1], [1, 0])
    assert all(valid(a, b) for a, b in zip(path[:-1], path[1:], strict=True))
