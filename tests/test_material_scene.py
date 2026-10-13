import fcl
import numpy as np
import pytest

from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene


def test_local_contact_geometry_preserves_raw_exact_queries_at_clearance_boundary():
    scene = MaterialScene()
    for i in range(80):
        scene.add([i * 0.02, 1, 0], [i * 0.02 + 0.008, 1, 0], 0.001, str(i))
    scene.add([-0.01, 0, 0], [0.05, 0, 0], 0.001, "join")
    scene.clearance_query_enabled = False
    local = scene.contact_view([0, 0, 0], 0.01, local=True)
    assert not local.clearance_query_enabled
    assert type(scene.contact_view([0, 0, 0], 0.01, local=False)) is MaterialScene
    for generation in range(2):
        reference = scene.outside_ball([0, 0, 0], 0.01)
        for gap in (0.000999, 0.001, 0.001001, -0.001):
            transform = np.eye(4)
            transform[:3, 3] = [0.03, 0, 0.002 + gap]
            body = placed(fcl.Sphere(0.001), transform)
            expected = reference.nearest(body)
            observed = local.nearest(body)
            assert observed["distance_m"] == pytest.approx(expected["distance_m"], abs=1e-8)
            assert (local.within_clearance(body, 0.001) is None) == (
                reference.within_clearance(body, 0.001) is None
            )
        if generation == 0:
            scene.add([0.03, 0, 0.003], [0.03, 0, 0.003], 0.001, "new obstruction")
    assert scene.clearance_stats["contact_views_refreshed"] == 2
    assert scene.clearance_stats["exact_nearest_queries"] > 0
