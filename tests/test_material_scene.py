import fcl
import numpy as np
import pytest

from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene


def test_incremental_scene_nearest_query_and_capsule_orientation():
    scene = MaterialScene()
    probe = placed(fcl.Sphere(0.05), np.eye(4))
    assert scene.nearest(probe) is None
    scene.add([1, 0, 0], [1, 1, 0], 0.05, "far")
    assert scene.nearest(probe)["name"] == "far"
    assert scene.nearest(probe)["distance_m"] == pytest.approx(0.9, abs=1e-6)
    scene.add([-0.1, 0, 0], [0.1, 0, 0], 0.01, "newly-deposited")
    assert scene.nearest(probe)["name"] == "newly-deposited"
    assert scene.nearest(probe)["distance_m"] <= 0
    clone = scene.copy()
    assert clone.nearest(probe)["name"] == "newly-deposited"
    assert len(clone.segments) == 2
    assert not clone.possibly_near([5, 0, 0], 0.1, 0.01)
    assert clone.possibly_near([0, 0, 0], 0.1, 0)
    # Equality cannot be culled; include capsule radius and the entire body.
    assert clone.possibly_near([1.16, 0, 0], 0.1, 0.01)
    assert not clone.possibly_near([1.17, 0, 0], 0.1, 0.01)

    # A long, spatially ordered insertion sequence must keep every capsule in
    # the balanced broadphase and agree with exhaustive narrowphase distance.
    ordered = MaterialScene()
    for index in range(64):
        ordered.add([index * 0.01, 1, 0], [(index + 1) * 0.01, 1, 0], 0.002, str(index))
    transform = np.eye(4)
    transform[:3, 3] = [0.73, 1, 0]
    near_end = placed(fcl.Sphere(0.005), transform)
    expected = min(
        fcl.distance(near_end, obj, fcl.DistanceRequest(), fcl.DistanceResult())
        for obj in ordered.objects
    )
    assert ordered.nearest(near_end)["distance_m"] == pytest.approx(expected, abs=1e-10)
    ordered.add([0.73, 0.99, 0], [0.73, 1.01, 0], 0.002, "later")
    assert ordered.nearest(near_end)["name"] == "later"
    assert ordered.nearest(near_end)["distance_m"] <= 0

    # A long link can have an overlapping enclosing sphere while its actual
    # body clears the material box. The box is only a sufficient certificate;
    # equality, overlap, and material added after a query must still be checked.
    long_body = placed(fcl.Box(4, 0.1, 0.1), np.eye(4))
    assert ordered.possibly_near([0, 0, 0], 2.01, 0.01)
    assert not ordered.possibly_near_body(long_body, 0.01)
    assert ordered.possibly_near_body(long_body, 1)
    ordered.add([0, 0, 0], [0, 0, 0], 0.01, "inside-body")
    assert ordered.possibly_near_body(long_body, 0)


def test_contact_clipping_keeps_material_outside_local_ball():
    scene = MaterialScene()
    scene.add([-1, 0, 0], [1, 0, 0], 0.01, "strand")
    clipped = scene.outside_ball([0, 0, 0], 0.1)
    probe = placed(fcl.Sphere(0.001), np.eye(4))
    assert clipped.nearest(probe)["distance_m"] > 0
    transform = np.eye(4)
    transform[0, 3] = 0.2
    assert clipped.nearest(placed(fcl.Sphere(0.001), transform))["distance_m"] <= 0
    assert len(clipped.segments) == 2
