"""Essential frame and swept-clearance checks for quarter-turn indexing."""


from dataclasses import replace

import fcl
import numpy as np
import pytest

from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene
from pen_art.planning.turntable import IndexedTurntable, audit_indexed_clock


def turntable():
    return IndexedTurntable(np.eye(4), np.eye(4), 0.25, 0.25, 1.0, 0.12, 0.01, 0.09, 0.18, 0.1)


def test_quarter_turn_rejects_collision_between_clear_endpoints():
    table = turntable()
    material = MaterialScene()
    material.add([0.05, 0, 0.1], [0.05, 0, 0.11], 0.001, "post")
    pose = np.eye(4)
    pose[:3, 3] = [0.05 / np.sqrt(2), 0.05 / np.sqrt(2), 0.105]
    obstacle = placed(fcl.Sphere(0.002), pose)
    assert material.nearest(obstacle)["distance_m"] > 0.001
    assert table.reposition(material, 0, np.pi / 2).nearest(obstacle)["distance_m"] > 0.001
    result = table.check_quarter_turn(material, 0, 1, {"obstacle": obstacle})
    assert result["status"] == "COLLISION"
    assert result["fraction"] == 0.5
    np.testing.assert_allclose(obstacle.getTranslation(), pose[:3, 3])
    pose[:3, 3] = [1, 1, 1]
    result = table.check_quarter_turn(material, 0, 1, {"obstacle": placed(fcl.Sphere(0.002), pose)})
    assert result["status"] == "CERTIFIED_TURNTABLE_INTERVAL"
    with pytest.raises(ValueError, match="quarter-turn"):
        table.check_quarter_turn(material, 0, 2, {})


def test_saved_turn_requires_parked_arm_and_settling():
    table = turntable()
    parked = np.zeros(7)
    duration = table.move_duration()
    path = ClockPath(
        [
            ClockEdge(
                0, duration, parked, parked, "INDEX_TURN", None, table_index=0, table_to_index=1
            ),
            ClockEdge(
                duration,
                duration + table.settling_s,
                parked,
                parked,
                "TABLE_SETTLE",
                None,
                table_index=1,
            ),
        ]
    )
    restored = ClockPath.from_record(path.record())
    assert audit_indexed_clock(table, restored, parked, 2)["final_index"] == 1
    with pytest.raises(ValueError, match="settles"):
        audit_indexed_clock(table, ClockPath([path.edges[0]]), parked, 2)
    record = path.record()
    record["edges"][0]["q1"][0] = 0.1
    with pytest.raises(ValueError, match="stationary"):
        ClockPath.from_record(record)
    with pytest.raises(ValueError, match="parking"):
        audit_indexed_clock(table, restored, np.ones(7), 2)
    shifted = [replace(edge, start_s=edge.start_s + 1, end_s=edge.end_s + 1) for edge in path.edges]
    feeding = ClockPath(
        [
            ClockEdge(0, 1, parked, parked, "PEN_START_WAIT", None, table_index=0),
            *shifted,
        ]
    )
    with pytest.raises(ValueError, match="pen, parking, cooling"):
        audit_indexed_clock(table, feeding, parked, 2)
    cooling = ClockPath(
        [
            ClockEdge(0, 0.5, parked, parked, "PEN_START_WAIT", None, table_index=0),
            ClockEdge(0.5, 1, parked, parked, "PEN_STOP_WAIT", None, table_index=0),
            *shifted,
        ]
    )
    with pytest.raises(ValueError, match="cooling"):
        audit_indexed_clock(table, cooling, parked, 2)
