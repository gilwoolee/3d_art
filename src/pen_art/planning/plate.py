"""The fixed build plate the object stands on, and the feed audit of a stroke.

The plate lies on the table and never moves; the arm reaches around the
object. Clocks keep a ``table_index`` field, which is always 0.
"""

import numpy as np


class FixedPlate:
    home_index = 0

    def __init__(self, world_from_build):
        self.world_from_build = np.asarray(world_from_build, float)

    @classmethod
    def from_profile(cls, profile):
        return cls(profile["world_from_build"])

    def build_transform(self, index=0):
        if index != 0:
            raise ValueError("A fixed build plate has one position")
        return self.world_from_build


def audit_plate_clock(path):
    """Check the pen feed of one saved clock: no drawing with the feed off, and
    a stroke chained to the next one still feeding when it ends. Cooling holds
    are audited with the process clock."""
    feeding = bool(path.metadata.get("chained_in"))
    for edge in path.edges:
        if edge.table_index not in (None, 0) or edge.table_to_index is not None:
            raise ValueError("A fixed build plate never turns")
        if edge.kind == "PEN_START_WAIT":
            feeding = True
        elif edge.kind == "PEN_STOP_WAIT":
            feeding = False
        elif edge.kind == "DRAW" and not feeding:
            raise ValueError("Drawing with feed off")
    if feeding != bool(path.metadata.get("chained_out")):
        raise ValueError("Clock ends with the pen feeding but nothing chained after it")
    return {"initial_index": 0, "final_index": 0, "turns": 0}
