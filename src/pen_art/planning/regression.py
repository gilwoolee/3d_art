"""Lazy backward construction search over material subsets and robot configurations.

The motion callback yields alternative *forward* section-and-transit witnesses.
Its returned configuration is the beginning of that witness, before extrusion.
It may yield None after bounded failed work so another source gets a turn.
Only structural results may be cached by material subset.
"""

from time import monotonic


def greedy_forward_order(actions, ground, stable, *, deadline=float("inf")):
    """Keep suggested priorities when support and cooled stiffness permit.

    This chooses one complete geometric order before motion sampling. It does
    not backtrack earlier order choices or prove impossibility on failure.
    """
    remaining = sorted(actions, key=lambda row: (row["priority"], row["id"]))
    available, built, order = set(ground), set(), []
    while remaining and monotonic() < deadline:
        framework_pending = any(row["stage"] == "framework" for row in remaining)
        for row in remaining:
            if monotonic() >= deadline:
                return None
            if framework_pending and row["stage"] != "framework":
                continue
            anchors = (
                {visit["node"] for visit in row["junction_visits"]}
                if row.get("closed")
                else {row["start"], row["end"]}
            )
            if available.intersection(anchors) and stable(built | {row["id"]}):
                break
        else:
            return None
        remaining.remove(row)
        order.append(row)
        built.add(row["id"])
        available.update((row["start"], row["end"]))
        available.update(visit["node"] for visit in row.get("junction_visits", []))
    return None if remaining else order
