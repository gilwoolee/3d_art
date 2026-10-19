"""The records a build is scheduled with: strokes, their motions and the state.

The planner validates every elapsed interval, including partial deposition.
Nothing here is collision validation.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class StrokeTask:
    identity: str
    anchors: frozenset[str]
    deposits: frozenset[str]
    cooling_s: float = 0.0


@dataclass(frozen=True)
class MotionProposal:
    """A complete travel/approach/draw/withdraw alternative.

    ``payload`` belongs to the backend (e.g. exact timed joint polynomials).
    ``start_anchor`` must already be cool when dispatched. End configurations
    are used only after the entire action finishes. Different path/orientation
    alternatives must have different variant IDs.
    """

    task: str
    arm: str
    variant: str
    duration_s: float
    start_anchor: str
    end_configuration: tuple[float, ...]
    payload: object = field(default=None, compare=False, repr=False)
    auxiliary: bool = False


@dataclass(frozen=True)
class ScheduledMotion:
    proposal: MotionProposal
    start_s: float

    @property
    def end_s(self):
        return self.start_s + self.proposal.duration_s


@dataclass(frozen=True)
class ScheduleState:
    time_s: float
    configurations: tuple[tuple[str, tuple[float, ...]], ...]
    completed: frozenset[str]
    support_ready: tuple[tuple[str, float], ...]
    active: tuple[ScheduledMotion, ...] = ()
    history: tuple[ScheduledMotion, ...] = ()
    payload: object = field(default=None, compare=False, repr=False)

    def ready_nodes(self):
        return frozenset(node for node, ready in self.support_ready if ready <= self.time_s)


def tasks_from_construction(construction, graph, cooling_s=0.0):
    """Extract alternative anchors, without importing the serial chosen order.

    Readiness is only a necessary geometric precondition. The backend must check
    support/stability along the proposed direction and growing material state.
    """
    tasks = []
    for action in construction["actions"]:
        deposits = frozenset(v["node"] for v in action["junction_visits"])
        # A closed curve may start at any junction, an open one at either end.
        anchors = deposits if action["closed"] else frozenset([action["start"], action["end"]])
        if not anchors or not anchors <= deposits:
            raise ValueError("Every alternative anchor must lie on its stroke")
        tasks.append(StrokeTask(action["id"], anchors, deposits, cooling_s))
    ground = frozenset(node["id"] for node in graph["nodes"] if node["ground"])
    return tuple(tasks), ground
