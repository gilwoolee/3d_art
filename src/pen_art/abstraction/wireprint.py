"""Horizontal contours and ordered connecting zigzags in the physical build frame.

Each zigzag is a vertical riser from the lower contour followed by a slanted
stroke back down to it, as in WirePrint.

This is a WirePrint-style generator, not a calibrated printer implementation.
The returned topology and order still require structural and robot validation.
"""

from pathlib import Path
from typing import NamedTuple

import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize, unary_union

from ..graph import base_anchors, connect_components, straight_edge
from ..io import artifact, read_json, sha256, write_json

# Geometric resolution: seams are chosen and junctions merged on this grid.
JUNCTION_M = 0.00002
# Diagonals are kept this far under the descent limit, to allow for the merge.
DESCENT_MARGIN_DEG = 1
# A point this close to the bed is on it.
ON_BED_M = 1e-7


def sample_loop(loop, fraction):
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))]
    location = (fraction % 1.0) * distances[-1]
    index = min(np.searchsorted(distances, location, side="right") - 1, len(loop) - 2)
    alpha = (location - distances[index]) / (distances[index + 1] - distances[index])
    return loop[index] * (1 - alpha) + loop[index + 1] * alpha


def align_loop(loop, reference):
    """Start an upper contour at the nearest point to its parent's seam."""
    a, delta = loop[:-1], np.diff(loop, axis=0)
    fractions = np.clip(
        np.sum((reference - a)[:, :2] * delta[:, :2], axis=1)
        / np.maximum(np.sum(delta[:, :2] ** 2, axis=1), 1e-30),
        0,
        1,
    )
    projections = a + fractions[:, None] * delta
    index = int(np.argmin(np.linalg.norm(projections[:, :2] - reference[:2], axis=1)))
    seam = projections[index]
    vertices = np.vstack([seam, loop[index + 1 : -1], loop[: index + 1], seam])
    keep = np.r_[True, np.linalg.norm(np.diff(vertices, axis=0), axis=1) > 1e-10]
    return vertices[keep]


def _layer_heights(
    mesh, layer_height, descent_deg, connectors, minimum_contour, coil_width, floor=0.003
):
    """Slice heights: ``layer_height`` apart, thinner where a loop is short.

    Between two loops a zigzag spreads its diagonals around the loop, each
    sloping no steeper than the descent limit. A loop of perimeter P therefore
    holds ``connectors`` of them only when the layer is at most
    P / connectors * tan(limit) high. Bands containing such a short loop get
    thinner layers (never below ``floor``); the rest keep ``layer_height``.
    Loops left out (too small) or wound as coils do not count.
    """
    low, high = mesh.bounds[:, 2]
    uniform = np.linspace(low, high, int(np.ceil((high - low) / layer_height)) + 1)
    if not connectors:
        return uniform
    slope = np.tan(np.radians(descent_deg - DESCENT_MARGIN_DEG))
    margin = min(0.000001, (high - low) / 1000)

    def shortest(z):
        section = mesh.section(
            plane_origin=[0, 0, float(np.clip(z, low + margin, high - margin))],
            plane_normal=[0, 0, 1],
        )
        lengths = []
        for points in [] if section is None else section.discrete:
            xy = np.asarray(points)[:, :2]
            length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
            width = float(np.ptp(xy, axis=0).max())
            if length < np.pi * minimum_contour or (coil_width and width <= coil_width):
                continue
            lengths.append(length)
        return min(lengths, default=np.inf)

    heights, z = [low], low
    while high - z > 1e-9:
        h = layer_height
        for _ in range(4):
            need = min(shortest(z), shortest(min(high, z + h))) / connectors * slope
            thinner = min(layer_height, max(floor, need))
            if thinner >= h - 1e-9:
                break
            h = thinner
        z = high if high - (z + h) < 0.25 * h else z + h
        heights.append(z)
    return np.asarray(heights)


def _validate(mesh, layer_height, zigzag_spacing, diameter, slice_simplification, minimum_contour):
    """Reject settings and meshes the generator cannot slice."""
    if min(layer_height, zigzag_spacing, diameter, slice_simplification) <= 0:
        raise ValueError("Positive layer, zigzag, diameter and simplification are required")
    if not np.isfinite(minimum_contour) or minimum_contour < 0:
        raise ValueError("Minimum contour size must be finite and nonnegative")
    low, high = mesh.bounds[:, 2]
    if high <= low:
        raise ValueError("Mesh must have positive height")


def _section(mesh, height):
    """Closed outlines where the plane at ``height`` cuts the mesh."""
    low, high = mesh.bounds[:, 2]
    # Stay beyond trimesh's plane/vertex tolerance at the extrema. Projecting
    # these contours to the end plane moves them by at most one micron.
    end_offset = min(0.000001, (high - low) / 1000)
    cut_height = np.clip(height, low + end_offset, high - end_offset)
    # A plane through mesh vertices or along flat faces can cut a closed
    # surface into open pieces. Nudge the cut by a few microns before
    # concluding that the surface itself is open.
    for nudge in (0.0, 2e-6, -2e-6, 5e-6, -5e-6, 1e-5, -1e-5):
        nudged = np.clip(cut_height + nudge, low + end_offset, high - end_offset)
        section = mesh.section(plane_origin=[0, 0, nudged], plane_normal=[0, 0, 1])
        paths = [] if section is None else section.discrete
        if all(np.linalg.norm(points[0] - points[-1]) <= 1e-6 for points in paths):
            break
    return paths


def _simple_rings(paths, slice_simplification, layer):
    """Simplified outlines as simple rings, crossings split into separate rings."""
    rings = []
    for points in paths:
        if np.linalg.norm(points[0] - points[-1]) > 1e-6:
            raise ValueError(f"Open contour at layer {layer}; repair the input mesh")
        original = LineString(points[:, :2])
        simplified = original.simplify(slice_simplification)
        if not Polygon(simplified).is_valid:
            # Node real crossings into simple rings, retaining every unique
            # boundary segment. Buffer-based repair can erase small features.
            linework = unary_union(simplified)
            polygons = list(polygonize(linework))
            boundaries = unary_union([polygon.boundary for polygon in polygons])
            lost_length = linework.difference(boundaries).length
            if not polygons or lost_length > 1e-9:
                raise ValueError(f"Unclosed slice linework at layer {layer}: {lost_length} m")
            unique = {}
            for polygon in polygons:
                for ring in (polygon.exterior, *polygon.interiors):
                    key = Polygon(ring).normalize().wkb
                    unique[key] = np.asarray(ring.coords)
            rings.extend(unique.values())
        else:
            rings.append(np.asarray(simplified.coords))
    return rings


def _contour_loops(rings, height, minimum_contour, layer):
    """Counterclockwise loops at ``height`` with a repeatable seam, sorted by nesting depth."""
    loops = []
    for xy in rings:
        if len(xy) < 4:
            continue
        polygon = Polygon(xy)
        if not polygon.is_valid:
            raise ValueError(f"Self-intersecting slice at layer {layer}")
        if polygon.area < 1e-12:
            continue
        # A rounded tip narrows to nothing, and a contour too small to carry
        # zigzags within the descent angle would thin every layer of the
        # object. End the shape at the last contour of printable size.
        if polygon.length < np.pi * minimum_contour:
            continue
        # Consistent counterclockwise winding and a repeatable geometric seam.
        signed = np.sum(xy[:-1, 0] * xy[1:, 1] - xy[1:, 0] * xy[:-1, 1])
        vertices = xy[:-1] if signed > 0 else xy[:-1][::-1]
        # Near-equal X coordinates must not randomly choose different
        # square corners on neighboring slices because of roundoff.
        quantized = np.rint(vertices / JUNCTION_M)
        seam = np.lexsort((quantized[:, 1], quantized[:, 0]))[0]
        vertices = np.roll(vertices, -int(seam), axis=0)
        closed = np.vstack([vertices, vertices[0]])
        loop = np.column_stack([closed, np.full(len(closed), height)])
        loops.append({"points": loop, "polygon": polygon, "joins": {0.0}})
    for loop in loops:
        point = loop["polygon"].boundary.interpolate(0)
        loop["depth"] = sum(
            other["polygon"].contains(point) for other in loops if other is not loop
        )
    loops.sort(key=lambda loop: (loop["depth"], *loop["points"][0, :2]))
    return loops


def _slice_layers(mesh, heights, slice_simplification, minimum_contour):
    """The contour loops of each layer, bottom up."""
    return [
        _contour_loops(
            _simple_rings(_section(mesh, height), slice_simplification, layer),
            height,
            minimum_contour,
            layer,
        )
        for layer, height in enumerate(heights)
    ]


def _add_base_ring(layers, low):
    """Copy the lowest contours onto the bed when the shape starts above it.

    A shape with a rounded underside has no contour on the bed: its lowest
    one starts a layer up. Posts under that contour would be the only thing
    holding the object, each stuck to the bed at a point. Draw the same
    outline flat on the bed instead, so the usual band of risers and
    diagonals carries the object on a base that grips along its whole length.
    """
    first = next((index for index, loops in enumerate(layers) if loops), None)
    if not first:
        return layers
    base = []
    for loop in layers[first]:
        points = loop["points"].copy()
        points[:, 2] = low
        base.append({**loop, "points": points, "joins": {0.0}})
    return [*layers[: first - 1], base, *layers[first:]]


def _overlaps(loop, other):
    """Whether two loops of neighbouring layers are the same wall, one on the other."""
    return loop["depth"] == other["depth"] and loop["polygon"].intersects(other["polygon"])


def _prune_unsupported(layers):
    """Leave out cavity walls and contours that begin in mid-air.

    An inner contour that begins above the bed is the wall of a cavity, such
    as the inside of a vase: a ring with nothing beneath it. Print the outer
    shell only. Removing it bottom-up also removes the contours it would have
    carried. Inner contours that start on the bed, like a torus's hole, stay.
    An outer contour that begins in mid-air, above the object's lowest
    contour, has nothing to be drawn onto either: an overhanging tip that
    joins the shape a layer or two up. It is left out the same way, and the
    contour it joins covers that part of the shape.
    """
    kept_layers = layers[:1]
    started = bool(layers and layers[0])
    for index in range(1, len(layers)):
        kept = []
        for up in layers[index]:
            carried = any(_overlaps(up, down) for down in kept_layers[index - 1])
            if not carried and (up["depth"] or started):
                continue
            kept.append(up)
        kept_layers.append(kept)
        started = started or bool(kept)
    return kept_layers


def _narrow(loop, maximum_width):
    """Whether an outer loop is narrow enough to be wound as a coil."""
    x0, y0, x1, y1 = loop["polygon"].bounds
    return not loop["depth"] and max(x1 - x0, y1 - y0) <= maximum_width


def _touching(loop, others):
    return [other for other in others if _overlaps(other, loop)]


def _narrow_run(layers, index, loop, maximum_width):
    """The run of narrow loops ending at ``loop``, and the layer it starts on."""
    chain, level = [loop], index
    while level > 0:
        below = _touching(chain[0], layers[level - 1])
        if (
            len(below) != 1
            or not _narrow(below[0], maximum_width)
            or len(_touching(below[0], layers[level])) != 1
        ):
            break
        chain.insert(0, below[0])
        level -= 1
    return chain, level


def _find_coil_towers(layers, low, maximum_width, diameter, pitch):
    """Take narrow runs of outlines out of the layers, to be wound as coils.

    A coil tower: a part too narrow for loops and zigzags, such as a raised
    arm, is wound as one strand that climbs by its own thickness each turn,
    so the turns touch and form a thin tube. It applies to a run of narrow
    outlines that ends without widening again, standing on one wider outline
    or on the bed. Those outlines leave the layers and are wound instead.
    """
    if not maximum_width:
        return layers, []
    towers, wound = [], set()
    for index in range(len(layers) - 1, -1, -1):
        for loop in layers[index]:
            # It ends, or everything it carries is itself wound.
            ends = index + 1 == len(layers) or all(
                id(above) in wound for above in _touching(loop, layers[index + 1])
            )
            if id(loop) in wound or not _narrow(loop, maximum_width) or not ends:
                continue
            chain, level = _narrow_run(layers, index, loop, maximum_width)
            support = _touching(chain[0], layers[level - 1]) if level else []
            grounded = abs(chain[0]["points"][0, 2] - low) < ON_BED_M
            if len(support) != 1 and not (grounded and len(chain) > 1):
                continue
            # The first layer leans from the supporting outline to the
            # tower. More than two strand widths per turn and its turns
            # would hang in air, so such a part is not wound.
            if support and not grounded:
                below = LineString(support[0]["points"][:, :2])
                reach = max(below.distance(Point(p)) for p in chain[0]["points"][:, :2])
                rise = chain[0]["points"][0, 2] - support[0]["points"][0, 2]
                if reach > 2 * diameter / pitch * rise:
                    continue
            towers.append(
                {
                    "level": level,
                    "loops": chain,
                    "support": None if grounded or not support else support[0],
                }
            )
            wound.update(id(member) for member in chain)
    layers = [[loop for loop in loops if id(loop) not in wound] for loops in layers]
    return layers, towers


class _Pair(NamedTuple):
    """A lower and an upper contour joined by one zigzag."""

    down: dict
    up: dict
    beneath: LineString
    overhead: LineString
    chained: bool


def _anchors(pair, u, v, nearest, count):
    """Foot, top and next foot of one zigzag: a riser and a diagonal.

    ``u`` and ``v`` are how far round the loop this riser and the
    next one stand, as fractions from the zigzag's starting point.

    As in WirePrint, the riser goes straight up, or as near to it as
    the two outlines allow, and only the stroke back down is
    slanted. Like outlines space the feet evenly on the lower
    contour and put each top at the nearest point above. Where the
    outlines differ the tops are spaced evenly on the upper contour
    and each foot is the nearest point beneath.
    """
    down, up, beneath, overhead, chained = pair
    # A zigzag starts where the zigzag beneath its lower contour
    # put its last riser top, so the pen can run from that zigzag
    # round the contour and straight into this one. With equal
    # counts the risers then also line up from layer to layer.
    # Otherwise an inner wall can stand on almost the same ring as
    # the outer wall beside it (a torus resting on the bed). Start
    # its zigzags half a spacing along, so they do not share feet.
    origin = down["chain"] if chained else (0.5 / count if down["depth"] % 2 else 0.0)
    if nearest:
        start = (
            overhead.project(Point(sample_loop(down["points"], origin)[:2]), normalized=True)
            if chained
            else origin
        )
        first, following = start + u, start + v
        a, c = (
            beneath.project(Point(sample_loop(up["points"], f)[:2]), normalized=True)
            for f in (first, following)
        )
        return (origin if chained and u == 0 else a), first, c
    first, following = origin + u, origin + v
    foot = Point(sample_loop(down["points"], first)[:2])
    top = overhead.project(foot, normalized=True)
    return first, top, following


def _descent(pair, b, c):
    """Degrees below horizontal of the diagonal from top ``b`` to foot ``c``."""
    fall = sample_loop(pair.up["points"], b) - sample_loop(pair.down["points"], c)
    return np.degrees(np.arctan2(fall[2], np.linalg.norm(fall[:2])))


def _steepest(pair, count, nearest):
    return max(
        _descent(pair, *_anchors(pair, i / count, (i + 1) / count, nearest, count)[1:])
        for i in range(count - 1)
    )


def _pair_contours(lower, upper):
    """Pairs of overlapping lower and upper contours; each upper seam follows its main parent."""
    pairs = []
    for ui, up in enumerate(upper):
        parents = [li for li, down in enumerate(lower) if _overlaps(up, down)]
        for li in parents:
            pairs.append((li, ui))
        if parents:
            parent = max(
                parents, key=lambda li: up["polygon"].intersection(lower[li]["polygon"]).area
            )
            up["points"] = align_loop(up["points"], lower[parent]["points"][0])
    return pairs


def _riser_count(pair, lower, co_parents, branching, spacing, limit, minimum_connectors):
    """How many risers the zigzag has, and whether they stand on the nearest part beneath."""
    perimeter = max(LineString(loop["points"]).length for loop in (pair.down, pair.up))
    fewest = max(3, minimum_connectors or 3)
    count = max(fewest, int(np.ceil(perimeter / spacing)))
    above = max(fewest, int(np.ceil(LineString(pair.up["points"]).length / spacing)))
    # Two contours of different outline can also pair badly by equal
    # fractions. Use the nearest part beneath when that alone brings
    # the diagonals within the limit; like outlines keep equal fractions.
    # A degree of margin covers the later 20 micron junction merge.
    nearest = branching or (
        _steepest(pair, count, False) > limit
        and _steepest(pair, above, True) <= limit - DESCENT_MARGIN_DEG
    )
    if nearest:
        count = above
    if branching and len(co_parents) > 1 and minimum_connectors:
        # Contours merging into one (pillars into a roof): risers stand
        # round the upper contour and go to the nearest one beneath.
        # Space them so each merging contour gets at least its share.
        narrowest = min(LineString(lower[k]["points"]).length for k in co_parents)
        length = LineString(pair.up["points"]).length
        count = max(count, int(np.ceil(length * fewest / narrowest)))
    return count, nearest


def _risers(pair, count, nearest, stretch, limit):
    """Yield foot, top, next foot and whether it is the last, for each riser round the loop."""
    # Where equal spacing would make a diagonal too steep, its foot and
    # the riser that follows stand further round the loop, far enough
    # for the diagonal to come down within the limit.
    i, shift = 0, 0.0
    while i / count + shift < 1 - 1e-9:
        u, v = i / count + shift, (i + 1) / count + shift
        ended = False
        a, b, c = _anchors(pair, u, v, nearest, count)
        if stretch and v < 1 - 1e-9 and _descent(pair, b, c) > limit - DESCENT_MARGIN_DEG:
            reach = v
            while (
                reach < min(u + 0.75, 1 - 0.5 / count)
                and _descent(pair, b, c) > limit - DESCENT_MARGIN_DEG
            ):
                reach += 0.125 / count
                a, b, c = _anchors(pair, u, reach, nearest, count)
            if _descent(pair, b, c) <= limit - DESCENT_MARGIN_DEG:
                shift += reach - v
                v = reach
            else:
                # No foot before the loop closes is far enough: this
                # riser ends the zigzag, as the closing one would.
                ended = reach >= 1 - 0.5 / count
                a, b, c = _anchors(pair, u, v, nearest, count)
        # A riser that would stand within half a spacing of the first
        # one is left out; this riser is then the zigzag's last.
        last = ended or v >= 1 - 1e-9 or bool(stretch and shift and v > 1 - 0.5 / count)
        if last:
            shift = 1.0
        i += 1
        yield a, b, c, last


def _zigzag(lower, upper, li, ui, pairs, spacing, limit, minimum_connectors):
    """Foot, top, next foot (or None) and step of each riser from ``li`` up to ``ui``."""
    down, up = lower[li], upper[ui]
    # Where contours split or merge, equal fractions pair unrelated
    # points. Stand each upper contour on the nearest part of the
    # contour beneath it instead.
    co_parents = [parent for parent, u in pairs if u == ui]
    branching = len(co_parents) > 1 or sum(parent == li for parent, _ in pairs) > 1
    # Only one zigzag can begin at the lower contour's chain point; a
    # sibling starting there would stand on the first one's riser.
    takes_chain = "chain" in down and down.setdefault("chain_user", ui) == ui
    pair = _Pair(
        down,
        up,
        LineString(down["points"][:, :2]),
        LineString(up["points"][:, :2]),
        takes_chain,
    )
    count, nearest = _riser_count(
        pair, lower, co_parents, branching, spacing, limit, minimum_connectors
    )
    rows = []
    for a, b, c, last in _risers(pair, count, nearest, not branching, limit):
        if branching:
            top = Point(sample_loop(up["points"], b)[:2])
            if li != min(co_parents, key=lambda k: lower[k]["polygon"].distance(top)):
                continue
        # WirePrint omits the closing diagonal to leave print-head
        # access. With one anchor this also avoids retracing the riser.
        if last:
            c = None
        # A diagonal that still comes out too steep is left out rather
        # than thinning every layer; its riser still stands.
        if c is not None and _descent(pair, b, c) > limit - DESCENT_MARGIN_DEG:
            c = None
        down["joins"].add(a % 1.0)
        if c is not None:
            down["joins"].add(c % 1.0)
        up["joins"].add(b % 1.0)
        # The last riser top, where the next zigzag up will begin.
        up["chain"] = b % 1.0
        rows.append((a, b, c, len(rows)))
    return rows


def _plan_zigzags(layers, spacing, limit, minimum_connectors):
    """Every riser between neighbouring layers, as (layer, lower, upper, a, b, c, step)."""
    bridges = []
    for index in range(1, len(layers)):
        lower, upper = layers[index - 1], layers[index]
        pairs = _pair_contours(lower, upper)
        for li, ui in pairs:
            rows = _zigzag(lower, upper, li, ui, pairs, spacing, limit, minimum_connectors)
            bridges.extend((index, li, ui, *row) for row in rows)
    return bridges


def _grid_pieces(inside, spacing):
    """Straight lines across ``inside`` along x (0) and y (1), ``spacing`` apart."""
    bounds = np.reshape(inside.bounds, (2, 2))
    pieces = {0: [], 1: []}
    for axis in (0, 1):
        across = 1 - axis
        count = int((bounds[1, across] - bounds[0, across]) // spacing)
        for row in range(count):
            level = bounds[:, across].mean() + (row - (count - 1) / 2) * spacing
            ends = np.zeros((2, 2))
            ends[:, axis] = bounds[0, axis] - 1, bounds[1, axis] + 1
            ends[:, across] = level
            crossing = inside.intersection(LineString(ends))
            for piece in getattr(crossing, "geoms", [crossing]):
                if piece.geom_type == "LineString" and piece.length >= spacing:
                    pieces[axis].append(np.asarray([piece.coords[0], piece.coords[-1]]))
    return pieces


def _base_grid(layers, low, spacing):
    """Grid lines across the outlines on the bed, and the index of that layer.

    A grid of straight lines across each outline that lies on the bed. It
    ties the base together both ways and gives it more length gripping the
    bed than the outline alone. Lines meet at shared points, as a contour
    meets the riser tops it is drawn across.
    """
    lines = []
    ground = next((index for index, loops in enumerate(layers) if loops), None)
    if not (
        spacing and ground is not None and abs(layers[ground][0]["points"][0, 2] - low) < ON_BED_M
    ):
        return lines, ground
    loops = layers[ground]
    rings = [LineString(loop["points"][:, :2]) for loop in loops]
    inside = unary_union([loop["polygon"] for loop in loops if loop["depth"] % 2 == 0])
    holes = [loop["polygon"] for loop in loops if loop["depth"] % 2]
    if holes:
        inside = inside.difference(unary_union(holes))
    pieces = _grid_pieces(inside, spacing)
    for axis in (0, 1):
        for piece in pieces[axis]:
            for point in piece:
                li = min(range(len(loops)), key=lambda k: rings[k].distance(Point(point)))
                loops[li]["joins"].add(rings[li].project(Point(point), normalized=True) % 1.0)
            stops = [piece[0], piece[1]]
            for other in pieces[1 - axis]:
                meet = np.array(other[0])
                meet[axis] = other[0][axis]
                meet[1 - axis] = piece[0][1 - axis]
                if (
                    piece[:, axis].min() < meet[axis] < piece[:, axis].max()
                    and other[:, 1 - axis].min() < meet[1 - axis] < other[:, 1 - axis].max()
                ):
                    stops.append(meet)
            stops.sort(key=lambda point: point[axis])
            lines.append([np.r_[point, low] for point in stops])
    return lines, ground


def _build_order(layers, bridges):
    """Order hint of each stroke, keyed (layer, "zigzag", li, ui) or (layer, "contour", ui).

    Build order within a layer: each contour straight after the zigzags that
    carry it, starting with a loop that stands on the contour finished last,
    so that consecutive strokes meet end to start wherever the shape allows.
    Loops that share a lower contour are the exception: a finished contour
    would stand in the pen's way to its sibling's feet just beneath, so all
    of their zigzags come first and their contours after.
    """
    turns, last = {}, None
    for index, loops in enumerate(layers):
        pairs = [(li, ui) for layer, li, ui, *_ in bridges if layer == index]
        supported = {ui for li, ui in pairs if li == last}
        children = {li: {u for parent, u in pairs if parent == li} for li, _ in pairs}
        sequence, held = [], []
        for ui in sorted(range(len(loops)), key=lambda ui: (ui not in supported, ui)):
            parents = sorted({li for li, u in pairs if u == ui}, key=lambda li: (li != last, li))
            sequence.extend(("zigzag", li, ui) for li in parents)
            if any(len(children[li]) > 1 for li in parents):
                held.append(("contour", ui))
            else:
                sequence.append(("contour", ui))
            last = ui
        for position, stroke in enumerate(sequence + held):
            turns[(index, *stroke)] = 1000 * index + position
        if held:
            last = held[-1][1]
    return turns


class _Wire:
    """The frame's nodes and straight members, as they are drawn."""

    def __init__(self, low, diameter):
        self.low, self.diameter = low, diameter
        self.nodes, self.edges, self.positions, self.by_edge = {}, [], [], {}

    def node(self, point):
        """The id of the node at ``point``, merged with any node within ``JUNCTION_M``."""
        # Plane intersections close to mesh vertices can leave micron-long
        # slivers. Merge them at the same declared geometric resolution used by
        # simplification, before defining source members or mechanical beams.
        if self.positions:
            distances = np.linalg.norm(np.asarray(self.positions) - point, axis=1)
            nearest = int(np.argmin(distances))
            if distances[nearest] <= JUNCTION_M:
                return f"n{nearest:05d}"
        identity = f"n{len(self.nodes):05d}"
        self.positions.append(np.asarray(point))
        self.nodes[identity] = {
            "id": identity,
            "position": np.asarray(point).tolist(),
            "ground": bool(abs(point[2] - self.low) < ON_BED_M),
        }
        return identity

    def add(self, a, b, source, interval, order):
        """Add a member from ``a`` to ``b``, or record a second source of an existing one."""
        first, second = self.node(a), self.node(b)
        if first == second:
            return
        key = tuple(sorted((first, second)))
        if key in self.by_edge:
            self.by_edge[key].setdefault("shared_sources", []).append(
                {
                    "parent_curve": source,
                    "source_interval": interval,
                }
            )
            return
        edge = straight_edge(
            first, second, self.nodes, f"member-{len(self.edges):05d}", "framework", self.diameter
        )
        edge.update(
            origin="wireprint",
            parent_curve=source,
            source_interval=interval,
            stage="framework",
            order_hint=order,
        )
        self.by_edge[key] = edge
        self.edges.append(edge)


def _coil_stack(tower, tops):
    """The outlines a tower's strand climbs through, from where it leaves its support."""
    loops, support = tower["loops"], tower["support"]
    stack, reference = [], loops[0]["points"][0]
    if support is not None:
        # The strand leaves the supporting outline at its nearest point and
        # widens to the tower's own outline over the first layer.
        centre = np.asarray(loops[0]["polygon"].centroid.coords[0])
        if id(support) in tops:
            # Standing on another tower: start from a point of its last turn.
            reference = min(
                tops[id(support)], key=lambda point: np.linalg.norm(point[:2] - centre)
            )
        else:
            ring = LineString(support["points"][:, :2])
            fraction = ring.project(Point(centre), normalized=True)
            support["joins"].add(fraction % 1.0)
            reference = sample_loop(support["points"], fraction)
        stack.append(reference)
    for loop in loops:
        stack.append(align_loop(loop["points"], reference))
        reference = stack[-1][0]
    return stack


def _coil_points(lower, upper, pitch):
    """Points of the strand winding from ``lower`` up to ``upper``, and points per turn."""
    z0, z1 = (lower[2] if lower.ndim == 1 else lower[0, 2]), upper[0, 2]
    turns = max(1, int(round((z1 - z0) / pitch)))
    per_turn = max(8, int(np.ceil(LineString(upper[:, :2]).length / 0.0015)))
    points = []
    for sample in range(turns * per_turn + 1):
        along = sample / (turns * per_turn)
        around = (sample % per_turn) / per_turn
        high = sample_loop(upper, around)
        base = lower if lower.ndim == 1 else sample_loop(lower, around)
        point = (1 - along) * base + along * high
        point[2] = z0 + along * (z1 - z0)
        points.append(point)
    return points, per_turn


def _wind_coils(wire, towers, diameter, pitch):
    """Draw each coil tower as one climbing strand; return the node pairs of touching turns."""
    fused, tops = [], {}
    for number, tower in enumerate(sorted(towers, key=lambda tower: tower["level"])):
        loops = tower["loops"]
        stack = _coil_stack(tower, tops)
        first, beneath = tower["level"] + (tower["support"] is None), []
        for step, (lower, upper) in enumerate(zip(stack[:-1], stack[1:], strict=True)):
            points, per_turn = _coil_points(lower, upper, pitch)
            # Each point rests on the turn below it: the same place one turn
            # back, or the nearest point of the section beneath.
            for sample, point in enumerate(points):
                if sample >= per_turn:
                    under = points[sample - per_turn]
                elif beneath:
                    under = min(beneath, key=lambda other: np.linalg.norm(other - point))
                else:
                    continue
                if np.linalg.norm(under - point) <= 2 * diameter:
                    fused.append([wire.node(under), wire.node(point)])
            beneath = points[-per_turn - 1 :]
            tops[id(loops[-1])] = beneath
            for sample, (start, end) in enumerate(zip(points[:-1], points[1:], strict=True)):
                wire.add(
                    start,
                    end,
                    f"coil-{number}-{first + step}",
                    [sample, sample + 1],
                    1000 * (first + step) + 900 + number,
                )
    return fused


def _draw_base_lines(wire, lines, ground):
    """Draw the base grid, before the outlines it joins while the bed is still clear."""
    for number, stops in enumerate(lines):
        for step, (start, end) in enumerate(zip(stops[:-1], stops[1:], strict=True)):
            wire.add(
                start, end, f"base-{number}", [step, step + 1], 1000 * ground - len(lines) + number
            )


def _draw_layers(wire, layers, bridges, turns):
    """Draw each layer's zigzags and contours, the contours split at every join."""
    for index, loops in enumerate(layers):
        # Rising/descending strokes stand on the already completed lower contour.
        for layer, li, ui, a, b, c, step in (row for row in bridges if row[0] == index):
            down, up = layers[layer - 1][li]["points"], layers[layer][ui]["points"]
            name = f"zigzag-{layer}-{li}-{ui}"
            order = turns[(index, "zigzag", li, ui)]
            top = sample_loop(up, b)
            wire.add(sample_loop(down, a), top, name, [2 * step, 2 * step + 1], order)
            if c is not None:
                wire.add(top, sample_loop(down, c), name, [2 * step + 1, 2 * step + 2], order)
        for li, loop in enumerate(loops):
            points = loop["points"]
            distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            fractions = sorted(loop["joins"] | set(distances[:-1] / distances[-1]) | {1.0})
            for a, b in zip(fractions[:-1], fractions[1:], strict=True):
                wire.add(
                    sample_loop(points, a),
                    sample_loop(points, b),
                    f"contour-{index}-{li}",
                    [a, b],
                    turns[(index, "contour", li)],
                )


def _mesh_framework_at_height(
    mesh,
    *,
    layer_height=0.01,
    zigzag_spacing=0.025,
    diameter=0.001,
    slice_simplification=JUNCTION_M,
    minimum_contour=0.0,
    maximum_branch_descent_deg,
    base_line_spacing=None,
    coil_maximum_width=None,
    coil_pitch=0.001,
    minimum_connectors=None,
):
    """The stroke graph of contours, zigzags, base grid and coils for one layer height."""
    _validate(mesh, layer_height, zigzag_spacing, diameter, slice_simplification, minimum_contour)
    low = mesh.bounds[0, 2]
    heights = _layer_heights(
        mesh,
        layer_height,
        maximum_branch_descent_deg,
        minimum_connectors,
        minimum_contour,
        coil_maximum_width,
    )
    layers = _slice_layers(mesh, heights, slice_simplification, minimum_contour)
    layers = _prune_unsupported(_add_base_ring(layers, low))
    layers, towers = _find_coil_towers(layers, low, coil_maximum_width, diameter, coil_pitch)
    bridges = _plan_zigzags(layers, zigzag_spacing, maximum_branch_descent_deg, minimum_connectors)
    base_lines, ground = _base_grid(layers, low, base_line_spacing)
    turns = _build_order(layers, bridges)
    wire = _Wire(low, diameter)
    fused = _wind_coils(wire, towers, diameter, coil_pitch)
    _draw_base_lines(wire, base_lines, ground)
    _draw_layers(wire, layers, bridges, turns)
    return artifact(
        "stroke_graph",
        frame="build",
        units="m",
        method="wireprint_contours_zigzags",
        nodes=list(wire.nodes.values()),
        edges=wire.edges,
        fused_turns=[pair for pair in fused if pair[0] != pair[1]],
        generation={
            "layer_height_m": layer_height,
            "zigzag_spacing_m": zigzag_spacing,
            "minimum_connectors": minimum_connectors,
            "layer_heights_m": np.round(np.diff(heights), 6).tolist(),
            "strand_diameter_m": diameter,
            "slice_simplification_m": slice_simplification,
            "minimum_contour_m": minimum_contour,
            "junction_merge_m": JUNCTION_M,
            "base_line_spacing_m": base_line_spacing,
            "coil_maximum_width_m": coil_maximum_width,
            "coil_pitch_m": coil_pitch,
        },
        physical_status="unverified",
        simulation_only=True,
        limitations=[
            "Pen geometry, extrusion, cooling, stiffness and robot access require validation"
        ],
    )


def _descending_angles(graph):
    """Measure actual generated chords after the generator's junction merging."""
    points = {node["id"]: np.asarray(node["position"]) for node in graph["nodes"]}
    angles = []
    for edge in graph["edges"]:
        if not edge.get("parent_curve", "").startswith("zigzag-"):
            continue
        delta = points[edge["end"]] - points[edge["start"]]
        if delta[2] < -1e-12:
            angles.append(float(np.degrees(np.arctan2(-delta[2], np.linalg.norm(delta[:2])))))
    return angles


def mesh_framework(
    mesh,
    *,
    layer_height=0.01,
    zigzag_spacing=0.025,
    diameter=0.001,
    maximum_descent_angle_deg,
    slice_simplification=JUNCTION_M,
    minimum_contour=0.0,
    base_line_spacing=None,
    coil_maximum_width=None,
    coil_pitch=0.001,
    minimum_connectors=None,
):
    """Generate unchanged-scale contours with nozzle-aware descending chords.

    The angle is measured down from horizontal. The connector spacing remains
    the requested spacing; excessive actual chord slopes reduce slice height.
    """
    if not np.isfinite(maximum_descent_angle_deg) or not 0 < maximum_descent_angle_deg < 90:
        raise ValueError("Maximum descent angle must be finite and between 0 and 90 degrees")
    if not np.isfinite(layer_height) or layer_height <= 0:
        raise ValueError("Layer height must be finite and positive")
    low, high = mesh.bounds[:, 2]
    candidate_height = float(layer_height)
    for iteration in range(13):
        intervals = int(np.ceil((high - low) / candidate_height))
        if intervals < 1 or (high - low) / intervals <= JUNCTION_M:
            raise ValueError(
                "Cannot satisfy maximum descent angle at the 20 micron junction resolution"
            )
        if intervals > 4096:
            raise ValueError(
                "Cannot satisfy maximum descent angle within 4096 slice intervals; "
                "the input contours have insufficient horizontal chord clearance"
            )
        graph = _mesh_framework_at_height(
            mesh,
            layer_height=candidate_height,
            zigzag_spacing=zigzag_spacing,
            diameter=diameter,
            slice_simplification=slice_simplification,
            minimum_contour=minimum_contour,
            maximum_branch_descent_deg=maximum_descent_angle_deg,
            base_line_spacing=base_line_spacing,
            coil_maximum_width=coil_maximum_width,
            coil_pitch=coil_pitch,
            minimum_connectors=minimum_connectors,
        )
        layer_z = sorted({node["position"][2] for node in graph["nodes"]})
        effective_height = max(np.diff(layer_z), default=0.0)
        angles = _descending_angles(graph)
        maximum = max(angles, default=0.0)
        graph["generation"].update(
            layer_height_m=float(layer_height),
            requested_layer_height_m=float(layer_height),
            effective_layer_height_m=float(effective_height),
            maximum_descent_angle_deg=maximum_descent_angle_deg,
            actual_maximum_descent_angle_deg=maximum,
            descending_member_count=len(angles),
        )
        if maximum <= maximum_descent_angle_deg + 1e-8:
            return graph
        if iteration == 12:
            break
        ratio = np.tan(np.radians(maximum_descent_angle_deg)) / np.tan(np.radians(maximum))
        # Re-slice the source: a perimeter-derived formula alone misses short
        # corner chords and differences between adjacent curved contours.
        candidate_height = effective_height * min(0.9, max(0.1, float(ratio) * 0.95))
    raise ValueError(
        f"Cannot satisfy maximum descent angle {maximum_descent_angle_deg} degrees "
        f"after 12 slice refinements (actual maximum {maximum:.6g} degrees)"
    )


def generate(run, **settings):
    run = Path(run)
    # Decimation may move extreme vertices across the bed or distort openings.
    # Slice the normalized source; keep the simplified mesh for robot previews.
    source_file = read_json(run / "asset_manifest.json")["artifacts"]["reference"]
    mesh = trimesh.load(run / source_file, force="mesh")
    # Slice geometry only: texture seams duplicate points along the surface and
    # would otherwise read as open edges.
    mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.faces), process=True)
    mesh.merge_vertices(merge_tex=True, merge_norm=True)
    config_path = run / "fabrication_config.json"
    config = read_json(config_path) if config_path.exists() else {}
    configured = config.get("wireprint", {})
    solidified = None
    if configured.get("minimum_feature_m"):
        # Opt-in simplification for shapes whose thin parts cannot be wire-printed.
        from .solidify import solidify

        mesh, solidified = solidify(mesh, configured["minimum_feature_m"])
        mesh.export(run / "solidified.glb")
    known = {
        "layer_height",
        "zigzag_spacing",
        "diameter",
        "maximum_descent_angle_deg",
        "slice_simplification",
        "minimum_contour",
        "base_line_spacing",
        "coil_maximum_width",
        "coil_pitch",
        "minimum_connectors",
    }
    parameters = {
        "maximum_descent_angle_deg": 27.0,
        "diameter": config.get("graph", {}).get("strand_diameter_m", 0.001),
    }
    if solidified is not None:
        # The rebuilt surface follows a grid: its contours carry grid-sized
        # steps and its tips are rounded. Neither is detail of the source.
        parameters["slice_simplification"] = solidified["pitch_m"] / 4
        parameters["minimum_contour"] = 0.75 * solidified["minimum_feature_m"]
    parameters.update({key: value for key, value in configured.items() if key in known})
    parameters.update(settings)
    graph = mesh_framework(mesh, **parameters)
    if solidified is not None:
        graph["generation"]["solidify"] = solidified
    inputs = [source_file, "asset_manifest.json"]
    if config_path.exists():
        inputs.append("fabrication_config.json")
    graph["inputs"] = {name: sha256(run / name) for name in inputs}
    # Explicit births remain visible to preflight; no silent bridges across empty space.
    write_json(run / "wireprint_graph.json", graph)
    return graph


def connect_framework_islands(graph, maximum_connector_m=0.035):
    """Record short permanent connectors for disconnected generated contours."""
    nodes = {node["id"]: node for node in graph["nodes"]}
    edges = graph["edges"]
    diameter = graph["generation"]["strand_diameter_m"]
    active = {edge[end] for edge in edges for end in ("start", "end")}
    if active and not any(nodes[key]["ground"] for key in active):
        lowest = min(nodes[key]["position"][2] for key in active)
        if lowest < -ON_BED_M:
            raise ValueError("Framework extends below the normalized build surface")
        low_nodes = sorted(key for key in active if nodes[key]["position"][2] <= lowest + 1e-7)
        for key in base_anchors(nodes, low_nodes, ON_BED_M):
            foot = f"bed-foot-{key}"
            nodes[foot] = {
                "id": foot,
                "position": [*nodes[key]["position"][:2], 0.0],
                "ground": True,
            }
            edge = straight_edge(foot, key, nodes, f"bed-support-{key}", "support", diameter)
            edge.update(
                parent_curve=edge["id"],
                source_interval=[0, 1],
                stage="framework",
                origin="explicit_bed_support",
                order_hint=-1,
            )
            edges.append(edge)
        graph["nodes"] = list(nodes.values())
    count = len(edges)
    orders = {}
    for edge in edges:
        for end in ("start", "end"):
            orders.setdefault(edge[end], []).append(edge["order_hint"])
    connect_components(
        nodes,
        edges,
        {
            "maximum_connector_m": maximum_connector_m,
            "strand_diameter_m": diameter,
        },
    )
    for edge in edges[count:]:
        edge.update(
            parent_curve=edge["id"],
            source_interval=[0, 1],
            stage="framework",
            origin="explicit_component_connector",
            order_hint=max(min(orders[edge[end]]) for end in ("start", "end")) - 0.5,
        )
    return graph


def reinforce_base(graph):
    """Add bed posts along the first usable contour after a failed screen."""
    nodes = {node["id"]: node for node in graph["nodes"]}
    active = {edge[end] for edge in graph["edges"] for end in ("start", "end")}
    diameter = graph["generation"]["strand_diameter_m"]
    raised = [key for key in active if nodes[key]["position"][2] > diameter]
    if not raised:
        return graph
    height = min(nodes[key]["position"][2] for key in raised)
    first_layer = sorted(key for key in raised if nodes[key]["position"][2] <= height + 1e-7)
    # Posts already stand under some of these nodes. Add more where the contour
    # is furthest from any post, until none of it is a zigzag spacing away.
    posted = {
        edge["end"] for edge in graph["edges"] if edge.get("origin") == "explicit_bed_support"
    }
    spacing = graph["generation"]["zigzag_spacing_m"]
    anchors = []
    while True:
        stood = [nodes[key]["position"][:2] for key in (*posted, *anchors)]
        free = [key for key in first_layer if key not in posted and key not in anchors]
        if not free:
            break
        gap = {
            key: min(
                (np.linalg.norm(np.subtract(nodes[key]["position"][:2], p)) for p in stood),
                default=np.inf,
            )
            for key in free
        }
        furthest = max(free, key=gap.get)
        if gap[furthest] < spacing:
            break
        anchors.append(furthest)
    for key in anchors:
        foot = f"reinforcement-foot-{key}"
        nodes[foot] = {"id": foot, "position": [*nodes[key]["position"][:2], 0.0], "ground": True}
        edge = straight_edge(foot, key, nodes, f"bed-reinforcement-{key}", "support", diameter)
        edge.update(
            parent_curve=edge["id"],
            source_interval=[0, 1],
            stage="framework",
            origin="explicit_bed_support",
            order_hint=-1,
        )
        graph["edges"].append(edge)
    graph["nodes"] = list(nodes.values())
    return graph
