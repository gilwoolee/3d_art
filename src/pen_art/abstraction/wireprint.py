"""Horizontal contours and ordered connecting zigzags in the physical build frame.

Each zigzag is a vertical riser from the lower contour followed by a slanted
stroke back down to it, as in WirePrint.

This is a WirePrint-style generator, not a calibrated printer implementation.
The returned topology and order still require structural and robot validation.
"""

import argparse
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize, unary_union

from ..graph import base_anchors, connect_components, straight_edge
from ..io import artifact, read_json, sha256, write_json


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


def _mesh_framework_at_height(
    mesh,
    *,
    layer_height=0.01,
    zigzag_spacing=0.025,
    diameter=0.001,
    minimum_anchor_spacing=0.0,
    omit_closing_diagonal=False,
    slice_simplification=0.00002,
    minimum_contour=0.0,
    maximum_branch_descent_deg=None,
    base_line_spacing=None,
    coil_maximum_width=None,
    coil_pitch=0.001,
):
    if min(layer_height, zigzag_spacing, diameter, slice_simplification) <= 0:
        raise ValueError("Positive layer, zigzag, diameter and simplification are required")
    if not np.isfinite(minimum_contour) or minimum_contour < 0:
        raise ValueError("Minimum contour size must be finite and nonnegative")
    if not np.isfinite(minimum_anchor_spacing) or minimum_anchor_spacing < 0:
        raise ValueError("Minimum anchor spacing must be finite and nonnegative")
    low, high = mesh.bounds[:, 2]
    if high <= low:
        raise ValueError("Mesh must have positive height")
    heights = np.linspace(low, high, int(np.ceil((high - low) / layer_height)) + 1)
    layers, repairs, omitted_small = [], [], 0
    for layer, height in enumerate(heights):
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
                if nudge:
                    repairs.append({"layer": layer, "method": "nudged_cut", "offset_m": nudge})
                break
        loops = []
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
                repairs.append(
                    {
                        "layer": layer,
                        "method": "node_crossings",
                        "simple_rings": len(unique),
                        "lost_unique_boundary_m": lost_length,
                    }
                )
            else:
                rings.append(np.asarray(simplified.coords))
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
                omitted_small += 1
                continue
            # Consistent counterclockwise winding and a repeatable geometric seam.
            signed = np.sum(xy[:-1, 0] * xy[1:, 1] - xy[1:, 0] * xy[:-1, 1])
            vertices = xy[:-1] if signed > 0 else xy[:-1][::-1]
            # Near-equal X coordinates must not randomly choose different
            # square corners on neighboring slices because of roundoff.
            quantized = np.rint(vertices / 0.00002)
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
        layers.append(loops)

    # A shape with a rounded underside has no contour on the bed: its lowest
    # one starts a layer up. Posts under that contour would be the only thing
    # holding the object, each stuck to the bed at a point. Draw the same
    # outline flat on the bed instead, so the usual band of risers and
    # diagonals carries the object on a base that grips along its whole length.
    base_ring = None
    first = next((index for index, loops in enumerate(layers) if loops), None)
    if maximum_branch_descent_deg is not None and first:
        base = []
        for loop in layers[first]:
            points = loop["points"].copy()
            points[:, 2] = low
            base.append({**loop, "points": points, "joins": {0.0}})
        layers[first - 1] = base
        base_ring = {
            "contours": len(base),
            "height_m": float(layers[first][0]["points"][0, 2] - low),
        }

    # An inner contour that begins above the bed is the wall of a cavity, such
    # as the inside of a vase: a ring with nothing beneath it. Print the outer
    # shell only. Removing it bottom-up also removes the contours it would have
    # carried. Inner contours that start on the bed, like a torus's hole, stay.
    # An outer contour that begins in mid-air, above the object's lowest
    # contour, has nothing to be drawn onto either: an overhanging tip that
    # joins the shape a layer or two up. It is left out the same way, and the
    # contour it joins covers that part of the shape.
    omitted_cavities = omitted_floating = 0
    started = bool(layers and layers[0])
    for index in range(1, len(layers)):
        kept = []
        for up in layers[index]:
            carried = any(
                up["depth"] == down["depth"] and up["polygon"].intersects(down["polygon"])
                for down in layers[index - 1]
            )
            if up["depth"] and not carried:
                omitted_cavities += 1
                continue
            if started and not carried and maximum_branch_descent_deg is not None:
                omitted_floating += 1
                continue
            kept.append(up)
        layers[index] = kept
        started = started or bool(kept)

    # A coil tower: a part too narrow for loops and zigzags, such as a raised
    # arm, is wound as one strand that climbs by its own thickness each turn,
    # so the turns touch and form a thin tube. It applies to a run of narrow
    # outlines that ends without widening again, standing on one wider outline
    # or on the bed. Those outlines leave the layers and are wound instead.
    towers = []
    if coil_maximum_width:

        def narrow(loop):
            x0, y0, x1, y1 = loop["polygon"].bounds
            return not loop["depth"] and max(x1 - x0, y1 - y0) <= coil_maximum_width

        def touching(loop, others):
            return [
                other
                for other in others
                if other["depth"] == loop["depth"] and other["polygon"].intersects(loop["polygon"])
            ]

        wound = set()
        for index in range(len(layers) - 1, -1, -1):
            for loop in layers[index]:
                ends = index + 1 == len(layers) or not touching(loop, layers[index + 1])
                if id(loop) in wound or not narrow(loop) or not ends:
                    continue
                chain, level = [loop], index
                while level > 0:
                    below = touching(chain[0], layers[level - 1])
                    if (
                        len(below) != 1
                        or not narrow(below[0])
                        or len(touching(below[0], layers[level])) != 1
                    ):
                        break
                    chain.insert(0, below[0])
                    level -= 1
                support = touching(chain[0], layers[level - 1]) if level else []
                grounded = abs(chain[0]["points"][0, 2] - low) < 1e-7
                if len(support) != 1 and not (grounded and len(chain) > 1):
                    continue
                # The first layer leans from the supporting outline to the
                # tower. More than two strand widths per turn and its turns
                # would hang in air, so such a part is not wound.
                if support and not grounded:
                    below = LineString(support[0]["points"][:, :2])
                    reach = max(below.distance(Point(p)) for p in chain[0]["points"][:, :2])
                    rise = chain[0]["points"][0, 2] - support[0]["points"][0, 2]
                    if reach > 2 * diameter / coil_pitch * rise:
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

    relations, bridges, spacing_adjustments, omitted_steep, nearest_pairs = [], [], [], 0, 0
    stretched = 0
    for index in range(1, len(layers)):
        lower, upper = layers[index - 1], layers[index]
        pairs = []
        for ui, up in enumerate(upper):
            parents = [
                li
                for li, down in enumerate(lower)
                if up["depth"] == down["depth"] and up["polygon"].intersects(down["polygon"])
            ]
            if not parents:
                relations.append({"layer": index, "upper": ui, "event": "unsupported_birth"})
            for li in parents:
                pairs.append((li, ui))
            if parents:
                parent = max(
                    parents, key=lambda li: up["polygon"].intersection(lower[li]["polygon"]).area
                )
                up["points"] = align_loop(up["points"], lower[parent]["points"][0])
        for li, ui in pairs:
            down, up = lower[li], upper[ui]
            perimeter = max(LineString(loop["points"]).length for loop in (down, up))
            # Where contours split or merge, equal fractions pair unrelated
            # points. Stand each upper contour on the nearest part of the
            # contour beneath it instead.
            co_parents = [parent for parent, u in pairs if u == ui]
            branching = (
                maximum_branch_descent_deg is not None
                and not minimum_anchor_spacing
                and (len(co_parents) > 1 or sum(parent == li for parent, _ in pairs) > 1)
            )
            beneath = LineString(down["points"][:, :2])
            overhead = LineString(up["points"][:, :2])

            # Only one zigzag can begin at the lower contour's chain point; a
            # sibling starting there would stand on the first one's riser.
            takes_chain = "chain" in down and down.setdefault("chain_user", ui) == ui

            def anchors(
                u, v, nearest, count, up=up, down=down, beneath=beneath, overhead=overhead,
                chained=takes_chain,
            ):
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
                        overhead.project(
                            Point(sample_loop(down["points"], origin)[:2]), normalized=True
                        )
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

            def descent(b, c, up=up, down=down):
                fall = sample_loop(up["points"], b) - sample_loop(down["points"], c)
                return np.degrees(np.arctan2(fall[2], np.linalg.norm(fall[:2])))

            def steepest(count, nearest, anchors=anchors, descent=descent):
                return max(
                    descent(*anchors(i / count, (i + 1) / count, nearest, count)[1:])
                    for i in range(count - 1)
                )

            count = max(3, int(np.ceil(perimeter / zigzag_spacing)))
            above = max(3, int(np.ceil(LineString(up["points"]).length / zigzag_spacing)))
            # Two contours of different outline can also pair badly by equal
            # fractions. Use the nearest part beneath when that alone brings
            # the diagonals within the limit; like outlines keep equal fractions.
            # A degree of margin covers the later 20 micron junction merge.
            nearest = branching or (
                maximum_branch_descent_deg is not None
                and not minimum_anchor_spacing
                and steepest(count, False) > maximum_branch_descent_deg
                and steepest(above, True) <= maximum_branch_descent_deg - 1
            )
            if nearest:
                count = above
                nearest_pairs += 1
            original_count = count
            if minimum_anchor_spacing:
                # Equal fractions on a large upper contour can crowd anchors
                # onto a much smaller lower contour. Reduce the connector count
                # until distinct anchors on both contours have room. This is a
                # layout screen, not a certificate for the entire pen/robot.
                while count > 1:
                    samples = [
                        np.asarray(
                            [
                                sample_loop(loop["points"], (i + offset) / count)
                                for i in range(count)
                            ]
                        )
                        for loop, offset in ((down, 0.0), (up, 0.0))
                    ]
                    if all(
                        min(
                            np.linalg.norm(points[i] - points[j])
                            for i in range(count)
                            for j in range(i)
                        )
                        >= minimum_anchor_spacing
                        for points in samples
                    ):
                        break
                    count -= 1
                spacing_adjustments.append(
                    {
                        "layer": index,
                        "lower": li,
                        "upper": ui,
                        "original_connector_pairs": original_count,
                        "connector_pairs": count,
                        "closing_diagonal_omitted": True,
                    }
                )
            # Where equal spacing would make a diagonal too steep, its foot and
            # the riser that follows stand further round the loop, far enough
            # for the diagonal to come down within the limit.
            stretch = (
                maximum_branch_descent_deg is not None
                and not branching
                and not minimum_anchor_spacing
            )
            step, i, shift = 0, 0, 0.0
            while i / count + shift < 1 - 1e-9:
                u, v = i / count + shift, (i + 1) / count + shift
                ended = False
                a, b, c = anchors(u, v, nearest, count)
                if stretch and v < 1 - 1e-9 and descent(b, c) > maximum_branch_descent_deg - 1:
                    reach = v
                    while (
                        reach < min(u + 0.75, 1 - 0.5 / count)
                        and descent(b, c) > maximum_branch_descent_deg - 1
                    ):
                        reach += 0.125 / count
                        a, b, c = anchors(u, reach, nearest, count)
                    if descent(b, c) <= maximum_branch_descent_deg - 1:
                        shift += reach - v
                        v = reach
                        stretched += 1
                    else:
                        # No foot before the loop closes is far enough: this
                        # riser ends the zigzag, as the closing one would.
                        ended = reach >= 1 - 0.5 / count
                        a, b, c = anchors(u, v, nearest, count)
                # A riser that would stand within half a spacing of the first
                # one is left out; this riser is then the zigzag's last.
                last = (
                    ended
                    or v >= 1 - 1e-9
                    or bool(stretch and shift and v > 1 - 0.5 / count)
                )
                if last:
                    shift = 1.0
                i += 1
                if branching:
                    top = Point(sample_loop(up["points"], b)[:2])
                    if li != min(co_parents, key=lambda k: lower[k]["polygon"].distance(top)):
                        continue
                # WirePrint omits the closing diagonal to leave print-head
                # access. With one anchor this also avoids retracing the riser.
                if last and (minimum_anchor_spacing or omit_closing_diagonal or v < 1 - 1e-9):
                    c = None
                # A diagonal that still comes out too steep is left out rather
                # than thinning every layer; its riser still stands.
                if (
                    c is not None
                    and (branching or stretch)
                    and descent(b, c) > maximum_branch_descent_deg - 1
                ):
                    omitted_steep += 1
                    c = None
                down["joins"].add(a % 1.0)
                if c is not None:
                    down["joins"].add(c % 1.0)
                up["joins"].add(b % 1.0)
                # The last riser top, where the next zigzag up will begin.
                up["chain"] = b % 1.0
                bridges.append((index, li, ui, a, b, c, step))
                step += 1
            relations.append(
                {
                    "layer": index,
                    "lower": li,
                    "upper": ui,
                    "event": "merge"
                    if sum(u == ui for _, u in pairs) > 1
                    else "split"
                    if sum(parent == li for parent, _ in pairs) > 1
                    else "continue",
                }
            )
        for li in range(len(lower)):
            if not any(parent == li for parent, _ in pairs):
                relations.append({"layer": index, "lower": li, "event": "contour_end"})

    # A grid of straight lines across each outline that lies on the bed. It
    # ties the base together both ways and gives it more length gripping the
    # bed than the outline alone. Lines meet at shared points, as a contour
    # meets the riser tops it is drawn across.
    base_lines = []
    ground = next((index for index, loops in enumerate(layers) if loops), None)
    if (
        base_line_spacing
        and ground is not None
        and abs(layers[ground][0]["points"][0, 2] - low) < 1e-7
    ):
        loops = layers[ground]
        rings = [LineString(loop["points"][:, :2]) for loop in loops]
        inside = unary_union([loop["polygon"] for loop in loops if loop["depth"] % 2 == 0])
        holes = [loop["polygon"] for loop in loops if loop["depth"] % 2]
        if holes:
            inside = inside.difference(unary_union(holes))
        bounds = np.reshape(inside.bounds, (2, 2))
        pieces = {0: [], 1: []}
        for axis in (0, 1):
            across = 1 - axis
            count = int((bounds[1, across] - bounds[0, across]) // base_line_spacing)
            for row in range(count):
                level = bounds[:, across].mean() + (row - (count - 1) / 2) * base_line_spacing
                ends = np.zeros((2, 2))
                ends[:, axis] = bounds[0, axis] - 1, bounds[1, axis] + 1
                ends[:, across] = level
                crossing = inside.intersection(LineString(ends))
                for piece in getattr(crossing, "geoms", [crossing]):
                    if piece.geom_type == "LineString" and piece.length >= base_line_spacing:
                        pieces[axis].append(np.asarray([piece.coords[0], piece.coords[-1]]))
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
                base_lines.append([np.r_[point, low] for point in stops])

    nodes, edges, positions, by_edge = {}, [], [], {}

    def node(point):
        # Plane intersections close to mesh vertices can leave micron-long
        # slivers. Merge them at the same declared geometric resolution used by
        # simplification, before defining source members or mechanical beams.
        if positions:
            distances = np.linalg.norm(np.asarray(positions) - point, axis=1)
            nearest = int(np.argmin(distances))
            if distances[nearest] <= 0.00002:
                return f"n{nearest:05d}"
        identity = f"n{len(nodes):05d}"
        positions.append(np.asarray(point))
        nodes[identity] = {
            "id": identity,
            "position": np.asarray(point).tolist(),
            "ground": bool(abs(point[2] - low) < 1e-7),
        }
        return identity

    def add(a, b, source, interval, order, role):
        first, second = node(a), node(b)
        if first == second:
            return
        key = tuple(sorted((first, second)))
        if key in by_edge:
            by_edge[key].setdefault("shared_sources", []).append(
                {
                    "parent_curve": source,
                    "source_interval": interval,
                }
            )
            return
        edge = straight_edge(first, second, nodes, f"member-{len(edges):05d}", role, diameter)
        edge.update(
            origin="wireprint",
            parent_curve=source,
            source_interval=interval,
            stage="framework",
            order_hint=order,
        )
        by_edge[key] = edge
        edges.append(edge)

    # Build order within a layer: each contour straight after the zigzags that
    # carry it, starting with a loop that stands on the contour finished last,
    # so that consecutive strokes meet end to start wherever the shape allows.
    # Loops that share a lower contour are the exception: a finished contour
    # would stand in the pen's way to its sibling's feet just beneath, so all
    # of their zigzags come first and their contours after.
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

    coil_turns, fused = 0, []
    for number, tower in enumerate(towers):
        loops, support, level = tower["loops"], tower["support"], tower["level"]
        stack, reference = [], loops[0]["points"][0]
        if support is not None:
            # The strand leaves the supporting outline at its nearest point and
            # widens to the tower's own outline over the first layer.
            centre = np.asarray(loops[0]["polygon"].centroid.coords[0])
            ring = LineString(support["points"][:, :2])
            fraction = ring.project(Point(centre), normalized=True)
            support["joins"].add(fraction % 1.0)
            reference = sample_loop(support["points"], fraction)
            stack.append(reference)
        for loop in loops:
            stack.append(align_loop(loop["points"], reference))
            reference = stack[-1][0]
        first, beneath = level + (support is None), []
        for step, (lower, upper) in enumerate(zip(stack[:-1], stack[1:], strict=True)):
            z0, z1 = (lower[2] if lower.ndim == 1 else lower[0, 2]), upper[0, 2]
            turns_here = max(1, int(round((z1 - z0) / coil_pitch)))
            per_turn = max(8, int(np.ceil(LineString(upper[:, :2]).length / 0.0015)))
            points = []
            for sample in range(turns_here * per_turn + 1):
                along = sample / (turns_here * per_turn)
                around = (sample % per_turn) / per_turn
                high = sample_loop(upper, around)
                base = lower if lower.ndim == 1 else sample_loop(lower, around)
                point = (1 - along) * base + along * high
                point[2] = z0 + along * (z1 - z0)
                points.append(point)
            coil_turns += turns_here
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
                    fused.append([node(under), node(point)])
            beneath = points[-per_turn - 1 :]
            for sample, (start, end) in enumerate(zip(points[:-1], points[1:], strict=True)):
                add(
                    start,
                    end,
                    f"coil-{number}-{first + step}",
                    [sample, sample + 1],
                    1000 * (first + step) + 900 + number,
                    "framework",
                )
    for number, stops in enumerate(base_lines):
        # Drawn before the outlines they join, while the bed is still clear.
        for step, (start, end) in enumerate(zip(stops[:-1], stops[1:], strict=True)):
            add(
                start,
                end,
                f"base-{number}",
                [step, step + 1],
                1000 * ground - len(base_lines) + number,
                "framework",
            )
    for index, loops in enumerate(layers):
        # Rising/descending strokes stand on the already completed lower contour.
        for layer, li, ui, a, b, c, step in (row for row in bridges if row[0] == index):
            down, up = layers[layer - 1][li]["points"], layers[layer][ui]["points"]
            name = f"zigzag-{layer}-{li}-{ui}"
            add(
                sample_loop(down, a),
                sample_loop(up, b),
                name,
                [2 * step, 2 * step + 1],
                turns[(index, "zigzag", li, ui)],
                "framework",
            )
            if c is not None:
                add(
                    sample_loop(up, b),
                    sample_loop(down, c),
                    name,
                    [2 * step + 1, 2 * step + 2],
                    turns[(index, "zigzag", li, ui)],
                    "framework",
                )
        for li, loop in enumerate(loops):
            points = loop["points"]
            distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            fractions = sorted(loop["joins"] | set(distances[:-1] / distances[-1]) | {1.0})
            for a, b in zip(fractions[:-1], fractions[1:], strict=True):
                add(
                    sample_loop(points, a),
                    sample_loop(points, b),
                    f"contour-{index}-{li}",
                    [a, b],
                    turns[(index, "contour", li)],
                    "framework",
                )
    return artifact(
        "stroke_graph",
        frame="build",
        units="m",
        method="wireprint_contours_zigzags",
        nodes=list(nodes.values()),
        edges=edges,
        contour_events=relations,
        contour_repairs=repairs,
        omitted_cavity_contours=omitted_cavities,
        omitted_floating_contours=omitted_floating,
        base_ring=base_ring,
        base_lines=len(base_lines),
        coil_towers={"count": len(towers), "turns": coil_turns},
        fused_turns=[pair for pair in fused if pair[0] != pair[1]],
        omitted_small_contours=omitted_small,
        omitted_steep_branch_diagonals=omitted_steep,
        nearest_point_zigzags=nearest_pairs,
        stretched_diagonals=stretched,
        anchor_spacing_adjustments=spacing_adjustments,
        generation={
            "layer_height_m": layer_height,
            "zigzag_spacing_m": zigzag_spacing,
            "strand_diameter_m": diameter,
            "slice_simplification_m": slice_simplification,
            "minimum_contour_m": minimum_contour,
            "junction_merge_m": 0.00002,
            "minimum_anchor_spacing_m": minimum_anchor_spacing,
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
    minimum_anchor_spacing=0.0,
    maximum_descent_angle_deg=None,
    slice_simplification=0.00002,
    minimum_contour=0.0,
    base_line_spacing=None,
    coil_maximum_width=None,
    coil_pitch=0.001,
):
    """Generate unchanged-scale contours with nozzle-aware descending chords.

    The angle is measured down from horizontal. The connector spacing remains
    the requested spacing; excessive actual chord slopes reduce slice height.
    ``None`` retains the explicit legacy geometry API for historical fixtures.
    """
    if maximum_descent_angle_deg is not None and (
        not np.isfinite(maximum_descent_angle_deg) or not 0 < maximum_descent_angle_deg < 90
    ):
        raise ValueError("Maximum descent angle must be finite and between 0 and 90 degrees")
    if not np.isfinite(layer_height) or layer_height <= 0:
        raise ValueError("Layer height must be finite and positive")
    low, high = mesh.bounds[:, 2]
    candidate_height = float(layer_height)
    refinements = []
    for iteration in range(13):
        intervals = int(np.ceil((high - low) / candidate_height))
        if maximum_descent_angle_deg is not None and (
            intervals < 1 or (high - low) / intervals <= 0.00002
        ):
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
            minimum_anchor_spacing=minimum_anchor_spacing,
            omit_closing_diagonal=maximum_descent_angle_deg is not None,
            slice_simplification=slice_simplification,
            minimum_contour=minimum_contour,
            maximum_branch_descent_deg=maximum_descent_angle_deg,
            base_line_spacing=base_line_spacing,
            coil_maximum_width=coil_maximum_width,
            coil_pitch=coil_pitch,
        )
        layer_z = sorted({node["position"][2] for node in graph["nodes"]})
        effective_height = max(np.diff(layer_z), default=0.0)
        angles = _descending_angles(graph)
        maximum = max(angles, default=0.0)
        omitted = (
            sum(
                event.get("event") in {"continue", "split", "merge"}
                for event in graph["contour_events"]
            )
            if maximum_descent_angle_deg is not None or minimum_anchor_spacing
            else 0
        )
        graph["generation"].update(
            layer_height_m=float(layer_height),
            requested_layer_height_m=float(layer_height),
            effective_layer_height_m=float(effective_height),
            maximum_descent_angle_deg=maximum_descent_angle_deg,
            actual_maximum_descent_angle_deg=maximum,
            descending_member_count=len(angles),
            closing_diagonal_omitted=bool(omitted),
            omitted_closing_diagonals=omitted,
            descent_angle_refinements=refinements,
        )
        if maximum_descent_angle_deg is None or maximum <= maximum_descent_angle_deg + 1e-8:
            return graph
        if iteration == 12:
            break
        ratio = np.tan(np.radians(maximum_descent_angle_deg)) / np.tan(np.radians(maximum))
        refinements.append(
            {
                "effective_layer_height_m": float(effective_height),
                "actual_maximum_descent_angle_deg": maximum,
            }
        )
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
        "minimum_anchor_spacing",
        "maximum_descent_angle_deg",
        "slice_simplification",
        "minimum_contour",
        "base_line_spacing",
        "coil_maximum_width",
        "coil_pitch",
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
    bed_supports = []
    if active and not any(nodes[key]["ground"] for key in active):
        lowest = min(nodes[key]["position"][2] for key in active)
        if lowest < -1e-7:
            raise ValueError("Framework extends below the normalized build surface")
        low_nodes = sorted(key for key in active if nodes[key]["position"][2] <= lowest + 1e-7)
        for key in base_anchors(nodes, low_nodes, 1e-7):
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
            bed_supports.append(edge["id"])
        graph["nodes"] = list(nodes.values())
    count = len(edges)
    orders = {}
    for edge in edges:
        for end in ("start", "end"):
            orders.setdefault(edge[end], []).append(edge["order_hint"])
    original_components = connect_components(
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
    graph["component_connections"] = {
        "original_components": original_components,
        "added_members": [edge["id"] for edge in edges[count:]],
        "maximum_connector_m": maximum_connector_m,
        "bed_supports": bed_supports,
    }
    return graph


def reinforce_base(graph, reason):
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
    added = []
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
        added.append(edge["id"])
    graph["nodes"] = list(nodes.values())
    graph["base_reinforcement"] = {
        "reason": reason,
        "added_members": added,
        "target_contour_height_m": height,
    }
    return graph


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    args = parser.parse_args()
    graph = generate(args.run)
    print(f"{len(graph['nodes'])} nodes, {len(graph['edges'])} sections")
