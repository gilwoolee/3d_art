# Measured pen and mount geometry

The drawing audit and sequence planner accept a measured tool profile under
`tool_geometry` in the collision YAML. `null` means no pen/mount geometry has
been supplied. It never selects dimensions from a product name. The current
Palace run uses `null` and the provisional 16 cm flange-to-TCP transform.

Coordinates are in meters in the robot's terminal flange frame. The profile's
`flange_to_tcp` must match `drawing_motion.json` exactly. After changing that
transform, regenerate placement and drawing paths before collision auditing.

| Field | Meaning |
| --- | --- |
| `schema_version` | `1` |
| `units` | `m` |
| `flange_to_tcp` | Rigid 4×4 transform from flange to physical pen tip |
| `tip_radius_m` | Positive radius of the tip's spherical contact approximation |
| `calibrated` | Optional declaration only; it is not proof of physical validation |
| `bodies` | Nonempty list of non-contact pen, mount, and relevant cable/filament bodies |

Each body requires a unique `name`, a rigid 4×4 `origin` in the flange frame,
and a `shape` mapping. Shape options:

| `shape.type` | Required dimensions |
| --- | --- |
| `box` | `size_m: [x, y, z]` |
| `sphere` | `radius_m` |
| `cylinder` | `radius_m`, `length_m`; axis is local Z |
| `mesh` | `path`, optional `scale: [x, y, z]` (default `[1, 1, 1]`) |

Mesh paths are relative to the collision YAML, or absolute. Each mesh is read
once and replaced by its filled convex hull. Split a concave assembly into
separate convex bodies when a single hull would close useful openings. The
webpage renders these same hull vertices, dimensions, and body transforms.

Bodies fixed to the same flange are treated as one rigid assembly. Other robot
link contacts are checked, including the adjacent forearm. A body's optional
`allowed_touch_links` list can explicitly exempt known mounting contacts; it
defaults to empty. Do not use it to exempt moving links that the pen can strike.
Table, platform, and deposited-material contacts are never exempted for these
non-contact tool bodies. The local joining allowance applies only to the tip.

The drawing audit checks the supplied non-contact geometry against the arm and
static scene. The sequence planner additionally checks it against growing
material during approach, drawing, withdrawal and travel. Travel/lifts use the
profile tip radius; without a profile they use `virtual_tip_radius_m` from the
transit settings. Drawing contact and the real deposition process remain separate
validation work. The spherical tip is an approximation, not a nozzle CAD model.

```bash
# Use a collision YAML containing the measured tool_geometry mapping:
.venv/bin/python -m pen_art.planning.collision_report \
  --config configs/measured_collision.yaml --run runs/palace_15cm
.venv/bin/python -m pen_art.planning.transit \
  --collision-config configs/measured_collision.yaml --run runs/palace_15cm
```

Those commands require the file to be supplied; no measured profile is bundled.
The audit embeds the rendered geometry and a fingerprint of the profile plus
mesh contents. A changed mesh, even at the same path, invalidates sequence reuse
and requires a new audit. Legacy arm-only audits remain usable only with a null
profile. Introducing a measured tool can invalidate existing drawing paths; it
does not automatically prove that the same arm configurations remain feasible.
