# Timing preview and pen-event contract

Run the timing stage only after a complete geometric sequence:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.pipeline \
  --config configs/palace_15cm_selected.yaml --stage timing
```

The stage uses `configs/timing.yaml` and the run's saved fabrication settings.
The output is `timed_schedule.json`, viewable under **Pen timing** in the local
workbench. It is a simulation artifact, not a hardware program. The original
geometric sequence remains available for comparison.

## Motion law

Each operation becomes a natural cubic joint spline through the existing joint
waypoints. The parameter uses tip distance plus a joint-distance term to handle
orientation-only motion. Numerical waypoints do not become extrusion stops.
Rest-to-rest motion is required at operation boundaries, where intentional joins
and pen-off travel occur.

[TOPPRA](https://hungpham2511.github.io/toppra/quickstart.html) computes a path clock
under URDF velocity limits, configured joint acceleration limits and a Cartesian
tip-speed constraint. The Cartesian constraint bounds the geometric path tangent
over each grid interval and constrains both endpoint clock speeds. Composing the
cubic joint path with the quadratic path clock gives a polynomial on each clock
interval. Polynomial extrema bound joint velocity and acceleration between
samples; a uniform time correction enforces those bounds if needed. This does
not establish jerk compliance, torque limits or real controller tracking.

The artifact stores joint names, limits, spline coefficients, path-clock knots,
timestamped positions/derivatives, and display samples. Timestamps within motion
operations are relative to their `start_s`; pen-event timestamps share the full
job clock. Web playback evaluates the spline and clock directly rather than
interpolating the display samples.

**Smoothing changes the path between waypoints.** Previous collision and tracking
certificates for joint-linear interpolation do not transfer. The timing artifact
therefore records `REQUIRES_SMOOTH_PATH_AUDIT` until a fresh audit is implemented
and passes. Position limits are checked over the cubic, not just at waypoints.

## Pen events and missing measurements

For each drawing stroke, the scheduler inserts a feed-off join, a start-response
interval, drawing, and a feed-off cooling interval. `START_FEED` precedes drawing
by the configured start lag. `STOP_FEED` precedes the drawing endpoint by the
configured stop lag. Expected plastic flow and commanded feed state are distinct;
this delay model needs actual pen measurements.

If a stroke is shorter than the assumed stop lag, the preview records a timing
conflict and clamps its stop command to the drawing start. This is explicitly
not an accepted feed schedule; the short action must be merged, redesigned or
covered by a calibrated response model before execution.

Unknown startup, cooling and shutdown durations remain `null` in the profile.
Their timeline placeholders contribute zero to the **modeled subtotal**, and the
missing durations are listed alongside that subtotal. Zero contribution does not
mean zero physical time. Neither a subtotal below 30 minutes nor a completed
retiming calculation establishes the full fabrication-time target.

Temperature, feed setting and calibration identity are also explicit profile
fields. Current acceleration limits, velocity fraction and travel speed are
development assumptions. No temperature or feed command is sent to hardware.
The output always retains `executable: false` while geometry, flow, cooling and
controller behavior remain unvalidated.

## Two-arm wireframe clock

The fixed-catalogue cell planner uses `configs/timing_continuous_bounded.yaml`.
Its drawing path is a midpoint-refined, shape-preserving cubic (PCHIP), and its
TOPPRA acceleration constraints bound each complete cubic interval using
Bernstein control values. It is C1 at the path knots; no claim of bounded jerk
or controller compatibility follows from that. Travel retains a rest-to-rest
quintic clock. Exact normalized joint polynomials are saved in each drawing
edge, so timing playback and shared collision replay use the same trajectory.

This cell path uses explicit endpoint waits: feed starts at `PEN_START_WAIT`,
the tip dwells for `JOIN`, draws the stroke, switches feed off at
`PEN_STOP_WAIT`, and then withdraws. The stop-lag wait occurs at the endpoint;
it is not the anticipatory stop model described above for the older single-arm
preview. This behavior needs pen calibration, including residual extrusion at
the joint. Both models keep their distinct event conventions in their artifacts.

`cell_allocation.json` is a duration estimate with estimated incoming travel.
`allocation_motions.json` contains the prepared full paths, their progress and
any selected-motion failures. `offline_dual_arm_plan.json` replaces estimated
transfers with constructed paths and supplies the shared clock and per-arm
feed events. Only its matching `offline_dual_arm_validation.json` can establish
that the saved complete schedule passed modeled geometric replay. Unknown
startup/shutdown overhead and physical process behavior still prevent treating
that replay as an accepted fabrication time.
