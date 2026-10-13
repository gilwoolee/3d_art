# Collision checking and timing: what the literature offers, and what to build

Recorded 2026-10-13. Web research plus this repository's profiles. Statements
marked *inference* are reasoning from the sources and our measurements, not
something a source states. No method below has been implemented or timed here.
Current numbers are from `runs/geomfirst_gf30_*_w24`; see
[fast planning and verification](2026_1013_fast_planning_and_verification.md).

## Where the time is

Palace, 366 actions, 62,194 path pieces, 24 workers:

| Work | Worker time | Per action | Wall share |
| --- | ---: | ---: | ---: |
| Pen against printed wire (planning check) | 399 s | 1.1 s | about 18 s |
| Transit validation | 172 s | 0.5 s | about 8 s |
| Retiming | 490 s | 1.3 s | about 21 s |
| Timing audit | 366 s | 1.0 s | about 16 s |

That is roughly 6 ms per path piece for timing and 6 ms per piece for the pen
check. The published methods below work in microseconds per piece. The gap is
almost entirely how the work is organised, not the mathematics.

## Collision: the pen against the wire

The agreed design for items 1-4 below is in
[pen collision design](2026_1013_pen_collision_design.md).

The arm is already cheap: clearances measured once certify 98.5% of edges. The
remaining cost is the pen, which by design is within a millimetre of wire.

### What does not fit

- **Sphere-model planners (VAMP, pRRTC).** VAMP compiles kinematics and
  sphere-against-primitive tests into SIMD code and plans in tens of
  microseconds, but it checks edges at sampled states, and a sphere model cannot
  represent a 0.8 mm nose at 1 mm clearance. The lesson to take is the layout:
  branch-free batched tests over arrays.
  [VAMP](https://arxiv.org/abs/2309.14545),
  [pRRTC](https://arxiv.org/abs/2503.06757)
- **cuRobo's continuous check.** It sweeps spheres between waypoints assuming
  linear motion and is a soft optimisation cost, not a guarantee.
  [cuRobo](https://arxiv.org/abs/2310.17274)
- **Voxel distance fields (Voxblox, FIESTA, nvblox).** A voxel field is only a
  certificate after subtracting about half a voxel diagonal. At 1 mm clearance
  that needs voxels near 0.1 mm, on the order of 10^9 cells for a 150 mm object
  (*inference*). [nvblox](https://arxiv.org/abs/2311.00626)
- **Clearance bubbles for the pen.** Bubbles, burs and safety certificates
  amortise one distance query over a ball whose size is clearance divided by
  lever arm. For the arm that ball spans many edges, which is what we use. For
  the pen it is about one edge (*inference*).
  [Safety certificates](https://www.ottelab.com/html_stuff/pdf_files/Bialkowski_ICRA_CRG13.pdf)
- **Learned configuration-space distance fields.** Approximate, not a
  certificate. [CDF](https://arxiv.org/abs/2406.01137)

### What fits

1. **A second-order motion bound.** Our check is the adaptive bisection of
   Schwarzer, Saha and Latombe: compare a distance with a first-order bound on
   how far a body can move (lever arm times joint motion), and halve the
   interval when the bound is too loose. For the pen the first-order bound is
   comparable to the clearance, which is very likely why intervals are halved
   repeatedly. Checking the *chord* of the motion exactly and adding only the
   deviation from the chord, at most (interval squared over 8) times the
   largest tip acceleration, makes the inflation about a micron instead of
   about a millimetre (*inference*). One test per edge then suffices almost
   always. The same remainder bound is already used here for tracking.
   [Adaptive bisection](https://ai.stanford.edu/~mitul/research.html),
   [swept volume of a translating convex body](https://arxiv.org/abs/2302.09704)
2. **Treat the pen as what it is: an upright body of revolution.** With fixed
   orientation the pen only translates, and its swept volume over a straight
   chord is the convex hull of its two end poses. Five-axis machining checks
   interference the same way, by distance from workpiece points to the tool
   axis against the tool's radius at that height. The test for one wire then
   becomes: distance from the wire segment to the swept axis, against the pen's
   radius profile plus the wire radius plus the inflation from item 1. No
   general-purpose collision library is needed. No paper was found giving this
   exact test for wire segments; the construction is ours (*inference*). The
   small orientation drift inside a joint-space piece must be added to the
   inflation, and the designated contact point needs an explicit exemption.
   [Tool-axis interference checks](https://link.springer.com/10.1007/s00170-019-03369-y)
3. **Batched analytic distance.** Segment-to-segment distance is closed form and
   vectorises trivially: all wires near one edge, or all edges of an action, in
   one array operation, in numpy, numba or on the GPU. The whole Palace is about
   3x10^8 pen-primitive-against-wire pairs before culling: seconds on one core,
   under a second on the GPU (*inference*; no published benchmark found for this
   batch shape).
4. **One static index with birth order.** Wire is only added, in a known order.
   Build one grid over the *finished* wire set, each wire tagged with the action
   that lays it, and have every edge test only wires born before it. Garrett et
   al. use a single static bounding-volume hierarchy for spatial extrusion for
   the same reason. No incremental structure, no rebuilt scene per check.
   [Garrett et al., RSS 2020](https://arxiv.org/abs/2002.02360)
5. **Derive the deposited wire from the same chords.** The swept shape of each
   new wire is today re-derived by subdividing the polynomial piece. With the
   chord-plus-remainder bound it is the chord inflated by the same remainder.

Other spatial-extrusion planners (Choreo, FrameFab) make this check cheap by
discretising tool directions and using hand-written approximate tests; they are
not certified and do not transfer directly.
[Choreo](https://arxiv.org/abs/1810.00998)

**Expected effect (*inference*, not measured):** items 1 to 4 should take the
pen check from about 1.1 s per action to 1-10 ms on a CPU. The main risk is the
remainder constant in item 1: it has to be derived rigorously, including
orientation drift, and a wrong constant voids the certificate. It should be
introduced beside the current check and compared verdict for verdict.

## Timing

### What the literature says

- **TOPP-RA is not slow.** The original paper reports about 52 microseconds per
  grid point in Python (N=500, 30 constraints, 2013 desktop). We spend about
  6 ms per piece, a hundred times more. The excess is around the solver:
  per-point constraint assembly in Python, interpolator objects, trajectory
  reconstruction, and fixed set-up repeated for many small problems
  (*inference* from our profile, where constraint assembly and polynomial
  objects dominate). [TOPP-RA](https://arxiv.org/abs/1707.07239)
- **An exact forward-backward pass exists.** Consolini et al. give a provably
  optimal two-pass algorithm for the discretised problem under velocity,
  acceleration and torque limits, linear in the number of points, at about
  0.25 microseconds per point in C++. Dong and Stori prove the same for the
  bidirectional scan used in machining.
  [Consolini et al.](https://arxiv.org/abs/1802.03294)
- **Printer and CNC planners do this on microcontrollers.** Grbl, Marlin and
  Klipper run a reverse then forward pass of trapezoids per segment. With full
  stops at corners every straight run is independent.
  [Klipper kinematics](https://www.klipper3d.org/Kinematics.html)
- **Ruckig is not a fit for the joints.** It is microsecond-fast but passes
  through waypoints rather than following a fixed path, which would break the
  same-sweep certificate. Used on the single path variable it is legitimate.
  [Ruckig](https://github.com/pantor/ruckig)
- **Bounds without sampling.** Derivatives of a Bezier curve are Bezier curves,
  so limits on control points bound a whole piece. One matrix product bounds
  every piece of a build at once. [Bezier bounds](https://arxiv.org/pdf/1906.09785)

### What to build

1. **Closed-form profile per straight run, certified from the spline's own
   coefficients.** A drawing is a polyline traversed at a slow process speed
   with full stops at corners. For a run with cruise speed v and ramp
   acceleration a along the path, joint limits hold if, for every joint,
   `max|q'| * v <= velocity limit` and
   `max|q'| * a + max|q''| * v^2 <= acceleration limit`, where q' and q'' are
   the path derivatives of the cubic spline, read directly from its
   coefficients. At pen speeds the second term is negligible and the profile is
   simply nominal speed with ramps, which is also time-optimal whenever the tip
   speed is the active limit. Fall back to the solver where the inequalities
   fail. No paper states this certificate for arms; it is elementary
   (*inference*). Cost: microseconds per action.
2. **Bernstein audit for all pieces at once.** With constant path acceleration
   per piece and a cubic path, each joint is a degree-6 polynomial in time per
   piece. Convert every piece to Bernstein form with one fixed matrix and bound
   velocity and acceleration from the coefficients. Subdivide only the pieces
   whose bound fails. Replaces extrema-finding per joint per piece.
3. **Tip speed from the tip path, not from Jacobians.** The tip is already
   certified to follow the polyline within 0.3 mm. Its speed along the path is
   then path speed times a factor within a known tolerance of one, and the
   adaptive Jacobian subdivision is unnecessary for drawing edges (*inference*;
   the tolerance must be made explicit).
4. **If the solver stays:** assemble constraints for all grid points in one
   array call, skip trajectory reconstruction, or use the compiled `toppra` or
   Drake's implementation. Expect milliseconds per action.

**Expected effect (*inference*):** items 1 and 2 together are the only
combination that removes both the 1.3 s of retiming and the 1.0 s of audit per
action. The main risks are conservative ramps (a slightly longer print; the
drawing itself is unaffected when tip speed is the active limit) and making the
tip-speed argument in item 3 rigorous.

## Setup and library lookup

Measured 2026-10-13 on the Palace.

**Setup (27 s before the change below).** Three things, none of them planning:

- about 10 s screening the structure after every step of the build order, each
  frame assembled and solved from scratch on the workers;
- about 15 s surveying how well the library covers the object, which only
  informs the report. It now runs on a worker beside planning: Palace setup
  27 s to 11 s, branching 6.8 s to 2.4 s;
- the rest loading the cell, the library index and the robot models.

The structural screens can be made incremental: each step adds a few members to
a frame that was just factorised.

**Library lookup (1.0 s per action, 372 s of worker time).** Every lookup scores
all 80 records and then loads up to four of them from JSON files of 3-55 MB,
parsing and validating each, with eight kept per worker. Two fixes:

- seed each action from the same kind of action one layer below in the current
  build, which needs no lookup at all for most actions;
- store the part of a record that adaptation uses (the seed joint path and its
  source points) as compact arrays, loaded once before the workers start.

## Suggested order

1. Closed-form timing with the coefficient certificate, and the Bernstein audit.
2. Chord-plus-remainder bound and the axisymmetric pen test, run beside the
   current check until verdicts agree on every build.
3. Static birth-ordered wire index and batched distances.
4. Layer-below seeding; compact library records.
5. Incremental structural screens.
