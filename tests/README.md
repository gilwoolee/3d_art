# Critical test suite

The routine suite has **29 tests total: 25 Python tests and four browser tests**.
The viewer API test was removed at the user's request.
It focuses on contracts that can admit an unsafe trajectory, lose source
geometry, misrepresent a replay, or break the active single-arm workflow.
Legacy dual-arm, deferred learning, implementation-detail and repeated variant
tests were removed. Variants of clock corruption share one rejection contract.

| Contract | Python tests |
| --- | ---: |
| Dense STL WirePrint geometry (including a solidified fork), whole zigzags, coverage, cooling and progressive support | 4 |
| Continuous collision, changed static geometry and growing material under reused proofs | 3 |
| Pen-only versus full-robot validation, fresh contact checks and clearance boundary parity | 3 |
| Fixed-orientation cache admission and lazy record integrity | 2 |
| Shared capsule journals and indexed material-prefix replacement | 2 |
| Repair invalidation and serial/parallel replay acceptance and rejection | 3 |
| Lossless packed clocks, corruption rejection and scoped local drawing proofs | 3 |
| Continuous dynamics, whole-action feed/vertex cooling, and an untimed sweep matching its timed clock | 3 |
| Turntable sweep, parked arm, settling and cooling | 2 |

The four browser tests cover exact clock evaluation, partial/full replay labels,
verification gating for newly saved schedules,
flange-relative tool geometry and independent robot transforms.
The existing export/replay contract also checks that export saves exact clocks
and feed events without invoking verification or creating binary sidecars, and
that the separate verifier publishes successful and rejected verdicts.

```bash
.venv/bin/pytest -q
tools/node/bin/node --test web/tests/*.test.mjs
.venv/bin/ruff check src tests
git diff --check
```
