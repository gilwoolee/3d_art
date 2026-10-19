# Tests

Five tests, the contracts most likely to let a wrong plan through:

| Test | Contract |
| --- | --- |
| `test_wireprint_nozzle.py` | The frame generator keeps slants within the pen's limit and keeps printable layers; connector wire leaves at most two odd nodes, so the frame is one trail, and never lies on other wire |
| `test_collision.py` | Continuous collision checking finds a collision between two clear endpoints |
| `test_timing.py` | Timing sweeps exactly the geometric path that was checked |
| `test_parallel_build.py::test_parallel_replay_reports_the_serial_rejection` | Verification reports a rejection |
| `test_parallel_build.py::test_coordinator_chains_continuing_strokes_and_drops_stale_results` | Planning chains a stroke that starts where the last one ended, reports every lift-off with its reason, and drops stale results |

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
```
