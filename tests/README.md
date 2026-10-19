# Tests

Five tests, the contracts most likely to let a wrong plan through:

| Test | Contract |
| --- | --- |
| `test_wireprint_nozzle.py` | The frame generator keeps slants within the pen's limit and keeps printable layers |
| `test_collision.py` | Continuous collision checking finds a collision between two clear endpoints |
| `test_timing.py` | Timing sweeps exactly the geometric path that was checked |
| `test_parallel_build.py::...reports_the_serial_rejection` | Verification reports a rejection |
| `test_parallel_build.py::test_coordinator_...` | Parallel planning repairs blocked strokes, drops stale results and chains strokes |

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
```
