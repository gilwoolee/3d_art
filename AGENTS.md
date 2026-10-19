# Agent instructions

Start with [README.md](README.md): what the code does, setup, how to run, and
a code map.

## Rules from the user

- **No `Co-Authored-By:` trailers** in commits or pull requests.
- **Exactly 5 tests.** Add assertions to existing test functions instead of
  adding tests.
- **Planning uses 24 workers.** Long runs go in tmux, because session restarts
  kill background jobs.
- **Per object:** frame wire at most 10 m; detail about 2 m and at most 15
  extra minutes of print time.
- **The pen must not twist wire while depositing:** one tilt per stroke and
  one turntable face per stroke.
- **Report plainly and faithfully:** what passed, what failed, what was not
  measured. Everything is simulation.
- **Never `pkill -f` a pattern that can match your own shell.**
- **Headless Chrome over SSH needs `env -u DISPLAY`.**

## Checks

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
(cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)
```
