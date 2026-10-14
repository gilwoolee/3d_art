# Agent instructions

Start with [docs/HANDOFF.md](docs/HANDOFF.md): current state, setup on a new
machine, how to run, next work. Adding detail to a frame:
[.claude/skills/detail-tours/SKILL.md](.claude/skills/detail-tours/SKILL.md).

## Rules from the user

- **No `Co-Authored-By:` trailers** in commits or pull requests.
- **Fewer than 30 tests in all** (now 25 Python and 4 browser tests). Add
  assertions to existing test functions instead of adding tests.
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
env -u DISPLAY tools/node/bin/node --test web/tests/*.test.mjs
(cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)
```
