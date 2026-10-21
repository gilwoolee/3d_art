# Agent instructions

Start with [README.md](README.md): setup, the commands and where each step's
code lives.

## Rules from the user

- **No `Co-Authored-By:` trailers** in commits or pull requests.
- **Planning uses 24 workers.** Long runs go in tmux, because session restarts
  kill background jobs.
- **Per object:** frame wire at most 10 m; detail about 2 m and at most 15
  extra minutes of print time.
- **Chain strokes as much as possible; minimize lift-offs.** The arm runs
  open loop and cannot be trusted to return to the same pose, and every rejoin
  is a joint onto wire that may have sagged.
- **The pen must not twist wire while depositing:** one pen orientation (tilt
  and lean direction) per stroke.
- **One arm, no turntable, geometry-first planning only.**
- **Report plainly and faithfully:** what passed, what failed, what was not
  measured. Everything is simulation.
- **Never `pkill -f` a pattern that can match your own shell;** find processes
  by their executable (`/proc/PID/exe`) and kill by PID.
- **Unset `PYTHONPATH` first** if the shell sets it (for example for ROS).
- **Headless Chrome over SSH needs `env -u DISPLAY`.**

## Keeping the code small

- A step's entry point calls one function per stage; keep functions short
  enough to read on one screen.
- No options the CLI never sets, no outputs nothing reads, no code kept for
  several arms, a turntable or another planner. Delete dead code instead of
  keeping it "just in case".
- Keep runs reproducible: never let the order of a set or dict of names decide
  what gets built.

## Checks

There are no tests. Check a change by running the ghost through every step
in the README and comparing the lines each step prints.

```bash
.venv/bin/ruff check src
(cd viewer && PATH=$PWD/../.cache/node/bin:$PATH npm run build)
```
