# Instructions for AI assistants

This project keeps the record of a physical thing under the traceable-design standard.
For every task, follow `.claude/skills/traceable-design/SKILL.md`.

- Start each session with `python tools/tdr.py status` and read `STATUS.md` only.
- Look values up with `python tools/tdr.py get <id or key>`, or the MCP server `tdr` (`python tools/tdr.py mcp`).
- Before finishing a change, run `python tools/tdr.py check` and `python tools/tdr.py status`, and end the change description with its tier trailers.

## About this project

- Owner: the owner of this repository (fujisato-koji). This is a personal project; bring in nothing from the owner's work.
- What the thing is: a drone (unmanned aircraft) that the owner designs and builds, aiming to be certifiable in Japan. Which certification (type certification or aircraft certification, class 1 or class 2) is open item Q-001. What the drone is for is Q-002; the repository name `drone` is provisional until then.
- Language: English for the record, the docs, and commit messages; Japanese only for talking with the owner (D-007). Ids, keys, and allowed values stay as the standard defines them.
- Where the owner keeps private facts: a local folder outside this repository, `C:\Users\user\Documents\drone-private` (never published). Never write where the owner lives or flies.
- Writing style: one sentence per line; no middle dots (・) and no dashes (— or –).
