# Fairino FR5 Simulator Wiki

> Living documentation for the FR5 6DOF forward/inverse kinematics simulator.
> Human-authored knowledge that does NOT belong in code comments.

## Sections

| # | Section | Purpose |
|---|---------|---------|
| 001 | [Inbox](001_Inbox/INDEX.md) | Work notes, experiment logs, dead ends |
| 002 | [Architecture](002_Architecture/INDEX.md) | Design docs, settled decisions |
| 003 | [Guides](003_Guides/INDEX.md) | Operation guides |
| 004 | [Ops](004_Ops/INDEX.md) | Deployment, environment config |
| 005 | [AgentMgmt](005_AgentMgmt/INDEX.md) | Agent context management |

## Conventions

- **One topic per file.** Split when exceeding ~300 lines.
- **INDEX.md** in every folder — serves as table of contents.
- **Frontmatter** (YAML `---` block): `status`, `scope`.
- **Link, don't duplicate.** Cross-references use relative paths.
- **Status tags:** `draft` → `active` → `retired`.

## A note on `tutorials/` references

Pages throughout this wiki (and docstrings throughout the source) cite
`tutorials/Stage5_README.md`, `Stage6_README.md`, `Stage7_README.md` and
similar as the roadmap of record. That directory **is published** — a clone
contains it and every cited sub-stage number resolves to a section.

One caveat when cross-reading. The stage READMEs are a clean reconstruction
written to be followed in order: corrections discovered in a later stage have
been folded back into the stage where a builder needs them, so a sub-stage may
describe the corrected approach rather than the one originally built there. This
file's own chronology is the authority — `002_Architecture/settled.md` records
every decision with its date and what superseded it, and `001_Inbox/` holds the
working notes. `003_Guides/` covers how a finished feature is operated.

## Start here

Read [`005_AgentMgmt/INDEX.md`](005_AgentMgmt/INDEX.md) first — it explains
the boot protocol both for humans and for an AI agent picking up this
project cold.
