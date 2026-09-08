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
similar as the roadmap of record. **That directory is local assignment
scaffolding and is not published** — it is gitignored, so a clone does not
contain it and those citations cannot be followed.

They are kept as historical provenance rather than rewritten, per the
never-delete convention: they record which roadmap stage a decision came from,
and the decision itself is always restated in `002_Architecture/settled.md`.
For anything you actually need to read: `003_Guides/` covers how a feature is
operated, `002_Architecture/settled.md` covers why it is built that way.

## Start here

Read [`005_AgentMgmt/INDEX.md`](005_AgentMgmt/INDEX.md) first — it explains
the boot protocol both for humans and for an AI agent picking up this
project cold.
