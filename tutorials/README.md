# FR5 Simulator — Build It Yourself

This directory is the human-readable roadmap for the whole project: how to get
from an empty Polyscope window to a six-axis FR5 that prints conformally onto a
curved surface and exports a solved job for a real controller.

It is written to be **followed**, not just read. Each stage states a goal, the
numbered pieces to build, and a `Verify:` checkpoint you can actually run before
moving on.

## Read in this order

| # | File | What it covers |
|---|---|---|
| 0 | [`wiki-beginner-guide.html`](wiki-beginner-guide.html) | The working method — why the wiki is an external brain, and how human/wiki/agent split the work. Open it in a browser first. |
| 1 | [`Stage1-4_README.md`](Stage1-4_README.md) | Project orientation: what you're building, the materials checklist, the learning path. |
| 2 | [`FR5_FK_Kit_README.md`](FR5_FK_Kit_README.md) | Stages 1–4 in detail — FK maths, mesh rendering, tool head/TCP, analytical IK. Environment setup lives here. |
| 3 | [`Stage5_README.md`](Stage5_README.md) | Flat-bed printing: build plate, G-code parsing, IK precompute, playback, caching. |
| 4 | [`Stage6_README.md`](Stage6_README.md) | Curved-surface printing: geodesics, print ordering, per-waypoint orientation, per-layer precompute. |
| 5 | [`Stage7_README.md`](Stage7_README.md) | Real calibration and job export: measured TCP and User Frame, orientation search, candidate filters, the external IK exchange format. |

Stages build strictly on each other — Stage 6 assumes Stage 5's precompute and
playback machinery, and Stage 7 assumes Stage 6's curved pipeline.

## Method

The construction methodology behind this project's `wiki/` — the five-layer
knowledge architecture, the boot matrix, the authoring templates — is documented
separately and project-agnostically in
[`../wiki-template/WIKI_CONSTRUCTION_GUIDE.md`](../wiki-template/WIKI_CONSTRUCTION_GUIDE.md).
Read it if you want to run the same working method on a different project.

## How these relate to the wiki

These READMEs are a **clean reconstruction**, not a diary. Each stage is written
as the thing you should build, in the order that works — so you never write code
that a later stage deletes.

The real project did not proceed that cleanly. Several things Stage 6 built
against stand-in data were found to be wrong once Stage 7 supplied the real
calibration, and those corrections have been folded back into Stage 6 where a
builder needs them. Where the wrong turn is instructive it survives as a short
**Pitfall:** note.

The unedited chronological record — every decision, its date, what superseded it,
and the measurements that settled it — is
[`../wiki/002_Architecture/settled.md`](../wiki/002_Architecture/settled.md),
with the working notes in [`../wiki/001_Inbox/`](../wiki/001_Inbox/). Each
sub-stage below cites its `settled.md` entry under `Full record:`. For how a
finished feature is *operated* rather than built, see
[`../wiki/003_Guides/`](../wiki/003_Guides/).
