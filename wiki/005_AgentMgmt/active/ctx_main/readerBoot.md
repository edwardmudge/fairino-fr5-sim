---
status: active
---

# Welcome — Human-Friendly Onboarding

If you're a person (not an agent) picking this project up, here's the
short version:

1. This is a from-scratch FK/IK simulator for a 6-axis robot arm, rendered
   in 3D with Polyscope.
2. **You** make the judgement calls (is the math right? which IK solution
   makes sense?). **`docs/`** holds the ground-truth reference tables you'd
   otherwise have to memorise. **The AI agent** helps implement and debug,
   but doesn't decide for you.
3. The 4-stage roadmap (FK maths → mesh rendering → tool head/TCP → IK) is
   complete, plus G-code toolpath preview, TCP trajectory recording, and a
   full toolpath execution pipeline (chunked/cached IK precompute,
   progressive-reveal playback) — see `ctx_system_current.md` for the
   per-feature status.
   Two more stages landed after that list was written, and they are now the
   bulk of the project: **curved-surface printing** (conformal toolpaths on a
   curved part — geodesic travel routing, per-waypoint tool orientation, a
   540-frame orientation search behind nine candidate filters) and **job
   export** (writing a validated robot job for an external IK team). If you
   are here for curved printing, start at
   [`../../../003_Guides/CurvedModel_PrintSetup.md`](../../../003_Guides/CurvedModel_PrintSetup.md),
   and read
   [`../../../003_Guides/CurvedModel_AdaptingYourOwnJob.md`](../../../003_Guides/CurvedModel_AdaptingYourOwnJob.md)
   before pointing it at a part of your own.
4. Before touching mesh rendering code, read
   `docs/FR5_Mesh_Convention.md` — skipping it guarantees broken rendering.
5. `wiki/002_Architecture/settled.md` will fill up with decisions as you
   make them — treat it as "don't re-litigate this" once an entry exists.

For the AI agent's equivalent entry point, see
[`ctx_system_current.md`](ctx_system_current.md).
