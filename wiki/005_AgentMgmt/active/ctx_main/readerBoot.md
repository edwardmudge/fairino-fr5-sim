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
3. Nothing is built yet except a blank 3D window — start at Stage 1 (FK
   maths) using `docs/FR5_DH_Table.md`.
4. Before touching mesh rendering code, read
   `docs/FR5_Mesh_Convention.md` — skipping it guarantees broken rendering.
5. `wiki/002_Architecture/settled.md` will fill up with decisions as you
   make them — treat it as "don't re-litigate this" once an entry exists.

For the AI agent's equivalent entry point, see
[`ctx_system_current.md`](ctx_system_current.md).
