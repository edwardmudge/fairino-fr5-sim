---
status: active
---

# Boot Matrix

Task type → required reading. Load only what's relevant, not everything.

| Task Type | Required Reading | Follow-up Reading | Code Anchor | Do NOT Treat as Current |
|-----------|-------------------|--------------------|--------------|--------------------------|
| FK maths | `docs/FR5_DH_Table.md` | `GLOSSARY.md` §1 | `geometry_backend.py` | — |
| Mesh rendering | `docs/FR5_Mesh_Convention.md` ★ | `docs/Polyscope_Quickstart.md`, `GLOSSARY.md` §2 | `geometry_backend.py`, `gui_panel.py` | — |
| Tool head / TCP | `docs/FR5_Mesh_Convention.md` (Nozzle and TCP section) | `GLOSSARY.md` §3 | `assets/printerHead/TCP.txt` | — |
| IK (Stage 4, not started) | `docs/FR5_DH_Table.md`, `docs/FR5_Joint_Limits.md` | `GLOSSARY.md` §1 | — (no IK code exists yet) | — |
| UI / sliders | `docs/Polyscope_Quickstart.md` (ImGui widgets section) | `docs/FR5_Joint_Limits.md` (slider ranges) | `gui_panel.py` | — |

★ = Getting this wrong renders the whole arm incorrectly — always read
before touching mesh transform code, per the warning in
`docs/FR5_Mesh_Convention.md` itself.
