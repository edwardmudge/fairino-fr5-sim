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
| IK | `docs/FR5_DH_Table.md`, `docs/FR5_Joint_Limits.md`, `docs/FR5_IK_Derivation.md` | `GLOSSARY.md` §1 | `geometry_backend.py` (`solve_ik`, `solve_ik_tcp`) | Anything describing IK as "not started" or "no IK code exists yet" |
| UI / sliders | `docs/Polyscope_Quickstart.md` (ImGui widgets section) | `docs/FR5_Joint_Limits.md` (slider ranges) | `gui_panel.py` | — |
| Build plate / G-code | `wiki/003_Guides/BuildPlate_UserFrame.md`, `wiki/003_Guides/Gcode_Toolpath.md` | `GLOSSARY.md` §3, `settled.md` S1.2/S1.3/S1.6/S1.7/S1.8/S1.10/S1.11 | `geometry_backend.py` (`load_build_plate`, `save_build_plate_position`, `load_saved_build_plate_position`, `parse_gcode`, `build_print_beads`, `load_gcode`, `set_print_reveal`), `gui_panel.py` ("Build Plate Orientation" panel) | Anything describing the build plate as translation-only; the G-code preview as a **curve network** / `GCODE_RADIUS_MM` (it is a swept bead surface mesh since S1.11); or the toolpath as not staying in sync when the plate moves |

★ = Getting this wrong renders the whole arm incorrectly — always read
before touching mesh transform code, per the warning in
`docs/FR5_Mesh_Convention.md` itself.
