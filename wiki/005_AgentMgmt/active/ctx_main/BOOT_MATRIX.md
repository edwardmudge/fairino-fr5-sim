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
| Build plate / G-code | `wiki/003_Guides/BuildPlate_UserFrame.md`, `wiki/003_Guides/Gcode_Toolpath.md` | `GLOSSARY.md` §3, `settled.md` S1.2/S1.3/S1.6/S1.7/S1.8/S1.23 | `geometry_backend.py` (`load_build_plate`, `save_build_plate_position`, `load_saved_build_plate_position`, `parse_gcode`, `load_gcode`), `gui_panel.py` ("Build Plate Orientation" panel) | Anything describing the build plate as translation-only, or the G-code toolpath as auto-reloading/staying in sync when the plate moves (removed by S1.23 — an explicit "Load G-code preview" click is required) |
| Toolpath execution / IK precompute / playback / caching | `settled.md` S1.14–S1.24 | `GLOSSARY.md` §5 | `geometry_backend.py` (`run_/step_/pause_/cancel_toolpath_ik_precompute`, `save_/load_toolpath_precompute_cache`, `run_/pause_/reset_/advance_toolpath_playback`, `_reset_toolpath_playback_state`), `gui_panel.py` ("Toolpath Settings" panel) | Anything describing toolpath execution as not started, precompute/playback as immune to the plate moving mid-session, or `cancel_toolpath_ik_precompute()` as safe to call without resetting playback (fixed in S1.24) |

★ = Getting this wrong renders the whole arm incorrectly — always read
before touching mesh transform code, per the warning in
`docs/FR5_Mesh_Convention.md` itself.
