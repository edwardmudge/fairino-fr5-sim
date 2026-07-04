---
status: active
---

# Truth Ladder

When sources disagree, this decides which one wins. Master rule: **if a
doc says X but the current code does Y, trust Y** and fix the doc.

```
1. Current code (geometry_backend.py, gui_panel.py, main.py) — highest authority
2. docs/*.md ground-truth tables (FR5_DH_Table, FR5_Joint_Limits, FR5_Mesh_Convention)
3. wiki/002_Architecture/settled.md
4. wiki/002_Architecture/* other design docs
5. README.md
6. wiki/001_Inbox/* — work notes, lowest priority
```

`docs/*.md` sits above `wiki/002_Architecture` because it captures
externally-verified ground truth (measured from the real robot / CAD
export) — architecture docs describe *how we chose to implement against*
that ground truth, which is more likely to shift as the code evolves.

## Example

If `wiki/001_Inbox/2026-XX-XX_ik_notes.md` describes an IK approach that
was later abandoned in favour of what `settled.md` records, `settled.md`
wins — the inbox note is left in place as history, not corrected.
