---
status: retired
scope: historical-archive
supersedes: null
superseded_by: wiki/002_Architecture/settled.md#S1.5
---

# `solve_ik_tcp` single-branch auto-pick (pre solution-list UI)

Archived record of `settled.md#S1.5`'s original decision, before the
"Inverse Kinematics" GUI panel gained a solution-list picker (see
[`settled.md#S1.5`](../../002_Architecture/settled.md) for the current
text). Kept for audit only — do not use as an implementation reference.

## Previous state

`solve_ik_tcp` computed every valid branch internally, then collapsed
them to a single winner before returning:

```python
best_angles, best_singular = min(valid, key=lambda pair: wrapped_dist(pair[0]))
status = "Solved" + (" (near wrist singularity)" if best_singular else "")
return best_angles, status
```

`gui_panel.py`'s "Solve IK" button received only that one solution — the
other valid branches were computed, filtered by joint limits, and then
discarded without ever being exposed to the caller.

## What changed and why

The user wanted to inspect/compare alternative valid IK solutions rather
than only ever receiving the one the backend silently picked. `solve_ik_tcp`
was changed to `sort()` the full `valid` list by the same wrapped-angle-
distance key (closest first, so index 0 reproduces the old behavior) and
return the whole list instead of collapsing it with `min()`. `gui_panel.py`
gained a `psim.ListBox` showing every valid branch, defaulting to index 0
but letting the user select any other row, which immediately re-applies
via `update_arm` (same "click → instant visual update" convention as the
rest of the panel).
