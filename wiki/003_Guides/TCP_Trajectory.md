---
status: active
---

# How to Create a TCP Trajectory

## What it is

The `"Trajectory"` curve network is a trail of TCP world positions, traced
as the arm moves. It renders as a Polyscope line running through every
recorded TCP point, letting you see the path the tool tip has taken.

## How it's triggered

Automatic once enabled. `gui_panel.py`'s `render()` calls
`self.content.record_trajectory_point()` at the top of every frame
(`UI_Menu.render`, `gui_panel.py`), which is Polyscope's existing
per-frame callback path (`main.py` -> `ps.set_user_callback(callback_loop)`
-> `ui.render()`). No changes to `main.py` were needed — it already runs
every frame regardless of whether the user is touching a slider.
`record_trajectory_point()` is called unconditionally every frame; whether
it actually records anything is gated by the "Enable Trajectory" checkbox,
see below.

## Enabling / disabling it

An "Enable Trajectory" checkbox sits at the top of the control panel
(`UI_Menu.render()`, `gui_panel.py`), checked by default. Toggling it
calls `content.set_trajectory_enabled(bool)` (`geometry_backend.py`):

- **Unchecked**: `self.trajectory_enabled` is set to `False`, so
  `record_trajectory_point()` bails out immediately at the top (no new
  points recorded), and `self.trajectory_handle.set_enabled(False)` hides
  the curve from the Polyscope scene.
- **Re-checked**: `self.trajectory_enabled` is set back to `True` —
  recording resumes and `self.trajectory_handle.set_enabled(True)` makes
  the curve visible again. Previously recorded points are kept, not
  cleared, so the curve picks up where it left off.

## Sampling behaviour

`VisContent.record_trajectory_point()` (`geometry_backend.py`):

1. At most one point is recorded every `TRAJECTORY_SAMPLE_INTERVAL_S`
   seconds (a wall-clock gate, checked with `time.time()`).
2. Once that window has elapsed, the current TCP world position
   (`self.tcp_world`) is compared to the last recorded point with
   `np.allclose(...)`. If they match — the arm hasn't moved since the
   last sample — the point is discarded, not appended.
3. Otherwise the point is appended to `self.trajectory_points` — this list
   stays dense, one entry per accepted sample, regardless of the render
   throttle below.

`self.tcp_world` itself is kept up to date inside `apply_delta_transform()`
— it's the same Delta_6-transformed TCP position used to drive the "TCP"
point cloud (index 7 in that method's loop), captured once per arm update
so `record_trajectory_point()` always has a current value to sample from.

`_update_trajectory_curve()` re-registers the curve network from scratch
(`ps.register_curve_network("Trajectory", nodes, edges)`), since Polyscope
curve networks don't support growing their node count in place the way
`update_vertex_positions` does for a fixed-size mesh. It needs at least 2
points before a curve exists.

Unlike `trajectory_points`, this re-registration is **throttled**: it only
fires every `TRAJECTORY_CURVE_RENDER_STRIDE` accepted samples, tracked by
`self._trajectory_curve_sample_count` (roadmap `Stage5_README.md` 5.9,
`settled.md` S1.17) — the O(n) rebuild cost otherwise scales with the
whole trail's length on every single new point. The data backing the
curve is never decimated, only how often it's pushed to Polyscope; the
curve is treated as a decimatable debug overlay, not the exported path.
One consequence: if recording stops mid-window (e.g. the arm stops moving,
or the checkbox is unchecked), up to `TRAJECTORY_CURVE_RENDER_STRIDE - 1`
recorded points can sit un-drawn until the next accepted sample pushes the
count over the threshold — the trail can visibly lag behind
`trajectory_points` by that many points while idle.

## How to tune it

Four module-level constants at the top of `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `TRAJECTORY_SAMPLE_INTERVAL_S` | Minimum seconds between recorded points. Lower = denser trail. |
| `TRAJECTORY_RADIUS_MM` | Trajectory line thickness, in world units (mm). Applied via `CurveNetwork.set_radius(TRAJECTORY_RADIUS_MM, relative=False)` each time the curve is rebuilt. |
| `TRAJECTORY_CURVE_RENDER_STRIDE` | The **floor** of the redraw stride (see "Changed at v1.0" below — it was a fixed value until then). How many accepted samples accumulate in `trajectory_points` before `_update_trajectory_curve()` re-registers the curve. Higher = fewer rebuilds, more visible lag while idle. |
| `TRAJECTORY_CURVE_NODES_PER_STRIDE` | Grows the redraw stride by 1 per N recorded points, so the rebuild interval scales with the O(n) rebuild cost. |

## Code anchors

- `geometry_backend.py`: `record_trajectory_point()`,
  `_update_trajectory_curve()`, `set_trajectory_enabled()`, the
  `self.tcp_world` capture in `apply_delta_transform()`, and the
  `self.trajectory_enabled` / `self.trajectory_handle` /
  `self._trajectory_curve_sample_count` state.
- `gui_panel.py`: the "Enable Trajectory" `psim.Checkbox` and the
  `self.content.record_trajectory_point()` call, both at the top of
  `render()`.

## Changed at v1.0 (2026-09-08)

**The redraw stride is now derived from the point count, not fixed at 5**
(`settled.md` **S1.69**). `record_trajectory_point()` tests against
`_trajectory_render_stride()`:

```python
max(TRAJECTORY_CURVE_RENDER_STRIDE,
    len(trajectory_points) // TRAJECTORY_CURVE_NODES_PER_STRIDE)
```

Why: `trajectory_points` is unbounded — only `clear_trajectory()` (the FK panel's
Reset) empties it — while `_update_trajectory_curve()` is an O(n) full
re-registration, because Polyscope curve networks have no incremental grow API.
A fixed stride therefore fired a growing rebuild at a constant ~2/sec, so redraw
work grew O(n²) over a session.

Measured at ~0.31 µs/node:

| Points | Rebuild | Derived stride | Amortised cost |
|---|---|---|---|
| 500 | 0.22 ms | 5 (unchanged) | 0.05% |
| 10,000 | 2.99 ms | 10 | 0.37% |
| 30,000 | 9.40 ms | 30 | 0.39% |
| 60,000 | 18.89 ms | 60 | 0.46% |

30,000 points is roughly a **50-minute planar playback at speed 1**. The sessions
that actually happen are far smaller — curved RX at speed 1 reaches about 529
points, planar at speed 100 about 302 — so this was a real but narrow problem,
and the fix is scoped to match.

Two properties worth knowing before tuning either constant:

- **Below 5,000 points nothing changed.** The `max()` floor pins the stride at
  exactly 5, so every realistic session behaves as it did before. This is a
  tail-case fix that deliberately does not perturb the common case.
- **No point is ever discarded.** Only redraw *frequency* changes. Capping the
  list was considered and rejected: the trail is a debug overlay whose whole value
  is showing where the TCP has been, so silent truncation would be a behaviour
  change rather than a performance one.

Same reasoning as **S1.55**, which replaced the fixed `PLAYBACK_RENDER_STRIDE`
with one derived from the solved path's own joint motion — a fixed count only
means a fixed cost if the thing being counted has fixed cost.
