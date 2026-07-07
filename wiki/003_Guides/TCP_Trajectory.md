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
3. Otherwise the point is appended to `self.trajectory_points`, and the
   curve network is rebuilt via `_update_trajectory_curve()`.

`self.tcp_world` itself is kept up to date inside `apply_delta_transform()`
— it's the same Delta_6-transformed TCP position used to drive the "TCP"
point cloud (index 7 in that method's loop), captured once per arm update
so `record_trajectory_point()` always has a current value to sample from.

`_update_trajectory_curve()` re-registers the curve network from scratch
on every new point (`ps.register_curve_network("Trajectory", nodes,
edges)`), since Polyscope curve networks don't support growing their node
count in place the way `update_vertex_positions` does for a fixed-size
mesh. It needs at least 2 points before a curve exists.

## How to tune it

Two module-level constants at the top of `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `TRAJECTORY_SAMPLE_INTERVAL_S` | Minimum seconds between recorded points. Lower = denser trail. |
| `TRAJECTORY_RADIUS_MM` | Trajectory line thickness, in world units (mm). Applied via `CurveNetwork.set_radius(TRAJECTORY_RADIUS_MM, relative=False)` each time the curve is rebuilt. |

## Code anchors

- `geometry_backend.py`: `record_trajectory_point()`,
  `_update_trajectory_curve()`, `set_trajectory_enabled()`, the
  `self.tcp_world` capture in `apply_delta_transform()`, and the
  `self.trajectory_enabled` / `self.trajectory_handle` state.
- `gui_panel.py`: the "Enable Trajectory" `psim.Checkbox` and the
  `self.content.record_trajectory_point()` call, both at the top of
  `render()`.
