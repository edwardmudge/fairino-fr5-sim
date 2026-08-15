---
status: inbox
stage: post-7.2
scope: geometry_backend.py (load_toolpath_precompute_cache, build_export_segments)
---

# A cached precompute exports zero segments — and used to pass validation

## What happened

Roadmap 7.2 added `build_export_segments()` and `validate_job()`. Found during
7.2's pre-commit review, not by the stage's own verification:

**`build_export_segments()` returns `[]` whenever the solved path came from disk
cache rather than from a fresh solve** — which is the normal case for both
toolpath sources once a cache exists.

Before the guard added alongside this note, `validate_job(vis, [])` then returned
**`ok=True`** and `format_validation` printed `==> job ACCEPTED`. An export
self-check reported a clean job for a job containing nothing.

## Root cause

`load_toolpath_precompute_cache()` restores five pieces of precompute state:

```python
self.precompute_joint_path = list(joint_path)
self.precompute_index      = len(joint_path)
self.precompute_total      = len(joint_path)
self.precompute_cache_meta = cached_meta
self.precompute_cache_path = cache_path
```

It does **not** restore `precompute_waypoints` or `precompute_R_target`, and
both runners `return` immediately on a hit — before
`_begin_toolpath_precompute()`, the only place those two are ever assigned. So a
cache hit leaves them at `None`, and `build_export_segments()`'s first guard
(`self.precompute_waypoints is None`) trips.

That guard is correct: without the waypoints there are no `is_feed_move` flags,
so there is no way to know where one continuous extrusion line ends and the next
begins. The joint path alone cannot be segmented.

Why the empty result then passed: rows 3–7 are all "no offender found" tests,
so over an empty segment list they pass vacuously at `n_points = 0`, and rows
1–2 never read the toolpath at all.

Repro — with a v6 planar cache on disk:

```
Run Precompute  ->  "Loaded 181375 waypoint(s) from cache"
build_export_segments()  ->  []
validate_job(vis, [])    ->  (True, ...)   # "==> job ACCEPTED"
```

## Why it was invisible until now

Nothing consumed `build_export_segments()` — 7.2 built it, and the GUI hookup is
7.5.

More to the point, `settled.md` S1.44's end-to-end planar verification was
measured **"after correctly rejecting the stale v5 cache"** — i.e. on the
fresh-solve path, the one where `_begin_toolpath_precompute()` had just
populated everything. The curved row-5 measurements read the cached joint paths
from a script that rebuilt the waypoints itself. Neither exercised the
cache-hit code path inside `VisContent`.

## Fixed here — only the safety half

`validate_job()` gained an **in-house row 0**, "job is non-empty", REJECT. It is
labelled in-house because it is *not* one of the spec's seven rows; the spec
never contemplates exporting nothing. `results` is now 8 long, row 0 first, then
the seven in table order.

That makes the failure loud rather than silent. **It does not fix the gap** — a
cached job still exports zero segments, it just now says so.

## Fix sketch for 7.4 (not implemented)

Persist what the segment builder needs, and restore it on load:

- `waypoint_positions` (N,3)
- `waypoint_is_feed` (N,) — the part that cannot be recovered any other way
- the `R_target` Z column (N,3) — store the normals directly rather than the
  full (N,3,3); it is all `build_export_segments()` reads, and on the planar
  path the (N,3,3) is a `broadcast_to` view of one constant anyway.

This is a cache schema change, so `PRECOMPUTE_CACHE_VERSION` **6 → 7** and every
cache is invalidated again (a ~120s planar re-solve plus both curved layers).
That cost belongs to 7.4, which needs these positions in the exported ply
regardless — not to 7.2, which would pay it for nothing.

**A cheaper curved-only half exists, if 7.3 wants it sooner.**
`run_curved_toolpath_ik_precompute()` already builds `waypoints` and
`R_target_array` *before* checking the cache — it has to, because the curved
cache key is computed from them — and simply discards them on a hit. Passing
them through to `load_toolpath_precompute_cache()` fixes curved with no schema
change and no invalidation.

Planar cannot do the same: it checks the cache *before* parsing, deliberately,
so that a hit skips the 187k-line G-code parse (roadmap 5.10). Restoring its
waypoints means either persisting them or giving up that saving. Persisting is
the right call, which is why the recommendation above is one fix at 7.4 rather
than two.

## Scope note

Deliberately **not** fully fixed as part of 7.2. 7.2's remit was the Rejection
Criteria; a schema change and a full re-solve of every cache is 7.4's territory,
and nothing is broken in the meantime because nothing calls the export path yet.
The guard is here because "reports ACCEPTED for an empty job" must not survive
into a commit, whether or not anything calls it today.

See `wiki/002_Architecture/settled.md` S1.44 "Known gap".
