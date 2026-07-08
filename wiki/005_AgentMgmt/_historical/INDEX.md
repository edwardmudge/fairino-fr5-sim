---
status: active
---

# _historical — Archive

Superseded code layouts and design notes, kept for audit only. **Not an
implementation reference** — see
[`TRUTH_LADDER.md`](../active/ctx_main/TRUTH_LADDER.md) (`_historical/*` is
the lowest-priority source; if it contradicts current code or
`settled.md`, current code/`settled.md` wins).

## Entries

| Date | Topic | Superseded by |
|------|-------|---------------|
| 2026-07-07 | [`end_effector_position_reorg`](2026-07-07_end_effector_position_reorg.md) — module-level FK layout before `VisContent` reorg | [`settled.md#s11-geometry-logic-lives-on-viscontent-not-as-bare-module-functions`](../../002_Architecture/settled.md) |
| 2026-07-08 | [`ik_single_branch_autopick`](2026-07-08_ik_single_branch_autopick.md) — `solve_ik_tcp` collapsing to one winning branch, before the multi-solution picker | [`settled.md#s15-ik-multi-solution-branches-are-filtered-by-joint-limits-then-ranked-by-proximity-to-the-current-pose----all-valid-branches-are-returned-not-just-the-closest`](../../002_Architecture/settled.md) |
