---
status: active
---

# 005_AgentMgmt — Boot Protocol

Agent startup is step-by-step, not a one-shot dump of every document.

```
Step 0 — Term alignment       → active/ctx_main/GLOSSARY.md
Step 1 — Safety first         → active/ctx_safety/README.md
Step 2 — System overview      → active/ctx_main/ctx_system_current.md
Step 3 — Task routing         → active/ctx_main/BOOT_MATRIX.md
Step 4 — Historical archive   → _historical/ (audit only, not implementation reference)
Step 5 — Conflict resolution  → active/ctx_main/TRUTH_LADDER.md
```

Steps 0 and 1 are unconditional — read them before writing any code, even if
the task seems unrelated to safety (there is no real robot connection yet,
but the habit matters once there is).

Humans should start at
[`active/ctx_main/readerBoot.md`](active/ctx_main/readerBoot.md) instead —
same information, plain-language framing.
