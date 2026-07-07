# Wiki Construction Guide — Knowledge Management Architecture for Human-AI Collaborative Projects

> **Purpose:** A reusable construction methodology for structuring a project's
> documentation as a knowledge base an AI agent can navigate reliably.
> Follow this guide directly to initialise a new project's wiki structure,
> CLAUDE.md, and agent management system.
>
> **Scope:** Any medium-to-large engineering project that involves deep AI
> agent participation in development.

---

## 1. Core Motivation: Why Do We Need This Kind of Wiki?

### 1.1 The Root Problem

AI agents have limited context windows, non-persistent memory, and a tendency to "confidently guess wrong." In Human-AI collaborative development, the following problems recur:

| Problem | Consequence |
|---------|-------------|
| Agent interprets domain terms inconsistently | The same word gets different meanings in different conversations (e.g., a status flag or lifecycle term means something different each time it comes up) |
| No information priority rules | When wiki docs conflict with code, the agent doesn't know which to trust |
| All context loaded at once | Large volumes of irrelevant information crowd the context window, degrading reasoning quality |
| Safety-critical knowledge not force-loaded | Agent skips safety rules and writes code directly, reintroducing known-fixed bugs |
| "Lessons learned" exist only in conversation memory | The same mistake gets repeated in new sessions (e.g., unit mismatches, API parameter semantics) |
| CLAUDE.md only says "how to do X," not "why" | Agent can't make correct judgements in edge cases because it doesn't understand the failure case behind the rule |

### 1.2 Design Goals

1. **Load on demand, don't waste context** — Agent reads only task-relevant documents based on task type (Boot Matrix routing)
2. **Conflicts have resolution rules** — When multiple sources contradict, there's an explicit priority ladder (Truth Ladder)
3. **Safety knowledge is front-loaded by default** — Operations involving hardware/safety must read safety rules before starting
4. **Failure experience is crystallised** — Every safety rule traces back to a real fault event (Postmortem → Rule)
5. **Terms defined once** — Glossary eliminates ambiguity; every easily confused concept pair has a precise definition
6. **Documents self-expire** — Frontmatter metadata marks document validity and scope, preventing outdated docs from being treated as truth

---

## 2. Overall Architecture: Five-Layer Knowledge Management

```
┌─────────────────────────────────────────────────────┐
│  Layer 0: CLAUDE.md (project root)                   │
│  Behavioural principles + technical hard rules + SDK  │
│  discipline                                          │
│  ─ Auto-loaded every time the agent starts            │
├─────────────────────────────────────────────────────┤
│  Layer 1: wiki/005_AgentMgmt/ (Agent Management)     │
│  Boot Protocol → Glossary → Safety → Boot Matrix     │
│  → Truth Ladder → System Current                     │
│  ─ Loaded step-by-step per protocol                  │
├─────────────────────────────────────────────────────┤
│  Layer 2: wiki/002_Architecture/ (Architecture)      │
│  System architecture + subsystem design + settled    │
│  decisions (settled.md)                              │
│  ─ Loaded on trigger via Boot Matrix                 │
├─────────────────────────────────────────────────────┤
│  Layer 3: wiki/001_Inbox/ + wiki/003_Guides/         │
│  Work notes + operation guides                       │
│  ─ Referenced manually on demand, never auto-loaded  │
├─────────────────────────────────────────────────────┤
│  Layer 4: _historical/ (Archive)                     │
│  Superseded design documents                         │
│  ─ For archaeology only, not implementation reference │
└─────────────────────────────────────────────────────┘
```

**Key Insight: Layers are organised by authority, not creation date.** Information closer to code and hardware has higher authority.

---

## 3. Detailed Design of Each Layer

### 3.1 Layer 0: CLAUDE.md — The Agent's Behavioural Constitution

CLAUDE.md is the instruction file auto-loaded every time the agent starts. Its design principles:

#### What Goes Into CLAUDE.md

| Category | What to include | What NOT to include |
|----------|----------------|---------------------|
| **Behavioural principles** | Think Before Coding, Simplicity First, Surgical Changes, Goal-Driven Execution | Specific technical details (put in wiki) |
| **SDK/API discipline** | "When encountering uncertain API parameters, FIRST action must be to read SDK source docstrings" | Specific API parameter values (put in wiki ctx_safety or glossary) |
| **Hard rules (with why)** | Each rule includes a failure event description, e.g. `retry_mode=1 permanently removed` + why it was removed | Rules without a "why" (agent won't understand the reason and will bypass the rule in edge cases) |
| **Code modification discipline** | Documentation update standards, language preferences, render-loop performance rules | Project progress, current task status (put in wiki) |
| **Performance rules** | Specify which operations cannot appear in which code paths (e.g., no heavy IO in UI render loop) | Optimisation data and benchmarks (put in wiki) |

#### 4 Authoring Disciplines for CLAUDE.md

1. **Rule + Why + Failure Case — all three together**
   - Bad: `Don't use the legacy retry_mode=1 flag`
   - Good: `retry_mode=1 permanently removed, must not be reintroduced. Reason: this flag silently changed retry counts from a fixed number to a percentage of the queue depth; the ambiguous SDK docs caused a production run to retry far more aggressively than intended`

2. **Verifiability**
   - Bad: `Code should be clean`
   - Good: `If you write 200 lines and it could be 50, rewrite it. Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.`

3. **Explicit scope declaration**
   - `This rule applies to: core/local_ik_solver.py, core/path_ik_resolve.py, scripts/placement_search.py, and any future script that uses Local IK.`

4. **Propagation to sub-agents**
   - `This applies to all agents and all contexts — delegate this principle when spawning sub-agents for SDK-related tasks.`

#### CLAUDE.md Structure Template

```markdown
# Project Principles
## 1. Think Before Coding
## 2. Simplicity First
## 3. Surgical Changes
## 4. Goal-Driven Execution

## SDK / API Investigation Rule
[Read SDK source first, don't guess]

## [Domain] Notes
[Specific technical hard rules, each with a "why"]

## Documentation Updates
[Documentation update discipline]

## Language
[Language preference]

# Project Agent Memory
## [Performance/Safety Rule Name]
[Architecture-level constraints learned from project operation]
```

---

### 3.2 Layer 1: Agent Management Layer (005_AgentMgmt/)

This is the wiki's core innovation: a **context loading protocol** designed for AI agents.

#### 3.2.1 Boot Protocol (INDEX.md)

Agent startup is step-by-step with dependencies — not a one-shot dump of all documents.

```
Step 0 — Term alignment (GLOSSARY.md)       ← Without shared language, everything else is wasted
Step 1 — Safety first (ctx_safety/*)        ← Hardware safety rules cannot be skipped
Step 2 — System overview (ctx_system_current) ← Know what the system currently looks like
Step 3 — Task routing (BOOT_MATRIX.md)       ← Load task-specific documents
Step 4 — Historical archive (_historical/)   ← For audit only, not implementation reference
Step 5 — Conflict resolution (TRUTH_LADDER.md) ← Resolution rules for conflicting information
```

**Insight: Steps 0 and 1 are unconditional prerequisites.** Without term alignment, the agent misunderstands tasks; without safety rules, the agent repeats already-fixed bugs.

#### 3.2.2 Glossary — Term Disambiguation

The Glossary is not a simple dictionary — it's a **precision disambiguation table for easily confused concept pairs**.

Design principles:
- **Group by confusion scenario**, not alphabetically. E.g., group all terms related to a single recurring point of confusion together, rather than scattering them alphabetically
- **Bilingual, if your team is**: Term definitions in every language your team works in, to eliminate translation ambiguity
- **Include code anchors**: Each term lists the corresponding code file and variable name
- **Don't include obvious terms**: Only include terms that are easily confused. If a term has a single unambiguous meaning, it doesn't need a Glossary entry

Template:
```markdown
## N. [Confusion Scenario Name]

| Term | Definition | Notes |
|------|-----------|-------|
| **A** | Precise definition | Code file, variable name |
| **B** | Precise definition (differs from A in that...) | Code file, variable name |
```

#### 3.2.3 Truth Ladder — Conflict Resolution Priority

When documents contradict each other, the question isn't "who wrote it right" — there's an explicit hierarchy:

```
1. Current code + config + tests               ← Highest authority
2. ctx_safety hard rules + settled.md + CLAUDE.md hard rules
3. active/ctx_* current contracts
4. wiki/002_Architecture/* design docs
5. README / PROJECT.md / TIMELINE.md
6. Planning documents
7. wiki/001_Inbox/* work notes
8. _historical/* archive                       ← Lowest priority
```

**Master Rule:** If memory/docs say X, but current code does Y, **trust Y** and update the outdated docs.

**Design Insights:**
- **Safety rules (ctx_safety) are explicitly elevated to Level 2**, same tier as settled.md — ordinary ctx_* documents (Level 3) cannot override safety rules
- **Concrete conflict resolution examples must be provided**, otherwise the agent still hesitates when facing real conflicts

#### 3.2.4 Boot Matrix — Task Routing Table

Different task types require reading different documents. The Boot Matrix is a table mapping **task types to required reading + code anchors + test anchors**.

```markdown
| Task Type | Required Reading | Follow-up Reading | Code Anchor | Test Anchor | Deprecated — Do NOT Use |
|-----------|-----------------|-------------------|-------------|-------------|------------------------|
| Auth flow      | ctx_safety/*     | ctx_auth/*      | auth.py         | test_auth.py         | _historical/... |
| Data pipeline  | ctx_safety/*     | ctx_pipeline/*  | pipeline.py     | test_pipeline.py     | ... |
```

**Insight: The Boot Matrix doesn't just say "what to read" — it also includes "what NOT to read" (Do NOT Treat as Current column).** This prevents the agent from extracting incorrect information from outdated historical documents.

#### 3.2.5 ctx_safety/ — Mandatory Safety Rules

Safety rules are distilled from **real fault events**, each with a postmortem:

```markdown
# R1: [Rule Name]
## Rule Content
[Clear, actionable prohibition/requirement]

## Source Event (Postmortem)
[What happened, what were the consequences, how was it discovered]

## Why This Rule Is Necessary
[What happens if it's not followed]
```

**Insight: A safety rule without a postmortem is ineffective.** If the agent doesn't understand "why," it will bypass the rule in seemingly reasonable situations. Humans are the same — without a painful lesson, you don't understand why the rule exists.

#### 3.2.6 ctx_* Domain Contexts

Context folders are divided by subsystem, one ctx_* directory per subsystem:

```
active/
├── ctx_main/                  ← System overview, routing, terms, conflict resolution
├── ctx_safety/                ← Mandatory safety rules (independently elevated priority)
├── ctx_[subsystem_a]/         ← e.g. the ingestion pipeline
├── ctx_[subsystem_b]/         ← e.g. the auth/session layer
├── ctx_[subsystem_c]/         ← e.g. the reporting/export layer
├── ctx_docs/                  ← Documentation audit
└── ctx_completed/             ← Completed task reference
```

Each ctx_* directory has a README.md describing the scope and current status of that subsystem.

**Insight: ctx_completed/ is not "deletion" — completed task documents are moved here rather than deleted, because they may be needed for rollback reference. But they are explicitly marked as "not a current implementation reference."**

#### 3.2.7 Document Frontmatter Metadata

Each key document has YAML frontmatter to help the agent judge document validity:

```yaml
---
status: active | retired | draft
scope: current-truth | boot-routing | deep-review | doc-only
verification_level: doc-only | sim-verified | production-verified
last_verified_against_code: 2026-05-03 | null
canonical_code:
  - core/engine.py
canonical_tests:
  - tests/test_engine.py
supersedes:
  - _historical/old_doc.md
superseded_by: null
do_not_use_for:
  - deep code modification — read another doc instead
---
```

**Insight: `do_not_use_for` is reverse guidance.** It tells the agent "this document covers a certain topic, but is NOT suitable for a specific purpose" — preventing the agent from extracting deep implementation details from overview documents.

---

### 3.3 Layer 2: Architecture Design Layer (002_Architecture/)

Organised by subsystem, one directory per subsystem:

```
002_Architecture/
├── architecture.md           ← System panorama (threading model, data flow, pipelines)
├── settled.md                ← Settled design decisions (S1.1, S1.2, ...)
├── design_insights.md        ← Engineering wisdom learned from iteration
├── system_diagrams.md        ← Mermaid architecture diagrams
├── [subsystem_a]/            ← One subsystem, e.g. an ingestion pipeline
│   └── INDEX.md
├── [subsystem_b]/            ← Another subsystem, e.g. a streaming/export engine
│   └── INDEX.md
├── [subsystem_c]/
├── [subsystem_d]/
└── [subsystem_e]/
```

#### The Significance of settled.md

settled.md collects all "decisions that have been made and should not be re-discussed" architectural constraints. Format:

```markdown
## S1.1 Config Writes Must Block-Wait
**Decision:** ...
**Reason:** ...
**Non-revertible unless:** the underlying constraint changes
**Verified on:** 2026-04-15
```

**Insight: The value of settled.md is telling the agent "these things don't need to be reconsidered."** Without settled.md, the agent will repeatedly propose already-rejected approaches (e.g., "we could make this write asynchronous" — which is infeasible given a constraint the agent doesn't know about, but sounds reasonable from a pure software perspective).

---

### 3.4 Layer 3: Workspace (001_Inbox/ + 003_Guides/)

- **001_Inbox/**: Work notes organised by date, plan drafts, code review records. They are **non-authoritative** (Truth Ladder priority 7)
- **003_Guides/**: User-facing operation guides

**Insight: Inbox's low authority is by design.** Work notes often contain "ideas at the time" that were later rejected. If Inbox contradicts settled.md, settled.md wins.

---

### 3.5 Layer 4: Archive (_historical/)

Superseded design documents are moved to `_historical/`, not deleted.

- Each archived file has a `superseded_by` field at its original location pointing to the new document
- Files in `_historical/` are **not used as implementation reference for anything**
- The Boot Matrix has a "Do NOT Treat as Current" column pointing to historical files to avoid

---

## 4. Migration Checklist: Building from Scratch

### Phase 1: Basic Skeleton (Day 1)

```
project_root/
├── CLAUDE.md                    ← Write behavioural principles + first batch of hard rules
└── wiki/
    ├── INDEX.md                 ← Top-level index
    ├── 001_Inbox/
    │   └── INDEX.md
    ├── 002_Architecture/
    │   ├── INDEX.md
    │   └── settled.md           ← Initialise as empty table
    ├── 003_Guides/
    │   └── INDEX.md
    ├── 004_Ops/
    │   └── INDEX.md
    └── 005_AgentMgmt/
        ├── INDEX.md             ← Boot Protocol (Steps 0-5)
        ├── active/
        │   ├── ctx_main/
        │   │   ├── GLOSSARY.md          ← Initial glossary
        │   │   ├── TRUTH_LADDER.md      ← Priority ladder
        │   │   ├── BOOT_MATRIX.md       ← Initial routing table (can be empty)
        │   │   ├── ctx_system_current.md ← Current system state
        │   │   └── readerBoot.md        ← Human-friendly onboarding
        │   └── ctx_safety/
        │       └── README.md            ← Safety rule placeholder
        ├── _historical/
        └── _templates/
```

### Phase 2: Fill In as the Project Evolves

| Event | Where to Write |
|-------|---------------|
| Made an architecture decision | → Add a numbered S entry to settled.md |
| Encountered a bug; fix revealed a cognitive error | → Add a rule to ctx_safety/ (with postmortem) |
| Term confusion caused the agent to err | → Add a disambiguation entry to GLOSSARY.md |
| Completed a subsystem | → Add subsystem folder + INDEX.md to 002_Architecture/ |
| Subsystem needs agent context loading | → Add ctx_[subsystem]/ to 005_AgentMgmt/active/ |
| Boot Matrix needs a new task type | → Add a row to BOOT_MATRIX.md |
| Old design superseded by new design | → Move old doc to _historical/, new doc's `supersedes` points to old |
| Wrote an operation guide | → 003_Guides/ |
| Have a deployment process | → 004_Ops/ |

### Phase 3: Quality Audit

Periodically run a documentation audit:

- [ ] Does every rule in CLAUDE.md still hold? (Cross-check with code)
- [ ] Are there new confusing terms that need to be added to the Glossary?
- [ ] Are there decisions in settled.md that the code has overturned?
- [ ] Do the rules in ctx_safety/ cover all known failure modes?
- [ ] Do the Boot Matrix code anchors still point to the correct files?
- [ ] Does the Truth Ladder need hierarchy adjustments?
- [ ] Do all files in _historical/ have `superseded_by` pointers?

---

## 5. CLAUDE.md + Wiki Division of Labour

| Information Type | CLAUDE.md | Wiki |
|-----------------|-----------|------|
| Agent behavioural principles (Think Before Coding, etc.) | **Yes**, loaded every time | No |
| Safety-critical rules + Postmortem | **Summary version** (brief rule + why) | **Full version** in ctx_safety/ |
| Term definitions | No | **Yes**, in GLOSSARY.md |
| Current system state | No | **Yes**, in ctx_system_current.md |
| Settled architecture decisions | Can include summaries | **Full version** in settled.md |
| SDK/API specific parameter semantics | **Yes**, with failure case | Wiki can add more context |
| Code performance constraints | **Yes** (e.g., render-loop rules) | Can add benchmark data |
| Subsystem design documents | No | **Yes**, in 002_Architecture/ |
| Conflict resolution rules | No | **Yes**, in TRUTH_LADDER.md |
| Work notes / experiment logs | No | **Yes**, in 001_Inbox/ |

**Principle: CLAUDE.md holds "needed every conversation" content; Wiki holds "loaded on demand" content.**

Reasonable overlap between CLAUDE.md and Wiki is normal — the SDK hard rules in CLAUDE.md and the full postmortems in ctx_safety cover the same topic at different levels of detail. CLAUDE.md is "quick reference + behavioural constraint"; ctx_safety is "why we do this + full failure analysis."

---

## 6. Modular Design Philosophy Summary

### 6.1 Single Responsibility

- One topic per file; split when exceeding ~300 lines
- Every directory has an INDEX.md as its table of contents
- Don't mix "what is it" and "how to do it" in the same file

### 6.2 Explicit File Lifecycle

```
draft → active → retired (moved to _historical/)
```

- Don't delete — archive instead, preserving historical decision context
- New documents use the `supersedes` field to point to old documents

### 6.3 Tiered Loading

- **Always Load**: CLAUDE.md + Glossary + Truth Ladder
- **Load on Demand**: Routed by Boot Matrix
- **Never Auto-Load**: Inbox (may be outdated), Historical (superseded)

### 6.4 Fail-Closed Knowledge Management

- Safety rules cannot be overridden by ordinary documents (explicitly elevated in Truth Ladder)
- `do_not_use_for` field prevents documents from being incorrectly applied
- Boot Matrix has a "do not read" column

### 6.5 Dual-Track Design for Humans and AI

- `readerBoot.md`: Human-friendly onboarding (role selector, analogies)
- `ctx_system_current.md`: Structured current state for AI agents
- `GLOSSARY.md`: Disambiguation reference serving both humans and agents

### 6.6 Postmortem-Driven Rule Evolution

The most valuable content in the wiki is not design documents — it's **rules learned from failure**:

```
Fault event → Postmortem analysis → Distill rule (Rule + Why) → Write to ctx_safety/
→ Summary written to CLAUDE.md → Boot Matrix ensures relevant task types must read it
```

This closed loop ensures:
1. Every rule is backed by a real failure
2. Rules are not "top-down dogma" but "hard-won lessons"
3. When the agent understands "why," it can make correct judgements in edge cases

---

## 7. Anti-Pattern Warnings

| Anti-Pattern | Consequence | Correct Approach |
|-------------|-------------|-----------------|
| Stuffing all information into CLAUDE.md | CLAUDE.md too long, agent context window consumed | CLAUDE.md only holds "needed every time" content |
| Writing "don't do X" without a why | Agent bypasses rule in seemingly reasonable situations | Rule + Why + Failure Case — all three together |
| Not disambiguating terms | Agent interprets the same word differently in different conversations | Glossary grouped by confusion scenario |
| No Truth Ladder | Agent randomly chooses which source to trust when docs conflict | Explicit priority ladder |
| Treating work notes as authoritative docs | Outdated plans treated as current design by the agent | Inbox has very low priority in Truth Ladder |
| Deleting old documents | Can't trace "why we designed it that way originally" | Move to _historical/, link with superseded_by |
| Docs only say "what it is," never "what it is NOT" | Agent expands document scope to incorrect domains | Use `do_not_use_for` + "Do NOT Treat as Current" column |
| Safety rules at same level as ordinary docs | Safety rules accidentally overridden by new ctx_* docs | Explicitly elevate ctx_safety priority in Truth Ladder |

---

## 8. INDEX.md and ctx_system_current.md Authoring Templates

### INDEX.md Template (wiki top level)

```markdown
# [Project Name] Wiki

> Living documentation for [brief project description].
> Human-authored knowledge that does NOT belong in code comments.

## Sections

| # | Section | Purpose |
|---|---------|---------|
| 001 | [Inbox](001_Inbox/INDEX.md) | Work notes, experiment logs |
| 002 | [Architecture](002_Architecture/INDEX.md) | Design docs, architecture decisions |
| 003 | [Guides](003_Guides/INDEX.md) | Operation guides |
| 004 | [Ops](004_Ops/INDEX.md) | Deployment, environment config |
| 005 | [AgentMgmt](005_AgentMgmt/INDEX.md) | Agent context management |

## Conventions

- **One topic per file.** Split when exceeding ~300 lines.
- **INDEX.md** in every folder — serves as table of contents.
- **Frontmatter** (YAML `---` block): `title`, `created`, `status`.
- **Link, don't duplicate.** Cross-references use relative paths.
- **Status tags:** `draft` → `active` → `retired`.
```

### ctx_system_current.md Template

```markdown
---
status: active
scope: current-truth
---

# Agent Boot File — [Project Name] (Current State)

## Step 0: Who You Are
[Role definition]

## Step 1: 30-Second Project Overview
### What the System Does
[One-sentence description]

### Current Status
| Feature | Status | Notes |
|---------|--------|-------|
| ... | done/in-progress/planned | ... |

## Directory Structure
[Current code directory tree]

## Threading / Architecture Model
[Threading model or architecture diagram]

## Key Constraints
[Hardware constraints, inviolable rules]

## Recent Decisions
[Recent architecture decisions]
```

---

## 9. Summary: Core Insights of This Wiki Architecture

1. **Context is a scarce resource** — More isn't better; precise loading is what matters. Boot Matrix + on-demand loading is the core mechanism.

2. **Conflicts are inevitable** — As projects evolve, documents will inevitably diverge from code. The Truth Ladder doesn't "prevent conflicts" — it provides "clear resolution rules when conflicts occur."

3. **Safety knowledge needs privilege** — Safety rules must have higher priority than ordinary documents and cannot be accidentally overridden by new design docs.

4. **Failure experience is the most valuable knowledge** — Every ctx_safety rule traces back to a real fault event. Rules without postmortems won't be truly followed by agents.

5. **Terminology is the foundation of collaboration** — A misunderstood term causes more damage than a missing document. The Glossary has extremely high ROI.

6. **Documents must self-describe their validity** — Frontmatter fields like `status`, `supersedes`, and `do_not_use_for` let documents declare "am I still valid?"

7. **Humans and AI need different entry points** — readerBoot.md uses analogies and role selectors for humans; ctx_system_current.md uses structured tables and code anchors for agents. Same knowledge base, two presentation modes.

8. **CLAUDE.md is the constitution, Wiki is the legal code** — CLAUDE.md holds unchanging principles and always-needed rules; Wiki holds detailed, evolvable, on-demand knowledge. Reasonable overlap exists, but their responsibilities differ.
