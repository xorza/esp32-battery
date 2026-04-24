---
name: design-review
description: Performs a design review of a specified module and writes findings to a markdown file. Questions the module's shape — state placement, data structures, control flow, API contracts — and proposes concrete alternatives. Use when the user asks to review the design, architecture, or approach of a module, or says "rethink", "from scratch", or "better approach". Skip in favor of improve-module when the design is sound and the user only wants code-quality polish.
---

# Design review

Rethink a module from scratch. The goal is to question the current shape — not to polish the code.

**Use for:** state placement, data structures, control flow, API contracts, error-prone invariants.
**Skip — use `/improve-module`:** when the design is sound and you want to dedupe, tighten, or modernize.

Usage: `/design-review <path>`. If no path is given, ask the user.

## Before starting

- **Confirm scope if ambiguous.** If the target is a directory with >5 files or >2000 LOC, ask the user to narrow the scope or use parallel subagents per submodule.
- **MUST read direct callers.** At least 3 call sites per public item in the module, before writing any finding. "Leaky abstraction" claims without call-site evidence are unfounded.
- **MUST NOT make code changes.** This is a read-only analysis.

## Process

1. **Summarize the current design** in 1–3 paragraphs — what state is cached vs derived, what flows through a typical operation, what invariants are maintained and where. Describe load-bearing decisions, not code structure a reader can see at a glance.

2. **Rethink from scratch.** Pretend you're building this today with full knowledge of the requirements but no attachment to existing code. Ground alternatives in named patterns — lazy vs eager cache, push vs pull, state machine vs flags, owned vs borrowed, compute-on-demand vs stored. Vague "this feels wrong" isn't useful; naming a concrete alternative is.

3. **Hunt these smells** (a checklist to consult, not quotas to fill):
   - **State that shouldn't exist** — caches recomputable on demand, cross-frame state that could be local, mutable fields written only once
   - **Data structures fighting the access pattern** — Vec always searched by key, HashMap whose iteration order matters, enum whose variants are always handled the same way
   - **Non-linear control flow** — data bouncing between modules, subtle ordering requirements, two-pass algorithms that could be one
   - **Error-prone contracts** — "must call X before Y", load-bearing comments, silent fallbacks, `Option`/`Result` always unwrapped, public fields with non-local invariants
   - **Leaky abstractions** — types forcing callers to understand internals, hidden coupling, load-bearing derives
   - **Missing named types** — primitives/tuples doing the job of a concept, stringly-typed keys, booleans that should be enums
   - **Premature generalization** — a generic with one concrete type, a trait with one impl, a helper called once
   - **Inverted responsibility** — callers doing work that belongs in the module, or vice-versa

4. **Propose, don't just critique.** Each finding includes at least one concrete alternative. If the current design is right after consideration, that's a valid finding too.

5. **Score.**
   - **Impact 1–5:** 1 = speculative; 3 = clearly worth doing; 5 = removes a class of errors.
   - **Effort 1–5:** 1 = rename; 3 = refactor within the module; 5 = cross-module redesign.

6. **Quality over quantity.** One sharp finding beats five generic ones. Zero findings is a valid outcome. Do not shoehorn findings into every smell category.

## Output

Write to `DESIGN_REVIEW_<module>.md` next to the target (filename derived from the module path — e.g. `DESIGN_REVIEW_graph_layout.md` for a file, `DESIGN_REVIEW_gui.md` for a directory). If the file exists, append a new dated section rather than overwriting.

### Template

```markdown
# Design review: <module path>  (<date>)

## Current design

1–3 paragraphs. Decision summary, not code summary.

## Overall take

One paragraph: is the core approach right?

## Findings

### [F1] <Title>
- **Category**: State / Data structures / Control flow / Contract / Abstraction / Types / Generalization / Responsibility
- **Impact**: N/5 — why
- **Effort**: N/5 — why
- **Current**: What exists now, with `file.rs:line` refs.
- **Problem**: Why this shape is wrong for the job.
- **Alternative(s)**: Concrete proposed design. Multiple if viable, with tradeoffs.
- **Recommendation**: Do it / Don't do it / Depends on <Z>.

(Sort by Impact descending within each Effort tier.)

## Rethink

(Include only if the module's approach is substantially wrong. Describe a coherent alternative — new type layout, new API, new data flow.)

## Considered and rejected

(Optional — omit if empty. Alternatives you thought about but don't recommend.)
```

## Example finding (tone anchor)

```markdown
### [F1] `GraphLayout` caches per-node geometry unnecessarily

- **Category**: State
- **Impact**: 4/5 — removes stale-cache bugs and a dual-update pass during drags
- **Effort**: 3/5 — ~5 call sites across node_ui, connection_ui, pan_zoom
- **Current**: `GraphLayout` stores `KeyIndexVec<NodeId, NodeLayout>`, rebuilt in `update()` each frame (graph_layout.rs:42). Every dragged frame recomputes the dragged node's entry twice — once in `update()`, once in `handle_node_drag` (node_ui.rs:262) — because interaction needs a rect that reflects the just-accumulated drag delta.
- **Problem**: The cache is cross-frame state readers can't reason about locally. Every reader must trust `update()` has run this frame. The double-pass is an ordering workaround requiring a comment to explain (node_ui.rs:255–259).
- **Alternative**: Cache only `NodeGalleys` (the actually-expensive shaped text) and expose `node_layout(gui, ctx, node_id, drag_offset) -> NodeLayout` computed on demand. Pure function; no ordering trap. Cost: ~N computes per frame instead of N+1 — hundreds of nanoseconds per node.
- **Recommendation**: Do it. The argument for caching layouts was a perceived cost that doesn't materialize.
```

Anchors: named file:line refs, a specific alternative design, a quantified cost for the alternative, a concrete recommendation.

## Guidelines

- Write for a reader who knows the codebase but not this module.
- Prefer concrete alternatives over abstract complaints. "Use a HashMap" beats "the data structure is wrong."
- Reference existing reviews for tone — e.g. `prism/LAYOUT_REVIEW.md`.
