# motion-spec-dsl — production-readiness audit

Date: 2026-07-19. Scope: motion-spec-dsl only (downstream motion-spec ignored).
Method: read grammars + builder + classes + validation; ran DSL generation on
`pick_place_single.robmot` capturing ClosedNamespace warnings; ran DSL tests +
ruff; diffed emitted vocabulary against metamodels PR #54 (`src/metamodels`,
branch `add-motion-spec-metamodels`) and `src/comp-rob2b/metamodels`.

## Verdict: NOT production-ready. Functionally works; three classes of issue.

Generation succeeds; `test_scoping` + `test_shacl_conformance` = 10/10; ruff clean;
no TODO/FIXME/broad-except. But: 2 undefined-vocabulary emissions, heavy
solver-algorithm derivation that violates the project's own layering principle,
and structural anti-patterns. SHACL passing is *not* a conformance proof — the
undefined terms are unconstrained extras under open-world semantics.

## A. Metamodel conformance — 2 emitted-but-undefined terms — DONE (PR #54 c707a0e)

Both terms now defined + SHACL-enforced; DSL generation emits 0 undefined-namespace
warnings (was 2); `test_scoping`+`test_shacl_conformance` 10/10.

1. **`cstr-hdl-ext:control-mode`** — added to `task/constraint-handler-extension.json`
   + a shape requiring exactly one value from `{ kc-stat:JointForce }`. Modeled as a
   **secorolab extension** term, not base `cstr-hdl:` — comp-rob2b's base
   constraint-handler has no control-mode and is third-party upstream. Consequential
   (not-in-this-PR) edits: `builder.py:2744` emits `CSTR_HDL_EXT`; motion-spec
   `namespace.py`/`check.py`/`ir_gen.py` read the ext predicate. Downstream fixtures
   `models/04-sc*/05-constraint-handler.json` still write base `cstr-hdl:control-mode`
   — deferred (downstream, out of scope).
2. **`geom-coord:has-coordinate`** — added to `geometry/coordinates.json` (set-valued
   `@type @id`). Kept the entity→coordinate direction the DSL already emits (rather
   than switching to the `of-*` inverse) per decision to define, not refactor.

## B. Unnecessary derivations — the biggest issue

The DSL bakes solver-algorithm math into the *specification* RDF, duplicating
knowledge the `algorithm` field already carries downstream.

- `_decode_control_signal` (builder.py:3161–3212) branches on
  `algorithm in {"ACHD","RNE"}` and emits ad-hoc `eacc-*` **acceleration-energy**
  quantity nodes with hard-coded QUDT units.
- Pose-equality fans out into per-axis **pose-diff component controllers +
  evaluators + acceleration-energy control signals** (`_emit_pose_diff_component_
  controllers`, `_emit_pose_diff_evaluator`, ~6× expansion of one constraint).
- The solver node already emits its `algorithm` (ACHD/RNE/…), so **motion-spec has
  everything to derive all of the above itself**. This is the exact pattern the
  project already fixed once: see the `ir_gen backend-leakage refactor` — backend/
  solver specifics belong downstream, not in authored-spec RDF. Cross-frame
  transform derivation was *already* correctly moved out of the DSL; this is the
  same move, not yet done for solver expansion.
- Extra irony: the base `slv:` metamodel **already defines** the canonical
  structured form — `AccelerationConstraintSpecification`, `acceleration-constraint`,
  `acceleration-energy`, `JointForceSpecification`. The DSL bypasses it for ad-hoc
  `eacc-*`/`tau-*` quantity nodes. So the derivation is both misplaced *and*
  non-canonical.

Recommendation: DSL emits the spec contract only — constraint + controller (gains,
view, target frame, saturations) + solver(algorithm). Drop acceleration-energy,
pose-diff, and per-axis expansion; motion-spec derives them from
`algorithm` + `constraint`. Keep `error-signal`/`evaluators` (base defines them).

## C. Anti-patterns (code / structure / style)

1. **God methods.** `_emit_constraint_handler` = 402 lines, `_emit_solver_
   interfaces` = 222, `_emit_context_quantities` = 134. Untestable, single-
   responsibility gone.
2. **Duck-typed dispatch.** builder.py has 73 `getattr(` + 82 `isinstance(` +
   `hasattr(ctrl_item,"ref")` — runtime type-switching on textx objects instead of
   polymorphic methods on the `classes/` hierarchy (which exists but is anemic;
   behavior lives in the builder).
3. **Speculative generality (YAGNI).** `HandlerControlMode` StrEnum + grammar rule
   + validation for a *single* value (`joint-force`).
   `SUPPORTED_CONTROL_MODES_BY_SOLVER_ALGORITHM` maps all 3 algorithms to the
   identical `{JointForce}` — a table carrying zero information.
4. **Concept placement.** `control-mode` is modeled as a handler predicate pointing
   at a `kc-stat` state kind; "commanded quantity space" is arguably a solver /
   motion-driver concern already expressible as `slv:joint-force` /
   `JointForceSpecification`. Decide new-term-vs-reuse before minting the term.

## D. Healthy

- Grammar modularization (`grammars/*.tx`) clean; scoping solid; provenance emitted;
  manifest portable (no absolute paths — `test_generated_manifest_is_portable`).
- DSL-only tests green, ruff green.

## Blocking checklist for prod
- [x] Add `control-mode` term + shape to PR #54 (A1) — done, PR #54 c707a0e.
- [x] Add `has-coordinate` term to PR #54 (A2) — done, PR #54 c707a0e.
- [ ] Strip solver-algorithm derivation from the builder; push to motion-spec (B). **NEXT**
- [ ] Collapse single-value control-mode machinery (C3).
- [ ] Cross-repo `test_motion_spec_graph.py` is RED — motion-spec `ir_gen` imports
      `MJ` (removed from its namespace). Downstream, out of scope, but it means the
      DSL's only integration test currently cannot run.
