# Plan 002: motion-spec derives the ACHD/RNE solver structure instead of reading it

> **Executor instructions**: Follow this plan step by step. Run every
> verification command and confirm the expected result before moving on. If
> anything in "STOP conditions" occurs, stop and report — do not improvise.
> When done, update the status row in `plans/README.md`.
>
> **This plan modifies the `motion-spec` repo (`../motion-spec` relative to the
> plans directory), not `motion-spec-dsl`.** The plans index lives in
> motion-spec-dsl only because 001/002/003 are one coordinated effort.
>
> **Drift check (run first)**:
> `git -C ../motion-spec diff --stat -- src/motion_spec/ir_gen.py`
> against the SHA in the Status block. On any change to the cited methods,
> re-read them before proceeding.

## Status

- **Priority**: P2
- **Effort**: L
- **Risk**: HIGH (behavior-preserving rewrite of the IR that drives codegen)
- **Depends on**: **Prerequisite P0** below (ir_gen must import). Pairs with plan 003.
- **Category**: tech-debt / architecture
- **Planned at**: motion-spec-dsl `4fa96df`; verify motion-spec HEAD with
  `git -C ../motion-spec rev-parse --short HEAD` before starting and record it here.

## Prerequisite P0 — ir_gen must import (blocker, fix first)

`../motion-spec/src/motion_spec/ir_gen.py:51` imports names removed from
`motion_spec/namespace.py` (`MJ`, and per the DSL audit also `POLY`, `InvertPose`,
`DirectionCoordinateView`, `JointTorque`). **`ir_gen` does not import today**, so no
motion-spec test can run. This plan cannot start until that is fixed. Confirm with:

`python -c "import motion_spec.ir_gen"` → exits 0, no ImportError.

If it raises, STOP and fix the imports (remove dead names / restore them in
`namespace.py`) as a separate first change, or report back.

## Why this matters

Today the DSL bakes ACHD/RNE control-solver internals into the emitted RDF and
`ir_gen` reads them straight into IR. That splits solver knowledge across two repos.
Plan 003 removes the emission; for that to be safe, `ir_gen` must first *derive* the
same structures from what remains in the spec graph — the solver `algorithm` plus each
controller's `constraint` and view. This plan is the derivation half. When it lands,
the IR produced from a *stripped* DSL graph is identical to the IR produced today, and
only then can plan 003 delete the DSL emission.

## Current state — what ir_gen READS today (all in `../motion-spec/src/motion_spec/ir_gen.py`)

- `motion_drivers(id_)` (~759): reads `SLV["acceleration-constraint"]` →
  `SLV["AccelerationConstraintSpecification"]` → `SLV["constraints"]`, calling
  `acceleration_constraint(c)` per member (lines ~767-770).
- `acceleration_constraint(id_)` (~818-833): reads a node's `SLV["subspace"]`,
  `SLV["acceleration-energy"]`, `SLV["AxisAligned"]`, `SLV["axis"]`, optional
  `algo-ext:limits` saturation and `geom-coord:as-seen-by`; returns an
  `AccelerationConstraint` IR dataclass.
- `controller(id_)` (~1009-1051): reads `CSTR_HDL["control-signal"]` (line ~1030) to get
  the controller's output quantity, and matches output/integral saturations against it.
- PoseDiff traversals: `GEOM_OP_EXT["PoseDiffEvaluator"]` at ~444, 893, 2262, 2401-2409,
  2551, 2591 (evaluator skipped in normal evaluator parsing and re-surfaced as MAP views).

`ir_gen` already parses everything the derivation needs from the *reduced* graph:
- the controller's type (`controller()` at ~1011-1013: PID / Impedance / FeedForward),
- its `cstr-hdl:constraint` and the constraint's relation type,
- the view (subspace + axis) via the MAP-view parsing (`~1659-1702`),
- the solver `algorithm` (`solver_with_input_and_output()` ~745-757, field `algorithm`),
- the target frame (`geom-rel:with-respect-to` / `as-seen-by`).

## The logic to port — DSL `controller_semantics.py`

The axis enumeration that must move lives in
`../motion-spec-dsl/src/motion_spec_dsl/controller_semantics.py` (read it in full;
~277 lines). The load-bearing pieces:

- `LINEAR_ACCELERATION_AXES` / `ANGULAR_ACCELERATION_AXES` / `POSE_ACCELERATION_AXES`
  (lines 117-132) — the axis tuples.
- `infer_command_type(subspace)` (147-165) — view subspace → command QuantityType.
- `constraint_view_subspace(constraint)` (168-198) — canonical `distance` /
  `joint-position` / `pose` / `position` / `orientation` subspace.
- `controller_command_record(controller)` (211-277) — the core: resolves command type
  (impedance ⇒ Force) and the acceleration-constraint axis set — whole-pose 6D, a single
  axis, or one `distance` direction-aligned constraint.
- `pose_diff_components(...)` (76-85) — ACHD axis records → neutral linear/angular pose-diff
  components.

**Do not copy it verbatim** — it operates on the DSL AST (`ControllerEntry`,
`ConstraintSpecification`, `SubSpace` enums). In motion-spec you re-express the same
decision table against the RDF/IR facts ir_gen already has (view subspace string, axis,
quantity kind, constraint relation type). The decision table itself — which axes a
(subspace, axis, command-type, relation) tuple expands to — is what must be preserved
exactly; the audit's whole point is that this table is derivable, so it belongs here.

## Commands you will need

| Purpose | Command | Expected |
|---|---|---|
| Activate venv | `source /home/batsy/work/ms/.venv/bin/activate` | prompt changes |
| Import check | `python -c "import motion_spec.ir_gen"` | exit 0 |
| motion-spec tests | `cd ../motion-spec && METAMODELS_PATH=/home/batsy/work/ms/src/metamodels PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q` | pass (record baseline first) |
| Lint | `ruff check ../motion-spec/src/` | clean |

Record the motion-spec test baseline (how many pass today, after P0) before changing anything.

## Scope

**In scope (motion-spec repo):**
- `../motion-spec/src/motion_spec/derive_solver.py` (create — the derivation)
- `../motion-spec/src/motion_spec/ir_gen.py` (reroute read-sites to derivation)
- `../motion-spec/tests/` (add IR-equivalence test)

**Out of scope:**
- `motion-spec-dsl` — plan 003 handles the DSL strip. Do NOT edit the DSL here. This
  plan must leave the DSL emitting the full structure, and must produce identical IR
  whether or not the ACHD nodes are present in the graph (so it works both before and
  after plan 003).
- Codegen templates (`code-generator/*.stg`) — the IR shape must be unchanged, so
  templates need no edits. If you find yourself wanting to change a template, STOP:
  the derivation is producing a different IR than the reader did.

## Steps

### Step 1: Port the axis-enumeration decision table

Create `derive_solver.py` with a pure function that, given (controller_type,
view_subspace, view_axis, command_type, constraint_relation, quantity_kind), returns the
ordered list of acceleration-constraint axis records `(subspace, axis)` — matching
`controller_command_record.acceleration_constraints` exactly, including: whole-pose
equality ⇒ 6D (`POSE_ACCELERATION_AXES` order), position/linvel single-axis vs 3D,
orientation/angvel single-axis vs 3D, `distance` ⇒ one `("linear-acceleration",
"distance")`, and force/posture-torque ⇒ empty (handled by the deferred force paths).
No RDF here — plain inputs/outputs, unit-testable.

**Verify**: `python -c "import motion_spec.derive_solver"` → exit 0.

### Step 2: Derive the acceleration constraints + energies in ir_gen

Rewrite `motion_drivers()` and `acceleration_constraint()` so that when the solver
`algorithm` is ACHD or RNE, the `AccelerationConstraint` / `AccelerationConstraintSpecification`
IR objects are *built from* Step 1's axis records + the controller's constraint/view/frame,
instead of read from `SLV["acceleration-constraint"]` edges. Synthesize the
acceleration-energy quantity IR per axis (kind `AccelerationEnergy`, unit `N-M2-PER-SEC2`),
mirroring what `builder._decode_control_signal` / `_emit_solver_interfaces` produce today.
Derive the acceleration saturations from the solver's `slv:limits` (already parsed).

Keep a compatibility path: if the explicit `SLV["acceleration-constraint"]` nodes ARE
present (pre-plan-003 graph), the derived result must equal the read result — assert this
internally behind a debug flag if helpful.

**Verify**: motion-spec test suite still passes at the recorded baseline (reading a full
graph now goes through derivation and matches).

### Step 3: Derive the controller control-signal

Rewrite `controller()` so `control_signal` is the derived per-controller output quantity
(the acceleration-energy node from Step 2, or the feed-forward/force node) rather than
`self.g.value(id_, CSTR_HDL["control-signal"])`. Preserve the output/integral saturation
matching (which currently keys off the control-signal node identity).

**Verify**: motion-spec tests pass at baseline.

### Step 4: Derive the pose-diff expansion

Replace the `PoseDiffEvaluator` traversals (444, 893, 2262, 2401-2409, 2551, 2591) with
derivation: for a whole-pose equality on an ACHD/RNE solver, generate the per-axis
component controllers, their measured-derivative views, and `PoseDifference` component
error terms from Step 1's `pose_diff_components`, producing the same IR the evaluator
traversal produced. This is the subtlest step — do it last and lean on the equivalence
test.

**Verify**: motion-spec tests pass at baseline.

### Step 5: IR-equivalence gate (the real proof)

Add `../motion-spec/tests/test_ir_equivalence_derived.py`: generate `pick_place_single`
(and, if present, a dual-arm example) with the DSL, build IR two ways —
  (a) from the full graph as emitted today, and
  (b) from a copy of the graph with the five structures stripped (delete triples for
      `SLV:AccelerationConstraint`, `AccelerationConstraintSpecification`,
      `CSTR_HDL:control-signal`, `GEOM_OP_EXT:PoseDiffEvaluator`, `SLV:acceleration-energy`) —
and assert the two IRs are equal modulo blank-node / generated-id renaming (compare the
normalized IR dataclass fields, not raw node ids). This test is the gate plan 003 checks.

**Verify**: `pytest -q tests/test_ir_equivalence_derived.py` → passes; both IR builds
equal.

## Test plan

- Unit test the Step 1 decision table (`tests/test_derive_solver.py`): one case per branch
  of `controller_command_record` (whole-pose 6D, position x-axis, position 3D, orientation
  single/3D, distance, force ⇒ empty, posture-torque ⇒ empty). Assert exact axis lists.
- IR-equivalence test from Step 5 is the integration gate.
- Existing motion-spec tests must stay green throughout (each step ends green).

## Done criteria

- [ ] `python -c "import motion_spec.ir_gen"` exits 0 (P0 satisfied)
- [ ] `derive_solver.py` exists; `tests/test_derive_solver.py` covers every axis-enumeration branch
- [ ] `ir_gen` no longer reads `SLV["acceleration-constraint"]`, `SLV["acceleration-energy"]`,
      `CSTR_HDL["control-signal"]`, or `GEOM_OP_EXT["PoseDiffEvaluator"]` as required inputs
      (`grep -n` shows they are only tolerated, not depended on)
- [ ] `tests/test_ir_equivalence_derived.py` passes (stripped-graph IR == full-graph IR)
- [ ] Full motion-spec suite ≥ recorded baseline
- [ ] Codegen templates unchanged (`git -C ../motion-spec status` shows no `.stg` edits)
- [ ] `plans/README.md` status row updated

## STOP conditions

- P0 unmet: `import motion_spec.ir_gen` still raises. Fix imports first or report.
- The IR-equivalence test cannot be made to pass — the derivation is NOT behavior-
  preserving; report the diff rather than adjusting codegen to hide it.
- You need to edit a `.stg` codegen template — means the IR shape drifted; STOP.
- A derived axis order differs from the read order (ACHD is order-sensitive) — STOP and
  reconcile against `controller_semantics.POSE_ACCELERATION_AXES` ordering.

## Maintenance notes

- After this, ACHD/RNE structure has exactly one source of truth: `derive_solver.py`.
  A new solver algorithm = a new branch here, no DSL change.
- The axis-order sensitivity (linear x/y/z then angular x/y/z) is load-bearing for ACHD;
  call it out in the derivation with a comment and a test.
- Plan 003 (DSL strip) must not merge until this plan's equivalence gate is green on
  every example model, not just `pick_place_single`.
