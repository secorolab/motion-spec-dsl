# Plan 003: Strip the ACHD/RNE solver derivation out of the DSL builder

> **Executor instructions**: Follow this plan step by step. Run every
> verification command and confirm the expected result before moving on. If
> anything in "STOP conditions" occurs, stop and report — do not improvise.
> When done, update the status row in `plans/README.md`.
>
> **Drift check (run first)**:
> `git diff --stat 4fa96df..HEAD -- src/motion_spec_dsl/rdf/builder.py src/motion_spec_dsl/controller_semantics.py src/motion_spec_dsl/rdf/_helpers.py`
> On any change to the cited methods, re-read them before editing. Line numbers
> are from `4fa96df` + the small uncommitted control-mode edit — re-locate by
> symbol name, never trust the absolute line number.

## Status

- **Priority**: P2
- **Effort**: M
- **Risk**: HIGH (removes emission the pipeline consumes; only safe once 002 lands)
- **Depends on**: **plan 002 (hard gate)** and the existing characterization test.
- **Category**: tech-debt / architecture
- **Planned at**: commit `4fa96df`, 2026-07-19

## HARD GATE — do not start without this

Plan 002 makes motion-spec `ir_gen` *derive* the ACHD/RNE structure instead of reading
it, proven by `../motion-spec/tests/test_ir_equivalence_derived.py`. **If that test is
not green on every example model, STOP.** Stripping the DSL emission before 002 lands
breaks the whole pipeline: `ir_gen` would find zero acceleration constraints, emit empty
solvers, and codegen would produce a non-functional controller.

Confirm the gate:
`cd ../motion-spec && METAMODELS_PATH=/home/batsy/work/ms/src/metamodels PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_ir_equivalence_derived.py`
→ passes. If it does not exist or fails, this plan is BLOCKED.

## Why this matters

Once motion-spec derives the solver structure (plan 002), the DSL emitting it is pure
duplication — solver knowledge on both sides of the boundary. Removing it makes the DSL
output a clean, solver-agnostic spec: adding a new solver algorithm no longer touches
the DSL. This is the payoff of finding B.

## Current state — what to remove (all in `src/motion_spec_dsl/rdf/builder.py`, class `MotionSpecDatasetBuilder`)

Re-locate each by symbol name:

- `_emit_acceleration_energy_quantity` (~2538) — types a node as acceleration-energy.
- `_emit_pose_diff_evaluator` (~2564) — emits `GEOM_OP_EXT:PoseDiffEvaluator` + views.
- `_emit_pose_diff_component_controllers` (~2654) — per-axis component controllers.
- `_emit_pose_diff_measured_derivative` (~2688) — pose-diff component derivative.
- `_decode_control_signal` (~3128) — ACHD/RNE `eacc-*` acceleration-energy synthesis
  (the `algorithm in {"ACHD","RNE"}` branches + trailing fallback).
- `_emit_solver_interfaces` (~3336) — the two `if solver.algorithm in {"ACHD","RNE"}`
  blocks (~3413-3470) that emit per-axis `SLV.AccelerationConstraint` / `AxisAligned` /
  `subspace` / `axis` / `acceleration-energy`, the local `emit_acceleration_saturation`,
  and the `AccelerationConstraintSpecification` aggregation (~3532-3556).
- In `_emit_constraint_handler` (~2726): the pose-diff branch (the `qty.type == Pose …
  isinstance(spec.expr, EqualityConstraint) …` block ~2786-2837 that calls
  `_emit_pose_diff_evaluator` / `_emit_pose_diff_component_controllers` then `continue`s),
  and the `control_signal_node = self._decode_control_signal(...)` emission (~2860-2883)
  including the `CSTR_HDL["control-signal"]` triple.

Shared helpers to remove **after** confirming zero live references (grep the whole
`src/` including `validation/`): in `controller_semantics.py` — `pose_diff_components`,
`PoseDiffComponentRecord`, and the `_pose_diff_*` id builders in `rdf/_helpers.py`
(`_pose_diff_energy_id`, `_pose_diff_error_id`, `_pose_diff_controller_id`,
`_pose_diff_measured_derivative_id`) and `_emit_acceleration_energy_quantity`'s helpers.

### Boundary — KEEP vs REMOVE

**KEEP (authored, solver-agnostic):** constraint specs; controllers (type, gains, the
`cstr-hdl:constraint` link, view, `apply at` frame, output/integral saturations,
`cstr-hdl-ext:reference-signal`); `cstr-hdl:controllers`/`evaluators`/`error-signal`;
monitors; `cstr-hdl-ext:control-mode`; solver declaration (`slv:solver` algorithm,
`agn:of-agent`, `slv:gravity`, `slv:limits`, `slv:motion-drivers`); CommandForwarding
`SLV.output`.

**REMOVE (now derived by motion-spec, plan 002):** `control-signal` → acceleration-energy;
per-axis `slv:AccelerationConstraint`(+Spec); pose-diff expansion; acceleration saturations
wrapping acceleration-energy nodes.

**DEFER (do NOT touch — separate future finding):** `SLV.CartesianForceSpecification`,
`SLV.JointForceSpecification`, force-command wrench construction (`_force_control_signal_node`,
`_emit_force_command_wrench`), and `tau-*` posture nodes. These are output specs on the
spec/solver boundary; leaving them bounds this plan's blast radius. If they turn out to be
entangled with the acceleration-energy code, STOP.

## Commands you will need

| Purpose | Command | Expected |
|---|---|---|
| Activate venv | `source /home/batsy/work/ms/.venv/bin/activate` | prompt changes |
| Count structures | Step 1 snippet in plan 001 | all five → 0 after strip |
| Characterization test | `METAMODELS_PATH=/home/batsy/work/ms/src/metamodels PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_solver_derivation_contract.py` | `1 passed` (after inverting) |
| Safe subset | `METAMODELS_PATH=/home/batsy/work/ms/src/metamodels PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_scoping.py tests/test_shacl_conformance.py` | `10 passed` |
| Lint | `ruff check src/` | `All checks passed!` |

## Scope

**In scope:** `src/motion_spec_dsl/rdf/builder.py`, `src/motion_spec_dsl/controller_semantics.py`,
`src/motion_spec_dsl/rdf/_helpers.py`, `tests/test_solver_derivation_contract.py`, `plans/README.md`.
**Out of scope:** `../motion-spec/*` (plan 002), `src/metamodels/*`, the DEFERRED force/joint-force paths.

## Steps

### Step 1: Remove the acceleration-energy control-signal synthesis
Delete the `algorithm in {"ACHD","RNE"}` branches and the fallback `eacc-*` emission in
`_decode_control_signal`; then delete the `control_signal_node = self._decode_control_signal(...)`
block and its `CSTR_HDL["control-signal"]` triple in `_emit_constraint_handler`. If
`_decode_control_signal` has no remaining non-deferred callers, delete the method.
**Verify**: count snippet → `control-signal: 0`, `acceleration-energy: 0`; `ruff check src/` clean.

### Step 2: Remove per-axis acceleration constraints
Delete the two `if solver.algorithm in {"ACHD","RNE"}` blocks, the `acc_constraint_nodes`
aggregation, the `AccelerationConstraintSpecification` emission, and the local
`emit_acceleration_saturation` in `_emit_solver_interfaces`. Keep solver typing, algorithm,
gravity, agent, motion-drivers, CommandForwarding output, DEFERRED force/joint-force paths,
velocity/force distribution solvers.
**Verify**: count snippet → `AccelerationConstraint: 0`, `AccelerationConstraintSpecification: 0`.

### Step 3: Remove the pose-diff expansion
Delete `_emit_pose_diff_evaluator`, `_emit_pose_diff_component_controllers`,
`_emit_pose_diff_measured_derivative`, `_emit_acceleration_energy_quantity`, and the pose-diff
branch in `_emit_constraint_handler`. Then remove orphaned helpers in
`controller_semantics.py` / `rdf/_helpers.py` **only after** `grep -rn "pose_diff\|PoseDiff\|
_emit_acceleration_energy" src/` shows no live references.
**Verify**: count snippet → `PoseDiffEvaluator: 0`; `grep -rn "PoseDiffEvaluator\|acceleration-energy\|AccelerationConstraint" src/motion_spec_dsl/rdf/builder.py` → no matches; `ruff check src/` clean.

### Step 4: Invert the characterization test
Change `tests/test_solver_derivation_contract.py` to assert all five counts are `0`, and
update its docstring to note the derivation now lives in motion-spec (plan 002).
**Verify**: characterization test `1 passed`; safe subset `10 passed`.

### Step 5: End-to-end re-check against motion-spec
Re-run plan 002's equivalence gate against the **stripped** DSL output to confirm the IR is
unchanged: `cd ../motion-spec && ... pytest -q tests/test_ir_equivalence_derived.py` → passes.
**Verify**: gate green with the stripped graph.

## Test plan

- Invert `tests/test_solver_derivation_contract.py` (0 for all five structures).
- `tests/test_scoping.py` + `tests/test_shacl_conformance.py` stay green.
- The behavior-preservation proof is motion-spec's equivalence gate (Step 5), not a DSL test.

## Done criteria

- [ ] `ruff check src/` exits 0
- [ ] Characterization test asserts 0 for all five structures and passes
- [ ] `grep -rn "PoseDiffEvaluator\|acceleration-energy\|AccelerationConstraint" src/motion_spec_dsl/rdf/builder.py` → no matches
- [ ] Generation of `pick_place_single` emits 0 undefined-namespace warnings
- [ ] Plan 002 equivalence gate green against the stripped graph (Step 5)
- [ ] No out-of-scope files modified (`git status`); DEFERRED force/joint-force paths untouched
- [ ] `plans/README.md` status row updated

## STOP conditions

- Plan 002's equivalence gate is missing or red — this plan is BLOCKED (hard gate).
- "Current state" excerpts don't match live code (drift).
- Removing a helper in Step 3 turns up a live reference in `validation/`.
- The DEFERRED force/joint-force paths share code with the acceleration-energy removal.
- A verification fails twice after a reasonable fix attempt.

## Maintenance notes

- After this lands, reject any new `solver.algorithm in {...}` branch in `builder.py` in
  review — solver-specific structure now belongs only in motion-spec `derive_solver.py`.
- The DSL graph is now solver-agnostic; the `slv:AccelerationConstraint*` vocabulary no
  longer appears in DSL output at all (it is IR-internal to motion-spec).
