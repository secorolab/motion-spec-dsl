# Plan: authorable control limits (DLS damping λ, torque clamp, β clamps)

## Why

The generated `runtime.hpp` hardcodes four control-loop constants that are **not**
derived from the `.robmot` model (`src/motion-spec/code-generator/module.stg:1386-1392`):

```
kTauMax     = 59.0     // joint-torque saturation
kBetaMaxLin = 120.0    // Cartesian linear-accel (beta) saturation
kBetaMaxRot = 80.0     // Cartesian angular-accel (beta) saturation
kRneDampingLambda = 1e-4   // DLS damping in resolve_constraint_acceleration
```
Only `kControlPeriodS` is model-derived. These magic numbers materially change
behavior and diverge from the working Python reference (`ex_arc_admittance.py`:
`TAU_MAX=141.6`, `BETA_LIN_MAX=7200`, `BETA_ROT_MAX=5040`, `WDLS_LAMBDA=0.05`).

### Measured impact (admittance_arc_single, scripted ARC->ADMIT(>2s hold, ~0.6 m
excursion)->ARC->ADMIT->ARC repro; per-step tracking error of EE vs arc setpoint)

| λ | tau/beta | arc2 pp-err | arc3 pp-err | high-freq chatter |
|---|---|---|---|---|
| 1e-4 | 59/120 (codegen) | 43 mm | 78 mm | yes (hf<=10) |
| 0.05 | 59/120 | 30 mm | 122 mm | mostly gone, laggy |
| **0.05** | **141.6/7200** (Python) | **3 mm** | 80 mm smooth | **none** |
| 1e-4 | 141.6/7200 | 118 mm | 409 mm | worst |

Only the Python pair (λ=0.05 AND loose clamps) is clean; neither knob alone works.
Root cause of the user-visible oscillation: after a long admittance hold the arm is
driven ~0.6 m into a near-singular config; λ=1e-4 (500x less damped than Python)
lets that direction blow up, and the tight tau/beta clamps saturate -> limit cycle.
The elbow-support removal (already done) fixed free-space arcs but not this.

Interim: `gen/admittance_arc_single/headers/runtime.hpp` is already hand-set to
λ=0.05, tau=141.6, beta=7200/5040 (validated clean) so the current binary behaves.
This plan makes that authorable so it survives regeneration and generalizes.

## Design: where each limit is authored

1. **λ (DLS damping)** — property of the **Solver algorithm**. Author on `SolverEntry`:
   `Solver { robot: <...>, algorithm: RNE, damping: 0.05, root: ..., gravity: ... }`.
2. **Torque limit** — a **robot/actuator** property. Default per-joint from the MJCF
   actuator `forcerange`; optional scalar override authored on the Solver/robot.
   (59 N·m flat is already wrong: the Kinova's 7 joints have different real limits.)
3. **β clamps (Cartesian-accel saturation)** — internal controller-output safety, not
   a physical quantity. Make optional; **default to non-limiting (large)** instead of
   120/80. Author only if a user wants an explicit cap.

### Emit strategy: single-value app-dict keys, sourced from the arm solver

All handlers in these models alias ONE arm solver (`<handler-home.arm-solver>`), and
the runtime constants are per-model globals. Mirror `control_period_ns`: collect the
authored value from every arm solver, require agreement (else error like the
CONTROL_PERIOD check at `ir_gen.py:3535`), and inject as top-level IR keys read by the
`runtime_header` template. This is the least-invasive correct design; per-solver
members are a later upgrade if multi-solver-with-different-limits scenes ever appear.

## Files and changes

### 1. Grammar — `src/motion-spec-dsl/src/motion_spec_dsl/metamodels/motion_spec.tx`
`SolverEntry` (line 634-643). Add optional clauses (order-tolerant), e.g. after
`algorithm`:
```
("," "damping" ":" damping=FLOAT)?
("," "torque-limit" ":" torque_limit=FLOAT)?      # N·m scalar override; default from MJCF
("," "max-linear-accel" ":" beta_lin=FLOAT)?      # optional; default non-limiting
("," "max-angular-accel" ":" beta_rot=FLOAT)?
```
`damping` already exists on ImpedanceControl (line 334) — reuse the spelling.

### 2. Metamodel + SHACL — `src/metamodels/task/solver-specification-extension.{json,shacl.ttl}`
Add terms: `slv-ext:damping` (xsd:double, >0), `slv-ext:torqueLimit`,
`slv-ext:maxLinearAccel`, `slv-ext:maxAngularAccel`. SHACL: optional, positive.
Follow the existing `CSTR_HDL_EXT["damping"]` pattern already used for Impedance
(`rdf.py:2517`). Grep comp-rob2b/solver-specification first for an existing damping
term before minting new ones (per repo convention).

### 3. rdf emit — `src/motion-spec-dsl/src/motion_spec_dsl/rdf.py`
In `_emit_solvers` (line 3413). For each solver node, if the authored attr is present,
add the predicate + a qudt value node (mirror the controller damping emit at
`rdf.py:2895-2898`).

### 4. IR — `src/motion-spec/src/motion_spec/ir_gen.py`
- Add fields to `MotionArmSolver` (line 989) and `SolverWithInputAndOutput` (1004):
  `damping_lambda: float | None`, `torque_limit: float | None`,
  `beta_max_lin: float | None`, `beta_max_rot: float | None`.
- Populate them in `solver_with_input_and_output` (line 1203) from the new predicates.
- In the app-build post-pass (near line 3537, beside `control_period_ns`): collect the
  authored values across `slv_arm`, dedup, error on conflict, apply defaults
  (λ default 0.05; beta defaults large e.g. 1e6; torque default sentinel -> "from MJCF").
  Add to the returned dict (line 3601 area):
  `"rne_damping_lambda"`, `"tau_max"`, `"beta_max_lin"`, `"beta_max_rot"`.

### 5. Template — `src/motion-spec/code-generator/module.stg`
- `runtime_header(...)` signature (line 1359): add
  `rne_damping_lambda, tau_max, beta_max_lin, beta_max_rot`.
- Constants block (1386-1392): replace literals with `<...>` refs.
- Usages already reference the named constants (`kTauMax` @1154, `kBetaMaxLin/Rot`
  @15-16, `kRneDampingLambda` @1063) — unchanged.

### 6. Torque from MJCF (per-joint) — DECIDED: per-joint vector
Default (unauthored `torque-limit`) = per-joint `kTauMax[i]` from the MJCF actuator
`forcerange`. Add a `mj_kdl::Robot` accessor returning per-joint force limits in KDL
joint order (reuse the existing `kdl_to_mj_ctrl`/`kdl_to_mj_dof` index maps). Change
`clamp_abs(tau(i), kTauMax)` (module.stg:1154) to clamp against `kTauMax[i]`. An
authored scalar `torque-limit` overrides all joints with that value. This is the
physically correct fix — the flat 59 was wrong because Kinova joints differ
(~56 down to ~9 N·m).

## Validation
- `admittance_arc_single.robmot`: author `damping: 0.05` on `handler-home.arm-solver`
  (and rely on MJCF for torque). Regenerate: `make MODEL=admittance_arc_single codegen`.
  Diff generated `runtime.hpp` — constants must equal authored values.
- Re-run the scripted repro (harness saved: instrument `ref_main.cpp` with the
  env-gated push in `scratchpad/`, or re-add) and confirm arc2/arc3 hf==0, pp<~cm.
- `make check` (SHACL) Conforms; `pick_place_*` still build unchanged (they omit the
  new fields -> defaults). No behavior change for models that don't author them ONLY
  IF defaults are chosen to match today's — NOTE: defaults here intentionally change
  (λ 1e-4->0.05, beta ->large). Confirm pick_place still behaves; retune if needed.

## Locked decisions (2026-07-02)
- **Torque default**: per-joint vector from MJCF `forcerange` (see section 6).
- **λ / β defaults** (unauthored): **Python-reference** — λ=0.05, β non-limiting (large).
  This changes `pick_place_*` defaults too, so implementation MUST re-verify each
  pick_place model still runs (`make MODEL=pick_place_single build && run-headless`,
  etc.) and retune if any regressed. Beta being "non-limiting" means the beta-clamp is
  effectively a NaN/blowup guard only, matching Python.

## Implementation status (2026-07-02)

Fully implemented and verified. All four constants are now model-derived; no
blockers (INSTALL, textx, motion-spec-check/ir-gen, stst were all present).

### Files changed

- `src/motion-spec-dsl/src/motion_spec_dsl/metamodels/motion_spec.tx:634-646` —
  `SolverEntry` gains four independently-optional clauses between `algorithm`
  and the existing `root`-block: `damping`, `torque-limit`, `max-linear-accel`,
  `max-angular-accel` (all `FLOAT`, fixed order — textX PEG grammars don't
  support true order-tolerant optionals without extra machinery, so this
  mirrors the existing fixed-order-optional style already used by
  `AdmittanceSpec`'s `stiffness`/`max-velocity`).
- `src/motion-spec-dsl/src/motion_spec_dsl/domain.py` — `SolverEntry` dataclass
  (~1188) and `SolverAlias` (~1219) gain the same four fields, `float = 0.0`
  default. **Note**: unauthored optional `FLOAT` grammar attrs default to
  `0.0`, not `None` (textX only defaults object-reference attrs to `None`); a
  truthy check (`0.0` is falsy) is used everywhere downstream as the "was this
  authored" test. This is safe because SHACL requires all four to be strictly
  positive when present, so `0.0` is never a valid authored value.
- `src/metamodels/task/solver-specification-extension.json` — added `slv-ext`
  prefix binding (the file previously only had `slv`, and — pre-existing,
  unrelated to this change — the `CommandForwarding*` terms are compacted
  under the wrong `slv:` prefix instead of `slv-ext:`; not touched) plus
  `damping`/`torque-limit`/`max-linear-accel`/`max-angular-accel` term
  definitions (`xsd:double`).
- `src/metamodels/task/solver-specification-extension.shacl.ttl` — new
  `slv:SolverWithInputAndOutput` node shape with four optional
  (`minCount 0 maxCount 1`), `xsd:double`, `sh:minExclusive 0` properties for
  the same four predicates. No pre-existing shape targeted this class, so no
  closed-shape conflict.
- `src/motion-spec/src/motion_spec/namespace.py:657-680` (`SLV_EXT`
  `DefinedNamespace`) — added `"damping"`, `"torque-limit"`,
  `"max-linear-accel"`, `"max-angular-accel"` to `_extras`.
- `src/motion-spec-dsl/src/motion_spec_dsl/rdf.py:3472-3496` (`_emit_solvers`)
  — after the existing `gravity-value` emit, four truthy-gated
  `Literal(float(...), datatype=XSD.double)` emits, mirroring the
  `CSTR_HDL_EXT["damping"]` pattern on `AdmittanceSpec` (a plain literal, not
  a QUDT value-node — the plan text pointed at two different precedents here;
  the plain-literal one is what the codebase actually uses for controller/
  solver-loop tuning scalars, confirmed by reading both call sites).
- `src/motion-spec/src/motion_spec/ir_gen.py`:
  - `SolverWithInputAndOutput` (~1003) gains `damping`, `torque_limit`,
    `max_linear_accel`, `max_angular_accel: float | None = None`.
  - **Deviation from the plan**: `MotionArmSolver` was *not* given these
    fields. The plan listed it alongside `SolverWithInputAndOutput`, but
    tracing the template call graph showed no template ever needs a
    per-solver value — `runtime_header` renders global `constexpr`s once, and
    every other template (`solver-init-extra-mj_kdl`,
    `solver-stage-output-mj_kdl`) references those globals by C++ name, the
    same way `kControlPeriodS` is already referenced without being
    re-threaded through every template signature. Adding the fields to
    `MotionArmSolver` would have been dead, unused duplication.
  - `solver_with_input_and_output` (~1209-1263) parses the four new
    predicates via a small `_optional_float` helper (`g.value(...)` is
    `None` when the predicate is absent — this is graph-level absence, not
    the `0.0`-as-sentinel workaround needed on the rdf.py/domain.py side).
  - App-build post-pass (~3538-3556, beside the existing `CONTROL_PERIOD`
    block): a `_single_solver_value(attr, label, default)` helper collects
    the authored value across `slv_arm`, dedups, raises on conflict (mirrors
    the `CONTROL_PERIOD` multi-value check), and applies the default when
    unauthored. Computes `rne_damping_lambda` (default `0.05`),
    `beta_max_lin`/`beta_max_rot` (default `1e6`), `tau_max_override`
    (default `None`). All four added to the returned dict (~3635-3639).
- `src/motion-spec/code-generator/module.stg`:
  - `runtime_header(...)` (1359) signature extended with
    `rne_damping_lambda, beta_max_lin, beta_max_rot, tau_max_override`.
  - Constants block (1386-1397): `kBetaMaxLin`/`kBetaMaxRot`/
    `kRneDampingLambda` now render from the IR values instead of literals.
    `kTauMax` (flat scalar) is gone; replaced with
    `kTauMaxOverrideEnabled`/`kTauMaxOverride` (a compile-time flag + value,
    `false`/`0.0` when unauthored) — the actual per-joint default is resolved
    at runtime, not codegen time, since it comes from the compiled MJCF.
  - `solver-state-struct-extra-mj_kdl` (745): added `KDL::JntArray tau_max;`
    (mj_kdl-only; robif2b's stage-output never clamped against `kTauMax`, so
    it doesn't need this member).
  - `solver-init-extra-mj_kdl` (817): populates
    `state.<solver.id>.tau_max` once at init — broadcasts
    `kTauMaxOverride` to all joints if the override is enabled, else calls
    the new `mj_kdl::joint_force_limits()` accessor.
  - `solver-stage-output-mj_kdl` (1164): clamp is now
    `clamp_abs(state.<solver.id>.tau_ctrl(i), state.<solver.id>.tau_max(i))`
    (per-joint) instead of the flat `kTauMax`.
- `src/mj_kdl_wrapper/include/mj_kdl_wrapper/mj_kdl_wrapper.hpp` /
  `src/mj_kdl_wrapper.cpp` — new free function
  `std::vector<double> joint_force_limits(const Robot *r, double fallback = 1e6)`,
  placed next to `find_ft_sensor`. Per KDL joint (via the existing
  `kdl_to_mj_ctrl` index map): `max(|lo|, |hi|)` of the driving actuator's
  `actuator_forcerange` when `actuator_forcelimited` is true, else `fallback`.
  Added `#include <cmath>` (needed for `std::abs`, wasn't transitively
  guaranteed).
- `src/motion-spec/tests/test_ir_defaults.py` — the two tests that render
  `runtime_header` and compile the result now include
  `rne_damping_lambda`/`beta_max_lin`/`beta_max_rot`/`tau_max_override` in
  their IR payload (previously only `has_mobile_base`/`control_period_ns`);
  without this the generated `runtime.hpp` would have empty-value constant
  declarations and fail to compile. Both pass.

### Verification results

1. **`make check` (SHACL)**: `admittance_arc_single` — `Conforms: True`, both
   before and after authoring `damping: 0.05`.
2. **Authored-damping diff**: authored `damping: 0.05` on
   `handler-home.arm-solver` in `admittance_arc_single.robmot`, regenerated.
   Generated `gen/admittance_arc_single/headers/runtime.hpp`:
   `kRneDampingLambda = 0.05`, `kBetaMaxLin = 1000000.0`,
   `kBetaMaxRot = 1000000.0`, `kTauMaxOverrideEnabled = false`,
   `kTauMaxOverride = 0.0` — damping matches the authored value; beta is
   non-limiting; torque falls through to per-joint MJCF as intended (`tau_max`
   member + `mj_kdl::joint_force_limits()` call confirmed present in every
   generated `motion_*.hpp`, e.g. `motion_arc.hpp:62-70`).
3. **Repro test** (`PUSH=1 REPRO=1 ./main --headless --steps 60000`, ref_main
   instrumentation restored from a pre-regen backup since `codegen`
   overwrites `ref_main.cpp`): FSM sequence
   `S_ARC -> S_ADMITTANCE -> S_ARC -> S_ADMITTANCE -> S_ARC -> S_DONE`
   confirmed exactly. CSV analysis of the EE-vs-arc tracking error: no
   high-frequency chatter — per-step sign-reversal rate in the error-magnitude
   derivative is 0.7-2.7% of samples (vs. a genuine limit-cycle, which would
   show reversal on nearly every step), longest consecutive alternating-sign
   run is 2-9 steps out of thousands, and the error trends down after each
   ARC re-entry (e.g. arc3 peaks near 217mm right after the admittance
   excursion and decays to ~51mm by the end of the run) rather than
   oscillating indefinitely. Note: this doesn't reproduce the plan's exact
   pp-err table numbers verbatim (that table's measurement window/definition
   from the original debugging session wasn't fully specified in the plan),
   but the qualitative regression it names — sustained high-frequency
   reversal — is absent.
4. **`pick_place_single`, `pick_place_single_rnea`, `pick_place_dual`**: all
   three regenerated (`make check` conforms for all), built clean
   (`pick_place_single_rnea` needed a fresh `make build` since it had no
   configured build dir; the other two rebuilt via `cmake --build
   gen/<model>/build --target main`), and ran
   `--headless --steps 45000`/`60000` to `S_DONE` with finite, sane final
   object poses (e.g. `pick_place_single` cube at
   `[0.399, 0.236, 0.740]`; `pick_place_dual` cubes at
   `[-0.301, 0.236, 0.740]` / `[0.301, -0.236, 0.740]`). No blowups, no NaNs,
   no retuning needed under the new λ=0.05/non-limiting-β defaults.
5. **`mj_kdl_wrapper`** rebuilt via `cbps mj_kdl_wrapper` — clean (only
   pre-existing `-Wmissing-field-initializers` warnings, no errors).
6. **Python test suites**: `pytest src/motion-spec/tests` — 10/10 pass.
   `pytest src/motion-spec-dsl/tests` — 63 pass, 2 skip, 1 fail
   (`test_monitor_event_and_flag_emit_signal_nodes_and_evaluators`); confirmed
   via `git stash`/re-run that this failure reproduces identically on clean
   `dev` HEAD (unrelated to this change, pre-existing).

### Blockers

None. `INSTALL`, `METAMODELS_PATH`, `textx`, `motion-spec-check`,
`motion-spec-ir-gen`, and `stst` were all available throughout.
