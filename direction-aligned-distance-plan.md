# Plan: Direction-aligned acceleration constraints for distance control

## Goal
Let a distance constraint be used as a **control** constraint driven by the ACHD solver, e.g.
`move-forward: keeping distance between <A> and <B> equal to <target>`. Realize it as a
**direction-aligned** ACHD acceleration constraint — the solver-consistent parallel to how a
`position.x` constraint becomes an `slv:AxisAligned` acceleration constraint.

**Key insight (from the user):** ACHD (Vereshchagin) takes each constraint as a *column* of the
`f_cstr` Jacobian. An axis-aligned x-constraint is the unit column `[1,0,0,0,0,0]`; a
direction-aligned constraint is just the **unit column** `[dx,dy,dz,0,0,0]` with the runtime unit
direction. So **no mj_kdl_wrapper change is needed** — only the generated code fills the linear
sub-column with the direction instead of a single `1.0`.

## Existing machinery to REUSE (do not reinvent)
- `rdf.py::_emit_force_command_wrench` (~1689): already handles a `distance`-subspace Pose by
  emitting `GEOM_OP.PoseToDirection` (pose → runtime unit `geom:Direction`) via
  `_emit_direction_coordinate`. **Reuse this exact PoseToDirection + direction-coordinate pattern**
  to get the runtime A→B unit direction from the distance relative pose.
- `rdf.py` ~4560-4571: the `slv:AxisAligned` acceleration-constraint emission (subspace, axis,
  acceleration-energy, as-seen-by). The DirectionAligned emission is the same minus `slv:axis`,
  plus `slv-ext:direction`.
- Acceleration-energy PID machinery (`_emit_acceleration_energy_quantity`, the eacc-* nodes) — the
  distance error (`pose-A-B.distance` from `PoseToLinearDistance` minus the target) drives the same
  acceleration-energy setpoint, unchanged.
- `solver.stg::constraint-force` / `solver-assign-f-cstr` (line 39-45): builds the `f_cstr` column.

## Layer-by-layer changes

### 1. Metamodel — secorolab extension ONLY (do NOT touch comp-rob2b)
`src/metamodels/task/solver-specification-extension.shacl.ttl`: add
```
slv-ext:DirectionAligned a rdfs:Class, sh:NodeShape ;
    sh:property [ sh:path slv:subspace ; sh:minCount 1 ; sh:maxCount 1 ; sh:nodeKind sh:IRI ;
                  sh:in ( slv:angular-acceleration slv:linear-acceleration ) ] ;
    sh:property [ sh:path slv-ext:direction ; sh:minCount 1 ; sh:maxCount 1 ;
                  sh:nodeKind sh:BlankNodeOrIRI ; sh:class geom-rel:Direction ] .
```
`slv:AccelerationConstraint` already requires `slv:acceleration-energy`; DirectionAligned is a
sibling of `slv:AxisAligned` and also carries it. Add `slv-ext:direction` to the `slv-ext`
namespace. Register the new predicate/prefix if `namespace.py`/graph bindings need it.

### 2. Emitter — `src/motion_spec_dsl/rdf.py`
When a **distance** control constraint targets an ACHD/RNE solver (the `_view_subspace(spec) ==
"distance"` case, mirroring the AxisAligned block at ~4552):
- Emit the two endpoint origin **Points** via `_frame_origin_point(...)` for the distance's `A` and
  `B` frames (this is the point-to-point structure — a distance is between points; do it whenever a
  distance constraint is authored, per grammar, not only when a Position references the frame).
- Emit the runtime **Direction** from the distance relative pose using the existing
  `PoseToDirection` + `_emit_direction_coordinate` pattern (as-seen-by the control frame).
- Emit an acceleration-constraint node typed `SLV.AccelerationConstraint` + `SLV_EXT.DirectionAligned`
  with `SLV.subspace = slv:linear-acceleration`, `SLV_EXT.direction = <direction node>`,
  `SLV["acceleration-energy"] = <energy node>`, `GEOM_COORD["as-seen-by"] = <frame>`.
- The acceleration-energy is the PID on the distance error (reuse the eacc machinery; the measured
  quantity is `pose-A-B.distance`, the reference is the `equal to <target>`).

### 3. Validation / command inference
- `controller_semantics.infer_command_type` + `validation/controllers.py` (~272,327): a distance
  view must resolve its subspace via `constraint_view_subspace(spec)` (returns `"distance"`), and
  `"distance"` must infer a linear command (like `SubSpace.Position → LinearVelocity`) so an ACHD
  distance-control constraint does NOT require explicit `as`. (Today it reads raw `view.subspace`
  = None and demands `as`.) Keep the existing `as Force` distance path working.

### 4. ir_gen — `src/motion-spec/src/motion_spec/ir_gen.py`
Parse `slv-ext:DirectionAligned` acceleration constraints into IR parallel to AxisAligned, but
carrying a **direction** (the `geom-rel:Direction` node, whose runtime x/y/z come from the
PoseToDirection op) instead of an axis. The IR constraint record needs a `direction` field (the
direction coordinate id) alongside/instead of `axis`.

### 5. Codegen — `src/motion-spec/code-generator/solver.stg` (+ `codegen.py`, `kdl-config.stg`)
`solver-assign-f-cstr` currently emits `f_cstr(<subspace*3+axis>, <col>) = 1.0` (unit column). For a
DirectionAligned constraint emit the linear (or angular) sub-column from the runtime direction:
```
f_cstr_<solver>(<off>+0, <col>) = shared.<direction.id>.x();
f_cstr_<solver>(<off>+1, <col>) = shared.<direction.id>.y();
f_cstr_<solver>(<off>+2, <col>) = shared.<direction.id>.z();
```
where `<off> = subspace-to-jacobian-offset * 3` (0 for linear, 3 for angular). `e_acc(<col>)` is
unchanged. `shared.<direction.id>` is the `KDL::Vector` produced by the PoseToDirection runtime
(already normalized via the direction-coordinate codegen). Branch on constraint kind
(AxisAligned vs DirectionAligned) in the template.

## Test model (build+run gate)
Rewrite `admittance_arc_single.robmot`'s `forward` motion so its control constraint is distance-based
(this is the exact edit that surfaced the gaps):
```
WHILE {
    move-forward-dist: keeping distance between <shared.world.pose-ee-base> and <shared.world.pose-elbow-base> equal to <spec.reach-target>,
    hold-y:  keeping <shared.world.pose-ee-base>.position.y equal to <spec.forward-y>,
    hold-ori: keeping <shared.world.pose-ee-base>.orientation equal to <spec.forward-start-pose>.orientation
}
UNTIL { reached-forward: distance between <shared.world.pose-ee-base> and <shared.world.pose-elbow-base> greater than <spec.reach-target> }
```
and the handler-forward controller `constraint: <forward.move-forward-dist>`. Pick a physically
reachable `reach-target`. (This is a scratch edit to exercise the path — keep it or revert per the
final state, but it must build+run.)

## Guardrails
- Do NOT modify vendored `comp-rob2b` or any comp-rob2b SHACL — the new class goes in the secorolab
  `slv-ext` extension. No mj_kdl_wrapper change (option: the direction column is filled in generated
  code; if you think the wrapper needs changing, STOP and report — it shouldn't).
- Conformance is necessary but not sufficient — the sim must actually run and move.

## Acceptance gate (all must hold)
1. `pytest tests/test_motion_spec_graph.py tests/test_validation.py tests/test_shacl_conformance.py -q`
   → all pass (add/adjust graph-test assertions for the new DirectionAligned emission if needed).
2. All 4 conformance models still conform.
3. `pick_place_single`, `pick_place_single_rnea` (regression) and the distance-control
   `admittance_arc_single` each build (`make build MODEL=<m>`) and run headless to `steps=2500`
   with a valid `chain ready` line and no `error`/`not found`/`failed`; the distance-control model
   must actually drive the arm (the forward motion progresses).
   Env: `METAMODELS_PATH=/home/batsy/work/ms/src/metamodels INSTALL=/home/batsy/work/ms/install`.
4. Determinism: `pick_place_single-app.json` byte-identical across `PYTHONHASHSEED=1` vs `=999`.
5. Generated C++ for the two pick_place models unchanged (this feature only adds a new path).
