# TODO

## Remaining SHACL violations (after generator fixes)

Checked against all three pick_place variants. The original four root causes (xsd:double, Position
rdf:type, PoseCoordinateView orientation, declared Pose frame context) are fixed; see commit log.

- **`qudt_quant:Position` (4/2/2 per variant)** — Trajectory nodes or full Pose nodes used as
  `cstr:reference-value` for `PositionConstraint`. The constraint `follow-pos: keeping
  <world.pose-ee-base>.position equal to <spec.pick-traj>.position` emits the trajectory node URI
  as the reference, but SHACL requires the reference-value to have `rdf:type qudt_quant:Position`.
  Fix: in `_emit_context_ref_node`, when `ref.subspace == "position"` and the quantity is a
  trajectory or pose, resolve to the `.position` sub-node instead of the parent URI. Requires
  creating a position sub-node for trajectory quantities.

- **`geom-ent:Frame` (7/7/23 per variant)** — Scene object instances (`cube`, `robot`, `table`)
  are used as `geom-rel:of` on Pose/Orientation nodes. SHACL requires `sh:class geom-ent:Frame`.
  The 23 in pick_place_relative comes from many cube-relative constraints. Fix: same as the Semantic
  Gaps item below — emit a body-fixed frame node for each SceneObject and use that as `of`/`wrt`.

- **`geom-ent:Point` (6 each)** — Same root cause: rigid body instances used as `of` for Position
  nodes, but `PositionShape` requires `sh:class geom-ent:Point`. Fix: same frame/point node
  emission structural fix.

- **`All constraints must be evaluated` (6 each)** — Open-gripper and close-gripper constraints
  use `FeedForward` controllers which don't produce error signals, so no evaluator is created. The
  constraint-handler SPARQL shape requires every motion constraint to have an evaluator entry.

- **Monitor `event-queue`, `error`, `error-signal` (per handler)** — Until-section monitor nodes
  don't have `cstr_hdl:event-queue` (needs an `ev:EventQueue` node) or `cstr_hdl:error`. FeedForward
  monitors have no `error-signal` on the controller.

- **`dyn_coord:UniformGravitationalFieldCoordinate` (1 each)** — Gravity world quantity emitted
  without the coordinate type.

- **`slv:motion-drivers` more than 1 (1 each)** and **`slv:attached-to` (4 each)** — solver spec
  structural issues.

## RDF/Geometry Semantic Gaps (found during SHACL audit)

- **Scene objects (`cube`, `table`) lack `geom:Frame` type** — `geom-rel:Pose` requires `of`/`wrt` to be
  `geom:Frame`, but scene objects emitted via `_emit_structural_entities` only get `ENV.RigidObject`.
  Fix: emit a body-fixed frame node (e.g. `cube.frame`) for each SceneObject, point Pose `of`/`wrt` to it,
  and update IR routing to key on `ENV.RigidObject` instead of absence of `GEOM_ENT.Frame`.

- **Assembly `EnvironmentPositionEntry` uses rigid body as `of`/`wrt`** — `geom-rel:PositionShape` requires
  `geom:Point`, not a body instance. Each assembly object needs an origin `Point` node, and `world_node`
  needs a corresponding `Point` node for the `wrt`.

- **Assembly `EnvironmentOrientationEntry` missing `as-seen-by` and coordinate values** — `of`/`wrt` points
  to the rigid body instance (needs `geom:Frame`); no `GEOM_COORD["as-seen-by"]` triple is emitted
  (unlike position which has it); `_emit_orientation_rpy` is not called so angle values are not stored
  (works by accident today since all assembly orientations are `{}`).

- **`half_arm_2_link` naming mismatch in robmot files** — declared as `half-arm-2-link` (hyphens) in World
  context but referenced as `of: half_arm_2_link` (underscores) in pose props. Produces two separate
  URI nodes; the Pose quantity's `geom-rel:of` lands on a `Frame`-only node instead of the declared
  `SimplicialComplex` node. Fix: change prop references to `half-arm-2-link` in all three pick_place
  robmot files.

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add upstream `control-mode` support to the comp-rob2b constraint-handler context
  so legacy JSON can use the compact `"control-mode": "JointTorque"` form
  without a local context override.
- Add PID output clamp support as an authored controller parameter propagated
  through DSL, RDF/JSON-LD, IR, and C++ codegen. This is not required for the
  current pick/place behavior, but should be modeled explicitly if clamps are
  needed later.
- Generalize pose-axis error grouping to all vector-valued superobjects. The generator
  currently groups multi-axis pose subobject equality constraints into one
  `KDL::diff(parent_pose, target_pose)` and projects the requested components. The
  same semantic grouping should extend to velocity twists, acceleration twists, and
  wrenches where applicable.
- Avoid emitting unused component locals. Grouped pose-diff blocks declare all six
  component values even when only position or only orientation is consumed. Emit only
  the components referenced by the group to eliminate compiler warnings.
- Add a `motion-spec` regression suite and clean up its Ruff/Pyright findings.
  Current known debt: no tests under `src/motion-spec`, unused symbols in
  `count.py` / `ir_gen.py`, bare `except` handlers in `ir_gen.py`, and rdflib
  typing issues in manifest/IR parsing.

## Validation

- **Real robot**: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection. The posture-only path relies on this unconstrained mode; KDL documentation does not explicitly guarantee it for zero-constraint invocations. Run a posture-hold experiment with a single joint and confirm `tau_ctrl(i) == tau_ff(i)` to within numerical tolerance before relying on this in production.
