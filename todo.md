# TODO

## Immediate

- `grasp_goal_pose` not-assigned issue: the current generated `motion_pick.hpp` does assign
  `shared.grasp_goal_pose` (both in the update function and inside the lerp block), so the
  original failure is resolved. The setpoints variant already shows the scalar approach:
  `lower-z: keeping <shared.world.pose-ee-base>.position.z equal to <spec.grasp-z>`.

## Generator SHACL Violations (found by running `check.py` on `pick_place*`)

All three variants (`pick_place`, `pick_place_setpoints`, `pick_place_relative`) fail SHACL
validation. Violations are grouped by root cause in `rdf.py`:

- **Numeric literals emitted without `xsd:double` datatype (216 violations each variant)** —
  `cstr_hdl:decay-rate`, `cstr_hdl:proportional-gain`, `cstr_hdl:integral-gain`,
  `cstr_hdl:derivative-gain`, and `geom-coord:x/y/z` values are all emitted as plain string
  literals (`Literal(str(v))`). The constraint-handler metamodel requires `sh:datatype xsd:double`
  for all gain/decay scalars; the geometry metamodel requires it for coordinate values via
  `sh:xone`. Fix: import `XSD` from `rdflib.namespace` and use `Literal(float(v), datatype=XSD.double)`
  everywhere numeric scalars are added to the graph (controller gains lines ~2005–2015, coordinate
  emission lines ~517–519, ~586–588, ~1167, ~1186–1188, ~1577, ~1708).

- **Position nodes missing `rdf:type QUDT_QKIND.Position` (46 violations)** —
  `PositionConstraint.quantity`/`.reference-value` and `PoseCoordinateView.subobject` shapes require
  `sh:class qudt_quant:Position` (i.e., the node must carry `rdf:type qudt_quant:Position`). The
  generator emits `hasQuantityKind: Position` but not `rdf:type: Position`. Fix: add
  `self.graph.add((position_node, RDF.type, QUDT_QKIND.Position))` wherever position sub-nodes are
  created (e.g., lines ~1178–1181, ~1596–1598 in `rdf.py`).

- **`PoseCoordinateView` misused for orientation axes (18 violations)** —
  `map:PoseCoordinateView` in the metamodel (`map.ttl`) restricts `map:subspace` to `map:position`
  only. The generator types orientation component views as `map:PoseCoordinateView` with
  `map:subspace: map:rotation`, violating the shape. Fix: do not add `map:PoseCoordinateView` to
  orientation view nodes — type them as `map:View` only (no orientation-specific coordinate view
  type exists in the current metamodel).

- **Declared Pose nodes missing `of`, `wrt`, `as-seen-by` (minCount 1 violations per
  goal-pose/trajectory node)** — Goal poses (`grasp-goal-pose`, `goal-pose-above`, etc.) and
  Trajectory nodes (`pick-traj`, `lift-traj`, etc.) are typed as `geom-rel:Pose` and
  `geom-coord:PoseCoordinate`, both requiring `of`/`wrt` (`geom:Frame`) and `as-seen-by`
  (`geom:Frame`) at minCount 1. The generator emits these without frame context because the DSL does
  not require it on declared setpoint poses. Fix: propagate the EE/base frame pair from the motion
  spec's world context to declared Pose quantities (e.g., `of: g_pinch, wrt: base_link,
  as-seen-by: base_link`), or extend the DSL to allow authors to specify frame context on declared
  poses explicitly.

- **`direction`/`position-force` controller nodes have untyped x/y/z (pick_place_relative only,
  `sh:xone` violations)** — Impedance controller direction and force application nodes have
  `geom-coord:x/y/z: "0.0"` (plain string). The geometry coordinate shape uses `sh:xone` between
  "values present typed as `xsd:double`" and "values absent", so a plain string fails both
  branches. Same root cause and fix as the first item above.

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

- **`check.py` was not loading constraints or imports** — `rdflib.Dataset.triples()` in this rdflib fork
  only searches the default graph; DSL-generated manifests use named graphs. Fixed by switching to
  `g.quads()` throughout. `geometry.shacl.ttl` added to the secorolab `metamodels/` repo; manifests
  should reference it via `https://secorolab.github.io/metamodels/geometry/geometry.shacl.ttl` with
  a `metamodels/` IRI mapping once the generator is updated.

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
