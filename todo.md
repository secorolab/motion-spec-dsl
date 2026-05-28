# TODO

## SHACL violations (remaining after generator fixes)

Checked against all three pick_place variants. Fixed issues are in the commit log.

- **`qudt_quant:Position` (4/2/2)** — trajectory/pose node used as `PositionConstraint` reference-value instead of the `.position` sub-node. Fix: in `_emit_context_ref_node`, resolve `ref.subspace == "position"` to the sub-node URI.

- **`geom-ent:Frame` (7/7/23) and `geom-ent:Point` (6)** — scene object rigid body instances (`cube`, `robot`, `table`) used directly as `geom-rel:of`/`wrt`. Fix: emit a body-fixed frame node per SceneObject; emit an origin Point node per assembly object; update IR routing to key on `ENV.RigidObject` instead of absence of `GEOM_ENT.Frame`.

- **Assembly orientation `as-seen-by` (4)** — `EnvironmentOrientationEntry` never emits `GEOM_COORD["as-seen-by"]`.

- **`half_arm_2_link` naming (pick_place_relative)** — declared as `half-arm-2-link` in World context but referenced with underscores in pose props. Fix: change prop references to use hyphens in all three pick_place robmot files.

- **`All constraints must be evaluated` (6)** — open/close-gripper FeedForward constraints have no evaluator. Structural gap; FeedForward controllers don't produce error signals.

- **Monitor `event-queue`/`error`/`error-signal`** — until-section monitors missing `ev:EventQueue` node; FeedForward monitors missing error/error-signal.

- **`dyn_coord:UniformGravitationalFieldCoordinate` (1)** — gravity world quantity missing coordinate type.

- **`slv:motion-drivers` > 1 (1) and `slv:attached-to` (4)** — solver spec structural issues.

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add upstream `control-mode` support to the comp-rob2b constraint-handler context so legacy JSON can use the compact `"control-mode": "JointTorque"` form without a local context override.
- Add PID output clamp support as an authored controller parameter propagated through DSL, RDF/JSON-LD, IR, and C++ codegen.
- Generalize pose-axis error grouping to all vector-valued superobjects (velocity twists, acceleration twists, wrenches).
- Avoid emitting unused component locals in grouped pose-diff blocks.
- Add a `motion-spec` regression suite and clean up Ruff/Pyright findings.

## Validation

- **Real robot**: verify `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` passes joint torques through to `tau_ctrl` without corruption. Run a posture-hold experiment and confirm `tau_ctrl(i) == tau_ff(i)` to within numerical tolerance.
