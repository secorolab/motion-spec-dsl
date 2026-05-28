# TODO

## SHACL Status

- `pick_place` and `pick_place_relative` currently conform with the generated manifest SHACL set.
- Add regression tests that run JSON-LD generation plus `motion_spec/check.py` for both examples.
- Keep local secorolab SHACL extensions aligned with upstream comp-rob2b shapes as upstream evolves.

## Semantic Follow-Up

- Decide whether comp-rob2b `PoseCoordinate` requiring `as-seen-by == with-respect-to` is sufficient for all relative-frame use cases, or whether an explicit relative/view frame concept is needed upstream.
- Revisit FeedForward/gripper evaluator semantics if assignment evaluators need runtime-visible outputs beyond satisfying constraint coverage.
- Add proper body-fixed frame/point nodes for scene objects instead of relying on object nodes also being typed as `Frame`, `Point`, and `SimplicialComplex`.
- Expand assembly orientation values with `_emit_orientation_rpy` when non-empty assembly orientations are used.
- Fix `half_arm_2_link` naming drift in robmot files if those models are still active.

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add upstream `control-mode` support to the comp-rob2b constraint-handler context.
- Add PID output clamp support as an authored controller parameter if clamps become part of the model.
- Generalize pose-axis error grouping to velocity twists, acceleration twists, and wrenches where applicable.
- Avoid emitting unused component locals in grouped pose-diff codegen.
- Add a `motion-spec` regression suite and clean up its Ruff/Pyright findings.

## Validation

- Real robot: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection.
