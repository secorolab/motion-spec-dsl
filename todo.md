# TODO

## Immediate

- fix this: I found the concrete failure: grasp_goal_pose is never assigned in the generated motion_pick.hpp, so KDL::diff(shared.pose_ee_base, shared.grasp_goal_pose) drives against  a default KDL::Frame target near the robot base.

        lower-z:    keeping <shared.world.pose-ee-base>.position  equal to <spec.grasp-position> 

this should work as long as both are of same types.


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
