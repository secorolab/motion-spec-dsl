# TODO

## SHACL Status

- Keep local secorolab SHACL extensions aligned with upstream comp-rob2b shapes as upstream evolves.

## Semantic Follow-Up

- Add proper body-fixed frame/point nodes for scene objects (and `world`/robot frames) instead of
  fusing `Frame` + `Point` + `SimplicialComplex` onto one node. Load-bearing, not localized: SHACL
  requires `Pose.of`=`Frame`, `Position.of`=`Point`, `Twist.of`=`SimplicialComplex`, so a single
  referenced entity must satisfy all three today. A proper fix needs context-aware `of`/`wrt`
  resolution (pose→frame, position→point, twist→body+reference-point) across all entities; it fixes
  no current SHACL violation, so it's a standalone refactor.

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add PID output clamp support as an authored controller parameter if clamps become part of the model.
- Add a `motion-spec` regression suite and clean up its Ruff/Pyright findings.

## Validation

- Real robot: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection.
