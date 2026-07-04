# TODO

## Semantic Follow-Up

- Add proper body-fixed frame/point nodes for scene objects (and `world`/robot frames) instead of
  fusing `Frame` + `Point` + `SimplicialComplex` onto one node. Load-bearing, not localized: SHACL
  requires `Pose.of`=`Frame`, `Position.of`=`Point`, `Twist.of`=`SimplicialComplex`, so a single
  referenced entity must satisfy all three today. The fusion is now applied to pose/twist `of`/`wrt`
  kinematic frames too (`rdf.py::_emit_structural_entities`) — it is what makes the trajectory demos
  conform, so a proper context-aware `of`/`wrt` resolution (pose→frame, position→point,
  twist→body+reference-point) must preserve that conformance, not just refactor structure.

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add PID output clamp support as an authored controller parameter if clamps become part of the model.
- Clean up `motion-spec` Ruff/Pyright findings. (Regression suite now runs: `test_shacl_conformance.py`
  exercises 4 conforming models end-to-end.) Ruff isn't installed in the venv; Pyright reports ~150 src
  findings, almost all rdflib/textx stub gaps (`Node` not assignable to `float`) — a large low-value grind.

## Validation

- Real robot: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection.
