# TODO

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Clean up `motion-spec` Ruff/Pyright findings. (Regression suite now runs: `test_shacl_conformance.py`
  exercises 4 conforming models end-to-end.) Pyright reports ~150 src findings, almost all rdflib/textx
  stub gaps (`Node` not assignable to `float`) — a large low-value grind.

## Validation

- Real robot: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection.
