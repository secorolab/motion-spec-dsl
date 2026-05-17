# TODO

## Immediate

- ABAG is reserved and immediately rejected as not implemented.


## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Add upstream `control-mode` support to the comp-rob2b constraint-handler context
  so legacy JSON can use the compact `"control-mode": "JointTorque"` form
  without a local context override.
- Add PID output clamp support as an authored controller parameter propagated
  through DSL, RDF/JSON-LD, IR, and C++ codegen. This is not required for the
  current pick/place behavior, but should be modeled explicitly if clamps are
  needed later.
- Add a `motion-spec` regression suite and clean up its Ruff/Pyright findings.
  Current known debt: no tests under `src/motion-spec`, unused symbols in
  `count.py` / `ir_gen.py`, bare `except` handlers in `ir_gen.py`, and rdflib
  typing issues in manifest/IR parsing.

## Validation
- **Real robot**: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection. The posture-only path relies on this unconstrained mode; KDL documentation does not explicitly guarantee it for zero-constraint invocations. Run a posture-hold experiment with a single joint and confirm `tau_ctrl(i) == tau_ff(i)` to within numerical tolerance before relying on this in production.
