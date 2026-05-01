# TODO

## Immediate

- ABAG is reserved and immediately rejected as not implemented.


## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.

## Validation
- **Real robot**: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection. The posture-only path relies on this unconstrained mode; KDL documentation does not explicitly guarantee it for zero-constraint invocations. Run a posture-hold experiment with a single joint and confirm `tau_ctrl(i) == tau_ff(i)` to within numerical tolerance before relying on this in production.
