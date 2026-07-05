# TODO

## Later

- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.


## Validation

- Real robot: verify that `KDL::ChainHdSolver_Vereshchagin` with `num_constraints=0` and non-zero `tau_ff` correctly passes joint torques through to `tau_ctrl` without corruption or solver rejection.
