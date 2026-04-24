# TODO

- Carry `JointPosition` / posture-control semantics through `motion-spec` IR parsing and downstream codegen.
- Decide whether non-posture joint-torque controllers should be supported as a separate dispatch family from `for Posture`.
- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
- Decide whether posture control should allow unilateral/bilateral joint-limit constraints in addition to the current equality/setpoint path.
- Add example models that combine posture torque with cartesian acceleration constraints and cartesian force in one ACHD handler.
