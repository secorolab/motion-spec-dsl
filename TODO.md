# TODO

- Carry `JointPosition` / posture-control semantics through `motion-spec` IR parsing and downstream codegen.
- Decide whether non-posture joint-torque controllers should be supported as a separate dispatch family from `for Posture`.
- Support explicit distance constraints between poses with different `wrt` frames when the required transform path is available; compose the transforms instead of requiring both endpoints to be expressed in the same frame.
- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
