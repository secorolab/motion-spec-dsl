# Pending Motion-Spec DSL Work

- Refactor `motion_spec_graph.py` around explicit semantic records for controller outputs, acceleration constraints, and force commands so graph emission and validation share the same classification logic.
- Split validation into focused modules or phases. The current single `validation.py` is growing around solver, controller, context, and robot concerns, which makes unsupported combinations harder to audit.
- Add end-to-end generated-code fixtures for representative valid motions beyond the sliding 5-axis case: full 6-axis pose hold, force-only contact, posture-only joint torque, and mixed acceleration plus external force.

## Later

- Carry `JointPosition` / posture-control semantics through `motion-spec` IR parsing and downstream codegen.
- Decide whether non-posture joint-torque controllers should be supported as a separate dispatch family from `for Posture`.
- Support explicit distance constraints between poses with different `wrt` frames when the required transform path is available; compose the transforms instead of requiring both endpoints to be expressed in the same frame.
- Add prioritization support for mixed ACHD motion drivers, including `joint-force`.
