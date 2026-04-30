# Pending Motion-Spec DSL Work

- Implement frame-aware ACHD alpha generation. KDL expects alpha unit constraint-force columns in the solver base frame; current codegen emits axis-aligned base-frame columns and does not transform authored `as-seen-by` frames.
- Add first-class semantic tests for frame transforms once alpha generation supports non-base frames. These should cover pose, velocity twist, and mixed pose/twist constraints.
- Refactor `motion_spec_graph.py` around explicit semantic records for controller outputs, acceleration constraints, and force commands so graph emission and validation share the same classification logic.
- Split validation into focused modules or phases. The current single `validation.py` is growing around solver, controller, context, and robot concerns, which makes unsupported combinations harder to audit.
- Add end-to-end generated-code fixtures for representative valid motions beyond the sliding 5-axis case: full 6-axis pose hold, force-only contact, posture-only joint torque, and mixed acceleration plus external force.
