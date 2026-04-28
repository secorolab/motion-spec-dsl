# SPDX-License-Identifier: MPL-2.0
"""Semantic validation for fully constructed motion-spec DSL models."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable

from textx import get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintAlias,
    ConstraintHandler,
    ConstraintSpecification,
    ControllerAlias,
    ControllerEntry,
    ControllerMode,
    ContextRef,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    Model,
    MotionSpec,
    QuantityType,
    RobotSpec,
    RobotType,
    SolverEntry,
    SolverRef,
    SubSpace,
    ContextQuantity,
    WorldQuantity,
    WorldQuantityType,
    _resolved_controller,
    _resolved_spec,
    _resolved_solver,
    _resolved_context_quantity,
    _resolved_world_quantity,
)


def _semantic_error(message: str, obj: object | None = None) -> TextXSemanticError:
    if obj is None:
        return TextXSemanticError(message)
    try:
        return TextXSemanticError(message, **get_location(obj))
    except Exception:
        return TextXSemanticError(message)


def _motion_specs(model: Model) -> Iterable[MotionSpec]:
    return (spec for spec in model.specs if isinstance(spec, MotionSpec))


def _constraint_handlers(model: Model) -> Iterable[ConstraintHandler]:
    return (spec for spec in model.specs if isinstance(spec, ConstraintHandler))


def _robot_specs(model: Model) -> Iterable[RobotSpec]:
    return (spec for spec in model.specs if isinstance(spec, RobotSpec))


def motion_constraint_items(spec: MotionSpec) -> list[ConstraintSpecification | ConstraintAlias]:
    """All section items (inline specs and aliases) by their local name."""
    return [item for section in spec.sections for item in section.constraints]


def motion_constraints(spec: MotionSpec) -> list[ConstraintSpecification]:
    """All resolved ConstraintSpecification objects for this motion."""
    return [_resolved_spec(item) for item in motion_constraint_items(spec)]


def handler_controller_items(handler: ConstraintHandler):
    return list(handler.controllers)


def handler_solver_items(handler: ConstraintHandler):
    return list(handler.solvers)


def _robot_component(robot: RobotSpec, component_name: str):
    if robot.type == RobotType.MobileManipulator:
        if component_name == "base":
            return robot.base
        return next(
            (component for component in robot.manipulators if component.name == component_name),
            None,
        )
    if component_name == robot.name or component_name == "chain":
        return robot.chain
    return None


def _robot_component_anchor(robot: RobotSpec, component_name: str, anchor: str) -> str:
    component = _robot_component(robot, component_name)
    return getattr(component, anchor, "") if component is not None else ""


def _solver_component(solver: SolverEntry):
    if solver.robot.component_name is None:
        return solver.robot.robot_spec.chain
    return _robot_component(solver.robot.robot_spec, solver.robot.component_name)


def _solver_component_anchor(solver: SolverEntry, anchor: str) -> str:
    if solver.robot.component_name is None:
        return getattr(solver.robot.robot_spec.chain, anchor, "")
    return _robot_component_anchor(
        solver.robot.robot_spec,
        solver.robot.component_name,
        anchor,
    )


def _anchor_matches_solver(anchor, solver: SolverEntry, expected_anchor: str) -> bool:
    return (
        anchor.robot_spec is solver.robot.robot_spec
        and anchor.component_name == solver.robot.component_name
        and anchor.anchor == expected_anchor
    )


def _infer_command_type(subspace: SubSpace | None) -> QuantityType | None:
    if subspace is None:
        return None
    return {
        SubSpace.LinVel: QuantityType.LinearVelocity,
        SubSpace.AngVel: QuantityType.AngularVelocity,
        SubSpace.Force: QuantityType.Force,
        SubSpace.Torque: QuantityType.Torque,
    }.get(subspace)


def validate_unique_constraint_names(model: Model) -> None:
    for motion in _motion_specs(model):
        items_by_name: dict[str, list[object]] = defaultdict(list)
        for item in motion_constraint_items(motion):
            items_by_name[item.name].append(item)

        duplicates = {name: items for name, items in items_by_name.items() if len(items) > 1}
        if duplicates:
            names = ", ".join(sorted(duplicates))
            first_duplicate = next(iter(duplicates.values()))[1]
            raise _semantic_error(
                f"Motion '{motion.name}' has duplicate constraint name(s): {names}. "
                "Constraint names must be unique across WHEN, WHILE, and UNTIL.",
                first_duplicate,
            )


def validate_constraint_aliases(model: Model) -> None:
    del model


def validate_context_aliases(model: Model) -> None:
    del model


def validate_handler_aliases(model: Model) -> None:
    del model


def validate_handler_constraint_assembly(model: Model) -> None:
    for handler in _constraint_handlers(model):
        assembled_specs = {
            id(_resolved_spec(item)) for item in motion_constraint_items(handler.motion)
        }

        for monitor in handler.monitors:
            if id(monitor.constraint.constraint) not in assembled_specs:
                raise _semantic_error(
                    f"Monitor '{monitor.name}' references constraint "
                    f"'{monitor.constraint}', but handler '{handler.name}' primary motion "
                    f"'{handler.motion.name}' does not assemble it.",
                    monitor,
                )

        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            if id(resolved_controller.params.constraint.constraint) not in assembled_specs:
                raise _semantic_error(
                    f"Controller '{controller.name}' references constraint "
                    f"'{resolved_controller.params.constraint}', but handler '{handler.name}' "
                    f"primary motion '{handler.motion.name}' does not assemble it.",
                    controller,
                )


def _decl_motion(obj: object) -> MotionSpec | None:
    context = getattr(obj, "parent", None)
    motion = getattr(context, "parent", None)
    return motion if isinstance(motion, MotionSpec) else None


def _context_ref_value(ref: ContextRef) -> ContextQuantity | None:
    value = (
        getattr(ref, "quantity", None)
        or getattr(ref, "value", None)
        or getattr(ref, "inline_quantity", None)
    )
    return value if isinstance(value, ContextQuantity) else None


def _is_inline_context_value(value: ContextQuantity) -> bool:
    return isinstance(getattr(value, "parent", None), ContextRef) and getattr(
        value.parent,
        "context_scope",
        None,
    ) is not None


def _constraint_context_refs(constraint: ConstraintSpecification) -> list[ContextRef]:
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        return [expr.reference]
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        return [expr.threshold]
    if isinstance(expr, BilateralConstraint):
        return [expr.lower, expr.upper]
    return []


def _constraint_view_quantities(constraint: ConstraintSpecification) -> list[object | None]:
    if (
        getattr(constraint.view, "distance_from", None) is not None
        and getattr(constraint.view, "distance_to", None) is not None
    ):
        return [constraint.view.distance_from, constraint.view.distance_to]
    return [constraint.view.quantity]


def validate_constraint_context_refs(model: Model) -> None:
    for motion in _motion_specs(model):
        for constraint in motion_constraints(motion):
            for quantity in _constraint_view_quantities(constraint):
                quantity = (
                    _resolved_world_quantity(quantity)
                    if isinstance(quantity, WorldQuantity)
                    else quantity
                )
                if not isinstance(quantity, WorldQuantity):
                    raise _semantic_error(
                        f"Constraint '{constraint.name}' references world quantity "
                        f"'{quantity}', but it is not resolved.",
                        constraint,
                    )

            for ref in _constraint_context_refs(constraint):
                value = _context_ref_value(ref)
                value = _resolved_context_quantity(value) if isinstance(value, ContextQuantity) else value
                if value is None:
                    raise _semantic_error(
                        f"Constraint '{constraint.name}' has an unresolved context reference.",
                        constraint,
                    )
                if _decl_motion(value) is None and _is_inline_context_value(value):
                    continue
                if _decl_motion(value) is None:
                    raise _semantic_error(
                        f"Constraint '{constraint.name}' references value '{value.name}', "
                        "but it is not resolved.",
                        ref,
                    )


def validate_handler_constraint_refs(model: Model) -> None:
    del model


def validate_handler_requirements(model: Model) -> None:
    for handler in _constraint_handlers(model):
        if handler.motion.while_.constraints and not (handler.controllers or handler.monitors):
            raise _semantic_error(
                "ConstraintHandler with WHILE constraints must have at least one controller or monitor.",
                handler,
            )
        if (handler.motion.when.constraints or handler.motion.until.constraints) and not handler.monitors:
            raise _semantic_error(
                "ConstraintHandler with WHEN or UNTIL constraints must have at least one monitor.",
                handler,
            )


def validate_robot_specs(model: Model) -> None:
    for robot in _robot_specs(model):
        if robot.type == RobotType.MobileManipulator:
            if robot.base is None:
                raise _semantic_error(f"Robot '{robot.name}' requires a base component.", robot)
            if not robot.manipulators:
                raise _semantic_error(
                    f"Robot '{robot.name}' requires at least one manipulator component.",
                    robot,
                )
            continue

        if robot.type == RobotType.Manipulator and (
            robot.chain is None or not robot.chain.root or not robot.chain.end
        ):
            raise _semantic_error(
                f"Manipulator robot '{robot.name}' requires chain root and end.",
                robot,
            )
        if robot.type == RobotType.MobileBase and (robot.chain is None or not robot.chain.root):
            raise _semantic_error(f"MobileBase robot '{robot.name}' requires chain root.", robot)


def validate_solver_refs(model: Model) -> None:
    for handler in _constraint_handlers(model):
        for solver in handler.solvers:
            component = _solver_component(solver)
            if component is None:
                raise _semantic_error(
                    f"Solver '{solver.name}' references unknown robot or component '{solver.robot}'.",
                    solver,
                )

            expected_root = _solver_component_anchor(solver, "root")
            if not _anchor_matches_solver(solver.root, solver, "root"):
                expected = (
                    f"{solver.robot}.chain.root"
                    if solver.robot.component_name is None
                    else f"{solver.robot}.root"
                )
                raise _semantic_error(
                    f"Solver '{solver.name}' root must reference '{expected}'.",
                    solver,
                )
            if expected_root and expected_root != _robot_component_anchor(
                solver.root.robot_spec,
                solver.root.component_name or solver.root.robot_spec.name,
                solver.root.anchor,
            ):
                raise _semantic_error(
                    f"Solver '{solver.name}' root does not match robot '{solver.robot}'.",
                    solver,
                )

            requires_end = getattr(component, "end", "") != ""
            if requires_end and solver.end is None:
                raise _semantic_error(
                    f"Solver '{solver.name}' for robot '{solver.robot}' requires end.",
                    solver,
                )
            if not requires_end and solver.end is not None:
                raise _semantic_error(
                    f"Solver '{solver.name}' for robot '{solver.robot}' must not define end.",
                    solver,
                )
            if solver.end is not None and not _anchor_matches_solver(solver.end, solver, "end"):
                expected = (
                    f"{solver.robot}.chain.end"
                    if solver.robot.component_name is None
                    else f"{solver.robot}.end"
                )
                raise _semantic_error(
                    f"Solver '{solver.name}' end must reference '{expected}'.",
                    solver,
                )


def validate_controller_solver_refs(model: Model) -> None:
    for handler in _constraint_handlers(model):
        resolved_handler_solvers = [_resolved_solver(solver) for solver in handler.solvers]
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            explicit_solver = getattr(resolved_controller.solver, "solver", None)
            if explicit_solver is None:
                if len(resolved_handler_solvers) == 1:
                    continue
                raise _semantic_error(
                    f"Controller '{controller.name}' must specify solver because handler "
                    f"'{handler.name}' assembles {len(resolved_handler_solvers)} solvers.",
                    controller,
                )
            if all(id(explicit_solver) != id(solver) for solver in resolved_handler_solvers):
                raise _semantic_error(
                    f"Controller '{controller.name}' references solver "
                    f"'{explicit_solver.name}', but handler "
                    f"'{handler.name}' does not assemble it.",
                    controller,
                )


def _controller_solver(controller: ControllerEntry | ControllerAlias) -> SolverEntry | None:
    solver_ref = getattr(controller, "solver", None)
    if isinstance(solver_ref, SolverRef):
        return solver_ref.solver
    return None


def _controller_domain(controller: ControllerEntry | ControllerAlias) -> str:
    resolved_controller = _resolved_controller(controller)
    subspace = resolved_controller.params.constraint.constraint.view.subspace
    command_type = resolved_controller.command_type or _infer_command_type(subspace)
    if command_type in {QuantityType.Force, QuantityType.Torque} or subspace in {
        SubSpace.Force,
        SubSpace.Torque,
    }:
        return "force"
    return "pose"


def validate_supported_solver_algorithms(model: Model) -> None:
    for handler in _constraint_handlers(model):
        for solver in handler.solvers:
            resolved_solver = _resolved_solver(solver)
            if str(resolved_solver.algorithm) == "RNE":
                raise _semantic_error(
                    f"Solver '{resolved_solver.name}' in handler '{handler.name}' uses RNE, "
                    "but RNE is not modeled in the DSL generator yet.",
                    solver,
                )


def validate_mixed_solver_domains(model: Model) -> None:
    for handler in _constraint_handlers(model):
        domains_by_algorithm: dict[str, set[str]] = {}
        for controller in handler.controllers:
            solver = _controller_solver(controller)
            if solver is None:
                continue
            domains_by_algorithm.setdefault(str(solver.algorithm), set()).add(
                _controller_domain(controller)
            )

        if "ACHD" not in domains_by_algorithm or "RNE" not in domains_by_algorithm:
            continue

        overlapping_domains = domains_by_algorithm["ACHD"] & domains_by_algorithm["RNE"]
        if overlapping_domains:
            overlap = ", ".join(sorted(overlapping_domains))
            raise _semantic_error(
                f"Handler '{handler.name}' mixes ACHD and RNE on the same domain(s): "
                f"{overlap}. They may only coexist when ACHD and RNE controllers are "
                "split across pose and force domains.",
                handler,
            )


def validate_controller_commands(model: Model) -> None:
    for handler in _constraint_handlers(model):
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            constraint_spec = resolved_controller.params.constraint.constraint
            subspace = constraint_spec.view.subspace
            command_type = resolved_controller.command_type or _infer_command_type(subspace)
            if command_type is None:
                raise _semantic_error(
                    f"Controller '{controller.name}' requires explicit 'as' for "
                    f"constraint subspace '{subspace}'.",
                    controller,
                )
            quantity = constraint_spec.view.quantity
            if (
                isinstance(quantity, WorldQuantity)
                and quantity.type == WorldQuantityType.JointPosition
                and resolved_controller.control_mode != ControllerMode.Posture
            ):
                raise _semantic_error(
                    f"Controller '{controller.name}' targets JointPosition and must declare "
                    "'for Posture'.",
                    controller,
                )
            if resolved_controller.control_mode == ControllerMode.Posture:
                if command_type != QuantityType.Torque:
                    raise _semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must use 'as Torque'.",
                        controller,
                    )
                if not isinstance(quantity, WorldQuantity) or quantity.type != WorldQuantityType.JointPosition:
                    raise _semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must target a JointPosition constraint.",
                        controller,
                    )
                quantity_props = getattr(quantity, "props", None)
                if quantity_props is None or not any(
                    getattr(pair, "key", None) == "of" and getattr(pair, "value", "")
                    for pair in getattr(quantity_props, "pairs", [])
                ):
                    raise _semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must target a JointPosition with explicit 'of'.",
                        controller,
                    )
            if (
                resolved_controller.apply_at is not None
                and resolved_controller.apply_at.type != WorldQuantityType.Link
            ):
                raise _semantic_error(
                    f"Controller '{controller.name}' apply at target must be a Link.",
                    controller,
                )


def validate_motion_spec_coverage(model: Model) -> None:
    referenced = {
        handler.motion.name
        for handler in _constraint_handlers(model)
        if handler.motion is not None
    }
    for motion in _motion_specs(model):
        if motion.name not in referenced:
            raise _semantic_error(
                f"MotionSpec '{motion.name}' is not referenced by any ConstraintHandler. "
                "Every MotionSpec must be bound to exactly one ConstraintHandler.",
                motion,
            )


def validate_model(model: Model, metamodel=None) -> None:
    del metamodel
    validate_robot_specs(model)
    validate_unique_constraint_names(model)
    validate_constraint_aliases(model)
    validate_context_aliases(model)
    validate_constraint_context_refs(model)
    validate_handler_constraint_refs(model)
    validate_handler_constraint_assembly(model)
    validate_handler_aliases(model)
    validate_handler_requirements(model)
    validate_solver_refs(model)
    validate_controller_solver_refs(model)
    validate_supported_solver_algorithms(model)
    validate_mixed_solver_domains(model)
    validate_controller_commands(model)
    validate_motion_spec_coverage(model)
