# SPDX-License-Identifier: MPL-2.0
"""Semantic validation for fully constructed motion-spec DSL models."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable

from textx import get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.generators.classes import (
    BilateralConstraint,
    ConstraintHandler,
    ConstraintSpecification,
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
    SubSpace,
    ValueVariable,
    WorldQuantity,
    WorldQuantityType,
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


def motion_constraints(spec: MotionSpec) -> list[ConstraintSpecification]:
    return [
        constraint
        for section in spec.sections
        for constraint in section.constraints
    ]


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
    return {
        SubSpace.LinVel: QuantityType.LinearVelocity,
        SubSpace.AngVel: QuantityType.AngularVelocity,
        SubSpace.Force: QuantityType.Force,
        SubSpace.Torque: QuantityType.Torque,
    }.get(subspace)


def validate_unique_constraint_names(model: Model) -> None:
    for motion in _motion_specs(model):
        constraints_by_name: dict[str, list[ConstraintSpecification]] = defaultdict(list)
        for constraint in motion_constraints(motion):
            constraints_by_name[constraint.name].append(constraint)

        duplicates = {
            name: constraints
            for name, constraints in constraints_by_name.items()
            if len(constraints) > 1
        }
        if duplicates:
            names = ", ".join(sorted(duplicates))
            first_duplicate = next(iter(duplicates.values()))[1]
            raise _semantic_error(
                f"Motion '{motion.name}' has duplicate constraint name(s): {names}. "
                "Constraint names must be unique across WHEN, WHILE, and UNTIL.",
                first_duplicate,
            )


def _decl_motion(obj: object) -> MotionSpec | None:
    context = getattr(obj, "parent", None)
    motion = getattr(context, "parent", None)
    return motion if isinstance(motion, MotionSpec) else None


def _context_ref_value(ref: ContextRef) -> ValueVariable | None:
    value = getattr(ref, "valRef", None) or getattr(ref, "value", None)
    return value if isinstance(value, ValueVariable) else None


def _constraint_context_refs(constraint: ConstraintSpecification) -> list[ContextRef]:
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        return [expr.reference]
    if isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
        return [expr.threshold]
    if isinstance(expr, BilateralConstraint):
        return [expr.lower, expr.upper]
    return []


def validate_constraint_context_refs(model: Model) -> None:
    for motion in _motion_specs(model):
        for constraint in motion_constraints(motion):
            quantity = constraint.view.quantity
            if not isinstance(quantity, WorldQuantity) or _decl_motion(quantity) is not motion:
                raise _semantic_error(
                    f"Constraint '{constraint.name}' references world quantity "
                    f"'{quantity}', but it is not declared in motion '{motion.name}'.",
                    constraint,
                )

            for ref in _constraint_context_refs(constraint):
                value = _context_ref_value(ref)
                if value is None:
                    raise _semantic_error(
                        f"Constraint '{constraint.name}' has an unresolved context reference.",
                        constraint,
                    )
                if _decl_motion(value) is not motion:
                    raise _semantic_error(
                        f"Constraint '{constraint.name}' references value '{value.name}', "
                        f"but it is not declared in motion '{motion.name}'.",
                        ref,
                    )


def validate_handler_constraint_refs(model: Model) -> None:
    for handler in _constraint_handlers(model):
        for monitor in handler.monitors:
            if monitor.constraint.motion is not handler.motion:
                raise _semantic_error(
                    f"Monitor '{monitor.name}' references motion "
                    f"'{monitor.constraint.motion.name}', but handler '{handler.name}' "
                    f"is bound to motion '{handler.motion.name}'.",
                    monitor,
                )

        for controller in handler.controllers:
            ref = controller.params.constraint
            if ref.motion is not handler.motion:
                raise _semantic_error(
                    f"Controller '{controller.name}' references motion "
                    f"'{ref.motion.name}', but handler '{handler.name}' "
                    f"is bound to motion '{handler.motion.name}'.",
                    controller,
                )


def validate_handler_requirements(model: Model) -> None:
    for handler in _constraint_handlers(model):
        if handler.motion.while_.constraints and not handler.controllers:
            raise _semantic_error(
                "ConstraintHandler with WHILE constraints must have at least one controller.",
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
        for controller in handler.controllers:
            if controller.params.solver not in handler.solvers:
                raise _semantic_error(
                    f"Controller '{controller.name}' references solver "
                    f"'{controller.params.solver.name}' outside handler '{handler.name}'.",
                    controller,
                )


def validate_controller_commands(model: Model) -> None:
    for handler in _constraint_handlers(model):
        for controller in handler.controllers:
            subspace = controller.params.constraint.constraint.view.subspace
            command_type = controller.command_type or _infer_command_type(subspace)
            if command_type is None:
                raise _semantic_error(
                    f"Controller '{controller.name}' requires explicit 'as' for "
                    f"constraint subspace '{subspace}'.",
                    controller,
                )
            if controller.apply_at is not None and controller.apply_at.type != WorldQuantityType.Link:
                raise _semantic_error(
                    f"Controller '{controller.name}' apply at target must be a Link.",
                    controller,
                )


def validate_model(model: Model, metamodel=None) -> None:
    del metamodel
    validate_robot_specs(model)
    validate_unique_constraint_names(model)
    validate_constraint_context_refs(model)
    validate_handler_constraint_refs(model)
    validate_handler_requirements(model)
    validate_solver_refs(model)
    validate_controller_solver_refs(model)
    validate_controller_commands(model)
