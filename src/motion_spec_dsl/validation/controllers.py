# SPDX-License-Identifier: MPL-2.0
"""Controller command and solver-interface validation."""

from __future__ import annotations

from collections import defaultdict

from motion_spec_dsl.controller_semantics import (
    SUBSPACE_ALIAS,
    axis_label,
    constraint_view_subspace,
    controller_command_record,
    infer_command_type,
    pose_diff_components,
)
from motion_spec_dsl.domain import (
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    ContextQuantity,
    EqualityConstraint,
    Model,
    ProfileSpec,
    QuantityType,
    ReferenceGeneratorType,
    SubSpace,
    WorldEntityType,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
    _resolved_controller,
    _resolved_world_quantity,
)
from motion_spec_dsl.validation.common import constraint_handlers, semantic_error
from motion_spec_dsl.validation.solvers import handler_controller_solver


def _achd_acceleration_axis_keys(
    controller: ControllerEntry | ControllerAlias,
) -> list[tuple[str, str]]:
    command = controller_command_record(controller)
    return [(record.subspace, record.axis) for record in command.acceleration_constraints]


def _profile_quantity(controller: ControllerEntry) -> ContextQuantity | None:
    ref = getattr(controller.params, "profile", None)
    if ref is None:
        return None
    quantity = getattr(ref, "quantity", None) or getattr(ref, "inline_quantity", None)
    return _resolved_context_quantity(quantity) if isinstance(quantity, ContextQuantity) else None


def _supports_velocity_profile(spec) -> bool:
    view = spec.view
    if getattr(view, "distance_from", None) is not None and getattr(view, "distance_to", None) is not None:
        return True
    quantity = getattr(view, "quantity", None)
    return (
        isinstance(quantity, WorldQuantity)
        and _resolved_world_quantity(quantity).type == WorldQuantityType.Pose
        and getattr(view, "subspace", None) == SubSpace.Position
        and getattr(view, "axis", None) is not None
    )


def _requires_measured_velocity(spec) -> bool:
    view = spec.view
    return not (
        getattr(view, "distance_from", None) is not None
        and getattr(view, "distance_to", None) is not None
    )


def _authored_subspace(view) -> str | None:
    subspace = getattr(view, "subspace", None)
    if subspace is None:
        return None
    raw = str(getattr(subspace, "value", subspace))
    return SUBSPACE_ALIAS.get(raw, raw)


def _validate_pid_measured_derivative(controller: ControllerEntry, resolved_controller: ControllerEntry) -> None:
    measured_derivative = getattr(resolved_controller.params, "measured_derivative", None)
    if measured_derivative is None:
        return

    command = controller_command_record(resolved_controller)
    quantity = command.quantity
    if (
        quantity is None
        or quantity.type != WorldQuantityType.Pose
        or not command.acceleration_constraints
        or not isinstance(resolved_controller.params.constraint.constraint.expr, EqualityConstraint)
    ):
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative is only supported for pose equality PID controllers.",
            measured_derivative,
        )

    measured_quantity = getattr(measured_derivative, "quantity", None)
    if not isinstance(measured_quantity, WorldQuantity):
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative must reference a derivative quantity.",
            measured_derivative,
        )
    measured_quantity = _resolved_world_quantity(measured_quantity)
    if measured_quantity.type != WorldQuantityType.VelocityTwist:
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative must reference a VelocityTwist for pose damping.",
            measured_derivative,
        )

    # This is the pose-diff measured_derivative check, not ACHD acceleration-constraint
    # validation (that's `_achd_acceleration_axis_keys` below) - translate to the
    # neutral pose-diff vocabulary before testing (D4).
    required_subspaces = {
        component.part for component in pose_diff_components(command.acceleration_constraints)
    }
    measured_subspace = _authored_subspace(measured_derivative)
    if measured_subspace is not None and measured_subspace not in required_subspaces:
        expected = ", ".join(sorted(required_subspaces))
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative subspace must match {expected}.",
            measured_derivative,
        )
    if len(required_subspaces) > 1 and measured_subspace is not None:
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative must reference the whole derivative quantity for full-pose damping.",
            measured_derivative,
        )

    required_axes = {component.axis for component in command.acceleration_constraints}
    measured_axis = axis_label(getattr(measured_derivative, "axis", None))
    if measured_axis is not None and (len(required_axes) != 1 or measured_axis not in required_axes):
        raise semantic_error(
            f"Controller '{controller.name}' measured_derivative axis must match the controlled axis.",
            measured_derivative,
        )


def validate_controller_commands(model: Model) -> None:
    for handler in constraint_handlers(model):
        profiled_constraints: dict[int, str] = {}
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            params = resolved_controller.params
            if params.duplicate_terms:
                duplicates = ", ".join(params.duplicate_terms)
                raise semantic_error(
                    f"Controller '{controller.name}' repeats parameter term(s): {duplicates}.",
                    controller,
                )
            if resolved_controller.type == ControllerType.ABAG:
                raise semantic_error(
                    f"Controller '{controller.name}' uses ABAG, but ABAG is not implemented yet.",
                    controller,
                )
            if resolved_controller.type == ControllerType.PID:
                missing_gains = [
                    name for name, val in (("Kp", params.kp), ("Ki", params.ki), ("Kd", params.kd))
                    if val is None
                ]
                if missing_gains:
                    raise semantic_error(
                        f"PID controller '{controller.name}' must specify all of Kp, Ki, and Kd; "
                        f"missing: {missing_gains}.",
                        controller,
                    )
                if params.has_impedance_terms:
                    raise semantic_error(
                        f"PID controller '{controller.name}' cannot use Stiffness or Damping terms.",
                        controller,
                    )
                _validate_pid_measured_derivative(controller, resolved_controller)
                profile_qty = _profile_quantity(resolved_controller)
                if params.profile is not None:
                    if profile_qty is None:
                        raise semantic_error(
                            f"Controller '{controller.name}' profile must reference a VelocityProfile.",
                            params.profile,
                        )
                    if profile_qty.type != ReferenceGeneratorType.VelocityProfile or not isinstance(profile_qty.value, ProfileSpec):
                        raise semantic_error(
                            f"Controller '{controller.name}' profile must reference a VelocityProfile.",
                            params.profile,
                        )
                    spec = params.constraint.constraint
                    if not isinstance(spec.expr, EqualityConstraint):
                        raise semantic_error(
                            f"Controller '{controller.name}' profile requires a distance equality constraint.",
                            controller,
                        )
                    if not _supports_velocity_profile(spec):
                        raise semantic_error(
                            f"Controller '{controller.name}' profile is only supported for distance constraints.",
                            controller,
                        )
                    if _requires_measured_velocity(spec) and profile_qty.value.measured_velocity is None:
                        raise semantic_error(
                            f"Controller '{controller.name}' profile must specify measured_velocity for pose-axis constraints.",
                            params.profile,
                        )
                    spec_id = id(spec)
                    if spec_id in profiled_constraints:
                        raise semantic_error(
                            f"Constraint '{spec.name}' has multiple profiled controllers: "
                            f"{profiled_constraints[spec_id]}, {controller.name}.",
                            controller,
                        )
                    profiled_constraints[spec_id] = controller.name
            elif resolved_controller.type == ControllerType.Impedance:
                if params.measured_derivative is not None:
                    raise semantic_error(
                        f"Controller '{controller.name}' measured_derivative is only supported on PID controllers.",
                        controller,
                    )
                if params.profile is not None:
                    raise semantic_error(
                        f"Controller '{controller.name}' profile is only supported on PID controllers.",
                        controller,
                    )
                if not params.has_impedance_terms:
                    raise semantic_error(
                        f"Impedance controller '{controller.name}' must author Stiffness, Damping, or both.",
                        controller,
                    )
                disallowed_gains = [
                    name for name, val in (("Kp", params.kp), ("Kd", params.kd))
                    if val is not None
                ]
                if disallowed_gains:
                    raise semantic_error(
                        f"Impedance controller '{controller.name}' cannot use "
                        f"{', '.join(disallowed_gains)} terms.",
                        controller,
                    )
            elif resolved_controller.type == ControllerType.FeedForward:
                if params.measured_derivative is not None:
                    raise semantic_error(
                        f"Controller '{controller.name}' measured_derivative is only supported on PID controllers.",
                        controller,
                    )
                if params.profile is not None:
                    raise semantic_error(
                        f"Controller '{controller.name}' profile is only supported on PID controllers.",
                        controller,
                    )
                if params.has_pid_gains or params.has_impedance_terms:
                    raise semantic_error(
                        f"FeedForward controller '{controller.name}' cannot use PID or impedance terms.",
                        controller,
                    )
                if not isinstance(params.constraint.constraint.expr, EqualityConstraint):
                    raise semantic_error(
                        f"FeedForward controller '{controller.name}' must reference an equality constraint.",
                        controller,
                    )

            if resolved_controller.type not in (
                ControllerType.PID,
                ControllerType.Impedance,
                ControllerType.FeedForward,
            ):
                raise semantic_error(
                    f"Controller '{controller.name}' uses {resolved_controller.type.value}, "
                    "which is not supported for graph emission.",
                    controller,
                )
            # Remaining validation applies to controllers that map a scalar error to a scalar control signal.
            constraint_spec = resolved_controller.params.constraint.constraint
            subspace = constraint_spec.view.subspace
            # `distance between <A> and <B>` has no raw SubSpace (subspace is None);
            # resolve through constraint_view_subspace ("distance") so it infers a
            # linear command without requiring an explicit 'as', like SubSpace.Position.
            command_type = (
                resolved_controller.command_type
                or infer_command_type(subspace)
                or infer_command_type(constraint_view_subspace(constraint_spec))
            )
            if resolved_controller.type == ControllerType.Impedance and command_type != QuantityType.Force:
                command_type = QuantityType.Force
            quantity = constraint_spec.view.quantity
            solver = handler_controller_solver(handler, controller)
            if resolved_controller.type == ControllerType.FeedForward:
                if solver is None:
                    raise semantic_error(
                        f"FeedForward controller '{controller.name}' must resolve to a solver.",
                        controller,
                    )
                if str(solver.algorithm) == "CommandForwarding":
                    quantity_props = getattr(quantity, "props", None)
                    if quantity_props is None or not any(
                        getattr(pair, "key", None) == "of" and getattr(pair, "value", "")
                        for pair in getattr(quantity_props, "pairs", [])
                    ):
                        raise semantic_error(
                            f"FeedForward controller '{controller.name}' via CommandForwarding needs target 'of'.",
                            controller,
                        )
                    continue
            explicit_command_type = resolved_controller.command_type
            whole_pose_command = (
                isinstance(quantity, WorldQuantity)
                and _resolved_world_quantity(quantity).type == WorldQuantityType.Pose
                and subspace is None
            )
            if explicit_command_type is not None and explicit_command_type != QuantityType.Force:
                explicit_torque_joint = (
                    explicit_command_type == QuantityType.Torque
                    and isinstance(quantity, WorldQuantity)
                    and quantity.type == WorldQuantityType.JointPosition
                )
                if not explicit_torque_joint:
                    raise semantic_error(
                        f"Controller '{controller.name}' uses unsupported explicit command "
                        f"'as {explicit_command_type.value}'. Only 'as Force' is supported; "
                        "'as Torque' is only supported for JointPosition controllers.",
                        controller,
                    )
            if isinstance(quantity, WorldQuantity) and quantity.type == WorldQuantityType.JointPosition:
                if command_type != QuantityType.Torque:
                    raise semantic_error(
                        f"Controller '{controller.name}' targets JointPosition and must use 'as Torque'.",
                        controller,
                    )
                quantity_props = getattr(quantity, "props", None)
                if quantity_props is None or not any(
                    getattr(pair, "key", None) == "of" and getattr(pair, "value", "")
                    for pair in getattr(quantity_props, "pairs", [])
                ):
                    raise semantic_error(
                        f"Controller '{controller.name}' targets JointPosition and needs explicit 'of'.",
                        controller,
                    )
            if command_type is None and not whole_pose_command:
                raise semantic_error(
                    f"Controller '{controller.name}' requires explicit 'as' for "
                    f"constraint subspace '{subspace}'.",
                    controller,
                )
            if command_type == QuantityType.Force and resolved_controller.apply_at is None:
                raise semantic_error(
                    f"Controller '{controller.name}' produces Force and must specify 'apply at <link>'.",
                    controller,
                )
            if (
                resolved_controller.apply_at is not None
                and resolved_controller.apply_at.type != WorldEntityType.Link
            ):
                raise semantic_error(
                    f"Controller '{controller.name}' apply at target must be a Link.",
                    controller,
                )


def validate_achd_acceleration_constraints(model: Model) -> None:
    for handler in constraint_handlers(model):
        axes_by_solver: dict[int, dict[tuple[str, str], list[str]]] = defaultdict(
            lambda: defaultdict(list)
        )
        solver_by_id = {}
        for controller in handler.controllers:
            solver = handler_controller_solver(handler, controller)
            if solver is None or str(solver.algorithm) != "ACHD":
                continue
            sid = id(solver)
            solver_by_id[sid] = solver
            resolved_controller = _resolved_controller(controller)
            for axis_key in _achd_acceleration_axis_keys(controller):
                axes_by_solver[sid][axis_key].append(resolved_controller.name)

        for sid, axes in axes_by_solver.items():
            duplicate_axes = {
                axis: controllers
                for axis, controllers in axes.items()
                if len(controllers) > 1
            }
            if duplicate_axes:
                solver = solver_by_id[sid]
                axis, controllers = next(iter(duplicate_axes.items()))
                axis_name = ".".join(axis)
                controller_names = ", ".join(controllers)
                raise semantic_error(
                    f"ACHD solver '{solver.name}' in handler '{handler.name}' has multiple "
                    f"acceleration constraints for Cartesian axis '{axis_name}': "
                    f"{controller_names}. Multiple constraints on the same Cartesian axis "
                    "are not supported yet.",
                    handler,
                )
            if len(axes) > 6:
                solver = solver_by_id[sid]
                raise semantic_error(
                    f"ACHD solver '{solver.name}' in handler '{handler.name}' has "
                    f"{len(axes)} unique acceleration constraints, but KDL "
                    "ChainHdSolver_Vereshchagin supports at most 6.",
                    handler,
                )
