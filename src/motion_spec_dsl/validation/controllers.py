# SPDX-License-Identifier: MPL-2.0
"""Controller command and solver-interface validation."""

from __future__ import annotations

from collections import defaultdict

from motion_spec_dsl.domain import (
    ControllerAlias,
    ControllerEntry,
    ControllerMode,
    Model,
    QuantityType,
    WorldQuantity,
    WorldQuantityType,
    _resolved_controller,
    _resolved_world_quantity,
)
from motion_spec_dsl.semantics import controller_command_record, infer_command_type
from motion_spec_dsl.validation.common import constraint_handlers, semantic_error
from motion_spec_dsl.validation.solvers import handler_controller_solver


def _achd_acceleration_axis_keys(
    controller: ControllerEntry | ControllerAlias,
) -> list[tuple[str, str]]:
    command = controller_command_record(controller)
    return [(record.subspace, record.axis) for record in command.acceleration_constraints]


def validate_controller_commands(model: Model) -> None:
    for handler in constraint_handlers(model):
        for controller in handler.controllers:
            resolved_controller = _resolved_controller(controller)
            constraint_spec = resolved_controller.params.constraint.constraint
            subspace = constraint_spec.view.subspace
            command_type = resolved_controller.command_type or infer_command_type(subspace)
            quantity = constraint_spec.view.quantity
            whole_pose_command = (
                isinstance(quantity, WorldQuantity)
                and _resolved_world_quantity(quantity).type == WorldQuantityType.Pose
                and subspace is None
            )
            if command_type is None and not whole_pose_command:
                raise semantic_error(
                    f"Controller '{controller.name}' requires explicit 'as' for "
                    f"constraint subspace '{subspace}'.",
                    controller,
                )
            if (
                isinstance(quantity, WorldQuantity)
                and quantity.type == WorldQuantityType.JointPosition
                and resolved_controller.control_mode != ControllerMode.Posture
            ):
                raise semantic_error(
                    f"Controller '{controller.name}' targets JointPosition and must declare "
                    "'for Posture'.",
                    controller,
                )
            if resolved_controller.control_mode == ControllerMode.Posture:
                if command_type != QuantityType.Torque:
                    raise semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must use 'as Torque'.",
                        controller,
                    )
                if not isinstance(quantity, WorldQuantity) or quantity.type != WorldQuantityType.JointPosition:
                    raise semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must target a JointPosition constraint.",
                        controller,
                    )
                quantity_props = getattr(quantity, "props", None)
                if quantity_props is None or not any(
                    getattr(pair, "key", None) == "of" and getattr(pair, "value", "")
                    for pair in getattr(quantity_props, "pairs", [])
                ):
                    raise semantic_error(
                        f"Controller '{controller.name}' with 'for Posture' must target a JointPosition with explicit 'of'.",
                        controller,
                    )
            if (
                resolved_controller.apply_at is not None
                and resolved_controller.apply_at.type != WorldQuantityType.Link
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
