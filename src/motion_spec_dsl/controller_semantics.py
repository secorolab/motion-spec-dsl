# SPDX-License-Identifier: MPL-2.0
"""Derived controller semantics shared by RDF generation and validation."""

from __future__ import annotations

from dataclasses import dataclass

from motion_spec_dsl.domain import (
    ConstraintSpecification,
    ControllerAlias,
    ControllerEntry,
    EqualityConstraint,
    QuantityType,
    SubSpace,
    WorldQuantity,
    WorldQuantityType,
    _resolved_controller,
    _resolved_world_quantity,
)


SUBSPACE_ALIAS: dict[str, str] = {
    "angvel": "angular",
    "linvel": "linear",
    "orientation": "rotation",
    "position": "distance",
    "force": "force",
    "torque": "torque",
}


@dataclass(frozen=True)
class AccelerationConstraintRecord:
    subspace: str
    axis: str

    @property
    def suffix(self) -> str:
        prefix = {
            "linear-acceleration": "lin",
            "angular-acceleration": "ang",
        }[self.subspace]
        return f"{prefix}-{self.axis}"

    @property
    def quantity_type(self) -> QuantityType:
        return {
            "linear-acceleration": QuantityType.LinearAcceleration,
            "angular-acceleration": QuantityType.AngularAcceleration,
        }[self.subspace]


@dataclass(frozen=True)
class ControllerCommandRecord:
    controller: ControllerEntry
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    view_subspace: str | None
    axis: str | None
    command_type: QuantityType | None
    acceleration_constraints: tuple[AccelerationConstraintRecord, ...]

    @property
    def is_force_command(self) -> bool:
        return self.command_type == QuantityType.Force or self.view_subspace == "force"

    @property
    def is_posture_torque_command(self) -> bool:
        return (
            self.command_type == QuantityType.Torque
            and self.quantity is not None
            and self.quantity.type == WorldQuantityType.JointPosition
        )


LINEAR_ACCELERATION_AXES: tuple[AccelerationConstraintRecord, ...] = (
    AccelerationConstraintRecord("linear-acceleration", "x"),
    AccelerationConstraintRecord("linear-acceleration", "y"),
    AccelerationConstraintRecord("linear-acceleration", "z"),
)

ANGULAR_ACCELERATION_AXES: tuple[AccelerationConstraintRecord, ...] = (
    AccelerationConstraintRecord("angular-acceleration", "x"),
    AccelerationConstraintRecord("angular-acceleration", "y"),
    AccelerationConstraintRecord("angular-acceleration", "z"),
)

POSE_ACCELERATION_AXES: tuple[AccelerationConstraintRecord, ...] = (
    *LINEAR_ACCELERATION_AXES,
    *ANGULAR_ACCELERATION_AXES,
)


def axis_label(axis: object | None) -> str | None:
    if axis is None:
        return None
    return str(getattr(axis, "value", axis))


def infer_command_type(subspace: SubSpace | None) -> QuantityType | None:
    if subspace is None:
        return None
    return {
        SubSpace.LinVel: QuantityType.LinearVelocity,
        SubSpace.Position: QuantityType.LinearVelocity,
        SubSpace.AngVel: QuantityType.AngularVelocity,
        SubSpace.Orientation: QuantityType.AngularVelocity,
        SubSpace.Force: QuantityType.Force,
        SubSpace.Torque: QuantityType.Torque,
    }.get(subspace)


def constraint_view_subspace(constraint: ConstraintSpecification) -> str | None:
    if (
        getattr(constraint.view, "distance_from", None) is not None
        and getattr(constraint.view, "distance_to", None) is not None
    ):
        return "distance"

    subspace = constraint.view.subspace
    if subspace is None:
        quantity = constraint.view.quantity
        if isinstance(quantity, WorldQuantity):
            quantity = _resolved_world_quantity(quantity)
            if quantity.type == WorldQuantityType.JointPosition:
                return "joint-position"
            if quantity.type == WorldQuantityType.Pose:
                return "pose"
        return None

    raw = str(getattr(subspace, "value", subspace))
    quantity = constraint.view.quantity
    if (
        isinstance(quantity, WorldQuantity)
        and _resolved_world_quantity(quantity).type == WorldQuantityType.Pose
        and raw in {"position", "orientation"}
        and constraint.view.axis is None
    ):
        return raw
    return SUBSPACE_ALIAS.get(raw, raw)


def resolved_constraint_quantity(
    constraint: ConstraintSpecification,
) -> WorldQuantity | None:
    quantity = getattr(constraint.view, "quantity", None)
    if isinstance(quantity, WorldQuantity):
        return _resolved_world_quantity(quantity)
    return None


def controller_command_record(
    controller: ControllerEntry | ControllerAlias,
) -> ControllerCommandRecord:
    resolved_controller = _resolved_controller(controller)
    constraint = resolved_controller.params.constraint.constraint
    quantity = resolved_constraint_quantity(constraint)
    raw_subspace = constraint.view.subspace
    axis = axis_label(getattr(constraint.view, "axis", None))
    view_subspace = constraint_view_subspace(constraint)
    command_type = resolved_controller.command_type or infer_command_type(raw_subspace)

    acceleration_constraints: tuple[AccelerationConstraintRecord, ...] = ()
    whole_pose_command = (
        quantity is not None
        and quantity.type == WorldQuantityType.Pose
        and raw_subspace is None
        and isinstance(constraint.expr, EqualityConstraint)
    )
    force_command = command_type == QuantityType.Force or view_subspace == "force"
    posture_torque = (
        command_type == QuantityType.Torque
        and quantity is not None
        and quantity.type == WorldQuantityType.JointPosition
    )
    if not (force_command or posture_torque):
        if whole_pose_command:
            acceleration_constraints = POSE_ACCELERATION_AXES
        elif raw_subspace in {SubSpace.Position, SubSpace.LinVel}:
            acceleration_constraints = (
                (AccelerationConstraintRecord("linear-acceleration", axis),)
                if axis is not None
                else LINEAR_ACCELERATION_AXES
            )
        elif raw_subspace in {SubSpace.Orientation, SubSpace.AngVel}:
            acceleration_constraints = (
                (AccelerationConstraintRecord("angular-acceleration", axis),)
                if axis is not None
                else ANGULAR_ACCELERATION_AXES
            )

    return ControllerCommandRecord(
        controller=resolved_controller,
        constraint=constraint,
        quantity=quantity,
        view_subspace=view_subspace,
        axis=axis,
        command_type=command_type,
        acceleration_constraints=acceleration_constraints,
    )
