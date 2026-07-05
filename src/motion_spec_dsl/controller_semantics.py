# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Derived controller semantics shared by RDF generation and validation."""

from __future__ import annotations

from dataclasses import dataclass

from motion_spec_dsl.domain import (
    ConstraintSpecification,
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    EqualityConstraint,
    QuantityType,
    SubSpace,
    WorldQuantity,
    WorldQuantityType,
    _resolved_controller,
    _resolved_world_quantity,
)


SUBSPACE_ALIAS: dict[str, str] = {
    "angacc": "angular-acceleration",
    "angvel": "angular",
    "linacc": "linear-acceleration",
    "linvel": "linear",
    "orientation": "rotation",
    "position": "distance",
    "force": "force",
    "torque": "torque",
}


@dataclass(frozen=True)
class AccelerationConstraintRecord:
    """One ACHD acceleration-constraint axis: a linear/angular acceleration subspace
    and its x/y/z axis."""

    subspace: str
    axis: str

    @property
    def suffix(self) -> str:
        """Compact id suffix, `lin-<axis>` or `ang-<axis>`."""
        prefix = {
            "linear-acceleration": "lin",
            "angular-acceleration": "ang",
        }[self.subspace]
        return f"{prefix}-{self.axis}"

    @property
    def quantity_type(self) -> QuantityType:
        """The matching linear/angular AccelerationTwist QuantityType."""
        return {
            "linear-acceleration": QuantityType.LinearAcceleration,
            "angular-acceleration": QuantityType.AngularAcceleration,
        }[self.subspace]


@dataclass(frozen=True)
class PoseDiffComponentRecord:
    """Neutral linear/angular axis component for the pose-diff evaluator/controller
    path. Distinct from `AccelerationConstraintRecord`: a pose-diff component is a
    position/orientation error term (Length/Angle), not an acceleration, even though
    it is derived from the same ACHD acceleration-constraint axis enumeration."""

    part: str  # "linear" | "angular"
    axis: str

    @property
    def suffix(self) -> str:
        """Compact id suffix `lin-<axis>`/`ang-<axis>`.

        Must equal AccelerationConstraintRecord.suffix: the pose-diff controller's
        energy/control-signal node and the ACHD acceleration-energy node share this URI.
        """
        prefix = {"linear": "lin", "angular": "ang"}[self.part]
        return f"{prefix}-{self.axis}"


def pose_diff_components(
    acceleration_constraints: tuple["AccelerationConstraintRecord", ...],
) -> tuple[PoseDiffComponentRecord, ...]:
    """Translate shared ACHD acceleration-constraint records into the neutral
    linear/angular vocabulary the pose-diff path actually means."""
    part_by_subspace = {"linear-acceleration": "linear", "angular-acceleration": "angular"}
    return tuple(
        PoseDiffComponentRecord(part_by_subspace[record.subspace], record.axis)
        for record in acceleration_constraints
    )


@dataclass(frozen=True)
class ControllerCommandRecord:
    """Resolved control-command shape for a controller+constraint: the commanded
    quantity, its view subspace/axis, the command type, and the acceleration-constraint
    axes the command expands into."""

    controller: ControllerEntry
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    view_subspace: str | None
    axis: str | None
    command_type: QuantityType | None
    acceleration_constraints: tuple[AccelerationConstraintRecord, ...]

    @property
    def is_force_command(self) -> bool:
        """Whether this commands a force (by command type or a force view)."""
        return self.command_type == QuantityType.Force or self.view_subspace == "force"

    @property
    def is_posture_torque_command(self) -> bool:
        """Whether this commands a joint-space posture torque."""
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
    """Map a roll/pitch/yaw (or already-x/y/z) axis token to `x`/`y`/`z`."""
    if axis is None:
        return None
    raw = str(getattr(axis, "value", axis))
    return {
        "roll": "x",
        "pitch": "y",
        "yaw": "z",
    }.get(raw, raw)


def infer_command_type(subspace: SubSpace | str | None) -> QuantityType | None:
    """The command QuantityType (linear/angular velocity, force or torque) implied by a
    view subspace, or None.
    """
    if subspace is None:
        return None
    # A `distance between <A> and <B>` view has no raw SubSpace enum (it resolves
    # via constraint_view_subspace to the string "distance"); it is control-wise a
    # linear command, the same as SubSpace.Position.
    if subspace == "distance":
        return QuantityType.LinearVelocity
    return {
        SubSpace.LinVel: QuantityType.LinearVelocity,
        SubSpace.Position: QuantityType.LinearVelocity,
        SubSpace.AngVel: QuantityType.AngularVelocity,
        SubSpace.Orientation: QuantityType.AngularVelocity,
        SubSpace.Force: QuantityType.Force,
        SubSpace.Torque: QuantityType.Torque,
    }.get(subspace)


def constraint_view_subspace(constraint: ConstraintSpecification) -> str | None:
    """The constraint's controlled subspace as a canonical string, unwrapping `Norm of` and
    resolving distance / joint-position / pose views. None if it has no subspace.
    """
    view = constraint.view
    # `Norm of <inner>`: subspace is the reduced view's; unwrap for downstream callers.
    while getattr(view, "norm_source", None) is not None:
        view = view.norm_source

    if (
        getattr(view, "distance_from", None) is not None
        and getattr(view, "distance_to", None) is not None
    ):
        return "distance"

    subspace = view.subspace
    if subspace is None:
        quantity = view.quantity
        if isinstance(quantity, WorldQuantity):
            quantity = _resolved_world_quantity(quantity)
            if quantity.type == WorldQuantityType.JointPosition:
                return "joint-position"
            if quantity.type == WorldQuantityType.Pose:
                return "pose"
        return None

    raw = str(getattr(subspace, "value", subspace))
    quantity = view.quantity
    if (
        isinstance(quantity, WorldQuantity)
        and _resolved_world_quantity(quantity).type == WorldQuantityType.Pose
        and raw in {"position", "orientation"}
        and view.axis is None
    ):
        return raw
    return SUBSPACE_ALIAS.get(raw, raw)


def resolved_constraint_quantity(
    constraint: ConstraintSpecification,
) -> WorldQuantity | None:
    """The resolved WorldQuantity a constraint's view targets, or None."""
    quantity = getattr(constraint.view, "quantity", None)
    if isinstance(quantity, WorldQuantity):
        return _resolved_world_quantity(quantity)
    return None


def controller_command_record(
    controller: ControllerEntry | ControllerAlias,
) -> ControllerCommandRecord:
    """Derive a controller's command record: resolve its command type (impedance forces a
    Force command) and the acceleration-constraint axes it drives -- whole-pose 6D, a
    single axis, or one distance-direction constraint.
    """
    resolved_controller = _resolved_controller(controller)
    constraint = resolved_controller.params.constraint.constraint
    quantity = resolved_constraint_quantity(constraint)
    raw_subspace = constraint.view.subspace
    axis = axis_label(getattr(constraint.view, "axis", None))
    view_subspace = constraint_view_subspace(constraint)
    # `raw_subspace` is None for a `distance between <A> and <B>` view (no SubSpace enum); fall
    # back to the resolved string subspace ("distance") so it still infers a linear command.
    command_type = (
        resolved_controller.command_type
        or infer_command_type(raw_subspace)
        or infer_command_type(view_subspace)
    )
    if resolved_controller.type == ControllerType.Impedance and command_type != QuantityType.Force:
        command_type = QuantityType.Force

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
        elif view_subspace == "distance" and raw_subspace is None:
            # `distance between <A> and <B>` (never the `.position.x` axis alias): a single
            # direction-aligned linear acceleration constraint, driven by the runtime direction.
            acceleration_constraints = (AccelerationConstraintRecord("linear-acceleration", "distance"),)

    return ControllerCommandRecord(
        controller=resolved_controller,
        constraint=constraint,
        quantity=quantity,
        view_subspace=view_subspace,
        axis=axis,
        command_type=command_type,
        acceleration_constraints=acceleration_constraints,
    )
