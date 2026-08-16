# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Controller and solver semantics derived from the parsed DSL model."""

from __future__ import annotations

from dataclasses import dataclass

from motion_spec_dsl.classes.constraint_handler import (
    CommandForwardingSolver,
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerType,
    SerialChainSolver,
    _resolved_controller,
    _resolved_solver,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintSpecification,
    EqualityConstraint,
)
from motion_spec_dsl.classes.context import (
    QuantityType,
    SubSpace,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
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
class ControllerCommandRecord:
    """Resolved control-command shape for a controller+constraint: the commanded
    quantity, its view subspace/axis, command type, and controlled Cartesian axes."""

    controller: ControllerEntry
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    view_subspace: str | None
    axis: str | None
    command_type: QuantityType | None
    controlled_axes: tuple[tuple[str, str], ...]

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

    @property
    def is_moment_command(self) -> bool:
        """Whether this commands a Cartesian moment (a couple about an axis), not a joint torque.

        An alignment view names two directions rather than a world quantity, so it has none to
        classify by; its command is a couple about the axes it drives.
        """
        if self.command_type != QuantityType.Torque:
            return False
        if self.view_subspace == "alignment":
            return True
        return self.quantity is not None and self.quantity.type != WorldQuantityType.JointPosition


# Resolved view subspaces whose command is a moment, not a force. A whole `.orientation` view
# keeps its raw token; only the per-axis form aliases to "rotation".
ANGULAR_SUBSPACES = frozenset(
    {"orientation", "rotation", "angular", "angular-velocity", "alignment"}
)

LINEAR_AXES = tuple(("linear", axis) for axis in "xyz")
ANGULAR_AXES = tuple(("angular", axis) for axis in "xyz")
POSE_AXES = (*LINEAR_AXES, *ANGULAR_AXES)


def _alignment_controlled_axes(
    constraint: ConstraintSpecification,
) -> tuple[tuple[str, str], ...]:
    """The angular axes an alignment drives: every axis but the reference direction's own, whose
    rotation-vector component is identically zero -- that zero is the free spin about the axis.
    """
    reference = getattr(constraint.view, "angle_to", None)
    if reference is None:
        return ANGULAR_AXES
    coords = getattr(getattr(_resolved_context_quantity(reference), "value", None), "coords", None)
    if coords is None or len(coords.values) != 3:
        return ANGULAR_AXES
    free = max(range(3), key=lambda i: abs(float(coords.values[i].value)))
    return tuple(("angular", axis) for index, axis in enumerate("xyz") if index != free)


def controller_solver(handler: ConstraintHandler, controller: ControllerEntry | ControllerAlias):
    """Resolve the authored, sole, or uniquely role-compatible solver for a controller.

    Only serial-chain dynamics and command-forwarding solvers bind implicitly to a
    controller; a mobile-platform velocity/force solver is never an implicit controller
    target -- it consumes an already-computed context signal, not a controller output.
    """
    resolved = _resolved_controller(controller)
    explicit = getattr(getattr(resolved, "solver", None), "solver", None)
    if explicit is not None:
        return _resolved_solver(explicit)
    solvers = [
        _resolved_solver(item)
        for item in handler.solvers
        if isinstance(_resolved_solver(item), (SerialChainSolver, CommandForwardingSolver))
    ]
    if len(solvers) == 1:
        return solvers[0]
    forwarding = resolved.type == ControllerType.FeedForward
    candidates = [
        solver for solver in solvers if isinstance(solver, CommandForwardingSolver) == forwarding
    ]
    return candidates[0] if len(candidates) == 1 else None


def axis_label(axis: object | None) -> str | None:
    """Normalise an axis token to its `x`/`y`/`z` string."""
    if axis is None:
        return None
    return str(getattr(axis, "value", axis))


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
    """The constraint's canonical distance, joint-position, or pose subspace."""
    view = constraint.view

    if (
        getattr(view, "angle_from", None) is not None
        and getattr(view, "angle_to", None) is not None
    ):
        return "alignment"

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
    """Resolve a controller's authored command type and controlled Cartesian axes."""
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
    # An impedance law commands through f_ext; the constrained subspace picks force vs moment.
    if resolved_controller.type == ControllerType.Impedance:
        command_type = (
            QuantityType.Torque if view_subspace in ANGULAR_SUBSPACES else QuantityType.Force
        )

    controlled_axes: tuple[tuple[str, str], ...] = ()
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
            controlled_axes = POSE_AXES
        elif raw_subspace in {SubSpace.Position, SubSpace.LinVel}:
            controlled_axes = (("linear", axis),) if axis is not None else LINEAR_AXES
        elif raw_subspace in {SubSpace.Orientation, SubSpace.AngVel}:
            controlled_axes = (("angular", axis),) if axis is not None else ANGULAR_AXES
        elif view_subspace == "alignment":
            controlled_axes = _alignment_controlled_axes(constraint)
        elif view_subspace == "distance" and raw_subspace is None:
            controlled_axes = (("linear", "distance"),)

    return ControllerCommandRecord(
        controller=resolved_controller,
        constraint=constraint,
        quantity=quantity,
        view_subspace=view_subspace,
        axis=axis,
        command_type=command_type,
        controlled_axes=controlled_axes,
    )
