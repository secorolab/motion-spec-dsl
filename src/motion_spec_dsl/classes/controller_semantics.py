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
    SolverAlias,
)
from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    BilateralConstraint,
    ConstraintSpecification,
    EqualityConstraint,
    LessThanConstraint,
    ViewForm,
    view_form,
)
from motion_spec_dsl.classes.context import (
    BODY_LINE_DISTANCE_OP,
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_DISTANCE_SUBSPACE,
    GEOMETRIC_PROJECTION_OPS,
    JOINT_SCALAR_TYPES,
    AngleBetweenView,
    ContextQuantityAlias,
    QuantityType,
    SubSpace,
    VectorXYZ,
    WorldQuantity,
    WorldQuantityAlias,
    WorldQuantityType,
    geometric_operand_kind,
    is_body_line_distance,
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

# The subspace a view of a whole quantity reads, by the quantity's kind.
WHOLE_QUANTITY_SUBSPACES = {
    WorldQuantityType.JointPosition: "joint-position",
    WorldQuantityType.JointVelocity: "joint-velocity",
    WorldQuantityType.JointForce: "joint-force",
    WorldQuantityType.Pose: "pose",
}

COMMAND_TYPES = {
    SubSpace.LinVel: QuantityType.LinearVelocity,
    SubSpace.Position: QuantityType.LinearVelocity,
    SubSpace.AngVel: QuantityType.AngularVelocity,
    SubSpace.Orientation: QuantityType.AngularVelocity,
    SubSpace.Force: QuantityType.Force,
    SubSpace.Torque: QuantityType.Torque,
}

# A whole `.orientation` view keeps its raw token; only the per-axis form becomes "rotation".
ANGULAR_SUBSPACES = {
    "orientation",
    "rotation",
    "angular",
    "angular-velocity",
    "alignment",
    "incident-angle",
    "plane-angle",
}

LINEAR_AXES = tuple(("linear", axis) for axis in "xyz")
ANGULAR_AXES = tuple(("angular", axis) for axis in "xyz")
POSE_AXES = (*LINEAR_AXES, *ANGULAR_AXES)


@dataclass(frozen=True)
class ControllerCommandRecord:
    """A controller's command: the quantity and view it drives, its type, and its axes."""

    controller: ControllerEntry
    constraint: ConstraintSpecification
    quantity: WorldQuantity | None
    view_subspace: str | None
    axis: str | None
    command_type: QuantityType | None
    controlled_axes: tuple[tuple[str, str], ...]
    is_force_command: bool
    is_posture_torque_command: bool
    # A couple about an axis, not a joint torque; an angle view names no world quantity.
    is_moment_command: bool


def alignment_is_pointwise(constraint: ConstraintSpecification) -> bool:
    """Whether an angle target names the direction itself (two rotational DOF), not a cone (one).

    Equality to a bare zero, a band opening at zero, and `less than` all name the direction.
    """
    expr = constraint.expr
    if isinstance(expr, EqualityConstraint):
        return expr.reference.bare is not None and expr.reference.bare.value == 0.0
    if isinstance(expr, LessThanConstraint):
        return True
    if isinstance(expr, BilateralConstraint):
        return expr.lower.bare is not None and expr.lower.bare.value == 0.0
    return False


def controller_solver(handler: ConstraintHandler, controller: ControllerEntry | ControllerAlias):
    """The solver a controller names, else the handler's one serial-chain or forwarding solver.

    A mobile-platform solver is never an implicit target: it consumes a computed signal.
    """
    resolved = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
    if resolved.solver is not None:
        explicit = resolved.solver.solver
        return explicit.ref.solver if isinstance(explicit, SolverAlias) else explicit
    solvers = [
        solver
        for item in handler.solvers
        if isinstance(
            solver := item.ref.solver if isinstance(item, SolverAlias) else item,
            (SerialChainSolver, CommandForwardingSolver),
        )
    ]
    if len(solvers) == 1:
        return solvers[0]
    forwarding = resolved.type == ControllerType.FeedForward
    candidates = [
        solver for solver in solvers if isinstance(solver, CommandForwardingSolver) == forwarding
    ]
    return candidates[0] if len(candidates) == 1 else None


def infer_command_type(subspace: SubSpace | str | None) -> QuantityType | None:
    """The command type a view subspace implies: linear or angular velocity, force, or torque."""
    # A distance and every Table II expression is a length, so its command is linear.
    if subspace == "distance" or subspace in GEOMETRIC_DISTANCE_SUBSPACE.values():
        return QuantityType.LinearVelocity
    return COMMAND_TYPES.get(subspace)


def constraint_view_subspace(constraint: ConstraintSpecification) -> str | None:
    """The constraint's canonical distance, angle, joint, or pose subspace."""
    view = constraint.view
    form = view_form(constraint)
    if form in ANGLE_VIEW_FORMS:
        return str(form)
    if form == ViewForm.DistanceBetween:
        return "distance"
    if form in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
        binary = view.binary
        kinds = (geometric_operand_kind(binary.left), geometric_operand_kind(binary.right))
        if form == ViewForm.ProjectionOn:
            op_type = GEOMETRIC_PROJECTION_OPS.get(kinds)
        elif GEOMETRIC_DISTANCE_OPS.get(kinds) == "PointLineToLinearDistance" and (
            is_body_line_distance(binary)
        ):
            op_type = BODY_LINE_DISTANCE_OP
        else:
            op_type = GEOMETRIC_DISTANCE_OPS.get(kinds)
        return GEOMETRIC_DISTANCE_SUBSPACE.get(op_type) if op_type else None
    quantity = view.quantity
    if view.subspace is None:
        if isinstance(quantity, WorldQuantity):
            return WHOLE_QUANTITY_SUBSPACES.get(quantity.type)
        return None
    raw = str(view.subspace)
    whole_pose_part = (
        isinstance(quantity, WorldQuantity)
        and quantity.type == WorldQuantityType.Pose
        and raw in {"position", "orientation"}
        and view.axis is None
    )
    return raw if whole_pose_part else SUBSPACE_ALIAS.get(raw, raw)


def resolved_constraint_quantity(constraint: ConstraintSpecification) -> WorldQuantity | None:
    """The world quantity a constraint's view reads, aliases resolved, or None."""
    quantity = constraint.view.quantity if constraint.view is not None else None
    if isinstance(quantity, WorldQuantityAlias):
        return quantity.ref
    return quantity if isinstance(quantity, WorldQuantity) else None


def controller_command_record(
    controller: ControllerEntry | ControllerAlias,
) -> ControllerCommandRecord:
    """A controller's command type and the Cartesian axes it controls."""
    resolved = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
    constraint = resolved.params.constraint.constraint
    quantity = resolved_constraint_quantity(constraint)
    raw_subspace = constraint.view.subspace
    axis = constraint.view.axis
    view_subspace = constraint_view_subspace(constraint)
    # A distance view has no raw subspace: the resolved "distance" still infers a linear command.
    command_type = (
        resolved.command_type
        or infer_command_type(raw_subspace)
        or infer_command_type(view_subspace)
    )
    # An impedance law commands through f_ext; the constrained subspace picks force or moment.
    if resolved.type == ControllerType.Impedance:
        command_type = (
            QuantityType.Torque if view_subspace in ANGULAR_SUBSPACES else QuantityType.Force
        )
    force_command = command_type == QuantityType.Force or view_subspace == "force"
    joint_quantity = quantity is not None and quantity.type in JOINT_SCALAR_TYPES
    posture_torque = command_type == QuantityType.Torque and joint_quantity
    moment_command = command_type == QuantityType.Torque and (
        view_subspace in ("alignment", "incident-angle", "plane-angle")
        or (quantity is not None and not joint_quantity)
    )
    controlled_axes: tuple[tuple[str, str], ...] = ()
    if not (force_command or posture_torque):
        if (
            quantity is not None
            and quantity.type == WorldQuantityType.Pose
            and raw_subspace is None
            and isinstance(constraint.expr, EqualityConstraint)
        ):
            controlled_axes = POSE_AXES
        elif raw_subspace in {SubSpace.Position, SubSpace.LinVel}:
            controlled_axes = (("linear", axis),) if axis is not None else LINEAR_AXES
        elif raw_subspace in {SubSpace.Orientation, SubSpace.AngVel}:
            controlled_axes = (("angular", axis),) if axis is not None else ANGULAR_AXES
        elif view_subspace == "alignment":
            controlled_axes = ANGULAR_AXES
            # The reference direction's own axis spins freely: its rotation-vector component is zero.
            binary = constraint.view.binary
            if isinstance(binary, AngleBetweenView):
                reference = binary.right
                if isinstance(reference, ContextQuantityAlias):
                    reference = reference.ref
                if isinstance(reference.value, VectorXYZ) and len(reference.value.coords.values) == 3:
                    values = reference.value.coords.values
                    free = max(range(3), key=lambda i: abs(float(values[i].value)))
                    controlled_axes = tuple(
                        ("angular", name) for index, name in enumerate("xyz") if index != free
                    )
        elif view_subspace == "distance" and raw_subspace is None:
            controlled_axes = (("linear", "distance"),)
    return ControllerCommandRecord(
        controller=resolved,
        constraint=constraint,
        quantity=quantity,
        view_subspace=view_subspace,
        axis=str(axis) if axis is not None else None,
        command_type=command_type,
        controlled_axes=controlled_axes,
        is_force_command=force_command,
        is_posture_torque_command=posture_torque,
        is_moment_command=moment_command,
    )
