# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""What a constraint's view resolves to: its scalar kind, its carrier quantity, and the frames it is in.

Validation and RDF emission both read these, so a check and the graph it guards agree.
"""

from __future__ import annotations

from typing import Any

from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    ConstraintSpecification,
    EqualityConstraint,
    ViewForm,
    view_form,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_PROJECTION_OPS,
    JOINT_SCALAR_TYPES,
    NORM_SCALAR_TYPES,
    POSE_SUBSPACE_SCALAR_TYPES,
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    DerivedScalarValue,
    GeometricPropKey,
    GeometricProps,
    GeoPropPair,
    QuantityLeaf,
    QuantityType,
    ReferenceGeneratorType,
    View,
    WorldQuantity,
    WorldQuantityAlias,
    WorldQuantityType,
    geo_prop,
    geo_prop_value,
    geometric_operand_kind,
    op_tree,
    path_pose_endpoints,
    pose_frame_names,
    quantity_axis_frame,
    scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import (
    alignment_is_pointwise,
    constraint_view_subspace,
)
from motion_spec_dsl.classes.dimensions import VECTOR_COMPONENT_TYPE, infer
from motion_spec_dsl.classes.motion_spec import ConstraintSection
from motion_spec_dsl.classes.path import PathValue

# One default band per unit family: an axis of a position and a distance are both metres.
TOLERANCE_DEFAULT_KIND = {
    QuantityType.Distance: QuantityType.Position,
    QuantityType.Angle: QuantityType.Orientation,
    QuantityType.PlaneAngle: QuantityType.Orientation,
}
# (whole part, one axis of it) kinds of a pose or path's position and orientation.
POSE_PART_KINDS = {
    "position": (QuantityType.Position, QuantityType.Distance),
    "orientation": (QuantityType.Orientation, QuantityType.Angle),
}
POSE_KINDS = {QuantityType.Pose, ReferenceGeneratorType.Path}
# The subspaces a component moves along a line in, and those it turns about an axis in.
LINEAR_PARTS = {"position", "linvel", "linacc", "force"}
ANGULAR_PARTS = {"orientation", "angvel", "angacc", "torque"}
# The kind of one subspace of a context twist or wrench.
COMPOSITE_PART_KINDS = {
    (QuantityType.VelocityTwist, "linvel"): QuantityType.LinearVelocity,
    (QuantityType.VelocityTwist, "angvel"): QuantityType.AngularVelocity,
    (QuantityType.AccelerationTwist, "linacc"): QuantityType.LinearAcceleration,
    (QuantityType.AccelerationTwist, "angacc"): QuantityType.AngularAcceleration,
    (QuantityType.Wrench, "force"): QuantityType.Force,
    (QuantityType.Wrench, "torque"): QuantityType.Torque,
}
_LINE_LINE_OPS = {"LineLineToLinearDistance", "LineOnLineProjection"}


def context_subspace_kind(quantity: ContextQuantity, subspace: str | None, axis: str | None) -> Any:
    """The kind a reference to SUBSPACE (and AXIS) of a context quantity reads, or None.

    With no subspace it is a 3-vector's component kind: only the axis tells the reference apart.
    """
    if subspace is None:
        return VECTOR_COMPONENT_TYPE.get(quantity.type)
    if quantity.type in POSE_KINDS and subspace in POSE_PART_KINDS:
        whole, component = POSE_PART_KINDS[subspace]
        return whole if axis is None else component
    return COMPOSITE_PART_KINDS.get((quantity.type, subspace))


def constraint_kind(constraint: ConstraintSpecification) -> Any:
    """The scalar kind a constraint's error is measured in; None for a goal status, which has no view.

    A path driver or guard measures a speed, a binary view a length or an angle, a world view the
    subspace it selects (a norm its length), a context view its subspace or its own kind, and an
    expression what it infers to.
    """
    view = constraint.view
    if view is None:
        return None
    if view.elapsed is not None:
        return QuantityType.Duration
    if view.moving is not None or view.progress is not None:
        return QuantityType.LinearVelocity
    form = view_form(constraint)
    subspace = constraint_view_subspace(constraint)
    axis = str(view.axis) if view.axis is not None else None
    if form is not None and form != ViewForm.Norm:
        return POSE_SUBSPACE_SCALAR_TYPES.get(subspace, subspace)
    quantity = view.quantity
    if isinstance(quantity, WorldQuantity):
        quantity = quantity.ref if isinstance(quantity, WorldQuantityAlias) else quantity
        if form == ViewForm.Norm:
            return NORM_SCALAR_TYPES[scalar_type(quantity, subspace, None)]
        return scalar_type(quantity, subspace, axis)
    if isinstance(quantity, ContextQuantity):
        quantity = quantity.ref if isinstance(quantity, ContextQuantityAlias) else quantity
        raw = str(view.subspace) if view.subspace is not None else None
        kind = context_subspace_kind(quantity, raw, axis)
        return kind if kind is not None and raw is not None else quantity.type
    return infer(op_tree(view.expr))


def resolve_world_quantity(ref, world_quantities: dict[str, WorldQuantity]) -> WorldQuantity | None:
    """REF itself when it is a world quantity, else the one in scope by its name."""
    if isinstance(ref, WorldQuantity):
        return ref
    return world_quantities.get(ref.name)


def distance_operand(ref, world_quantities: dict) -> WorldQuantity | ContextQuantity | None:
    """A distance endpoint: a world pose, or a context pose such as a snapshot, compared by value."""
    quantity = resolve_world_quantity(ref, world_quantities)
    if quantity is not None:
        return quantity
    return ref if isinstance(ref, ContextQuantity) else None


def existing_world_pose(world_quantities: dict, of_frame, wrt_frame) -> WorldQuantity | None:
    """The declared world pose of OF_FRAME wrt WRT_FRAME, or None.

    A view reads an already-computed pose: relating two frames is the world model's work.
    """
    return next(
        (
            quantity
            for quantity in world_quantities.values()
            if quantity.type == WorldQuantityType.Pose
            and geo_prop(quantity.props, "of") == of_frame
            and geo_prop(quantity.props, "wrt") == wrt_frame
        ),
        None,
    )


def owning_motion(constraint: ConstraintSpecification):
    """The motion whose section holds a constraint; for a derived scalar, the context declaring it."""
    if isinstance(constraint.parent, ContextQuantity):
        return constraint.parent.parent
    section = constraint.parent
    while not isinstance(section, ConstraintSection):
        section = section.parent
    return section.parent


def derived_view_constraint(quantity: ContextQuantity) -> ConstraintSpecification:
    """A spec scalar's binary view, held by a constraint so it resolves as a constraint's view does."""
    view = View(None, quantity.value.view, None, None, None, None, None, None, None, None)
    return ConstraintSpecification(quantity, False, quantity.name, view, None, None)


def leaf_gradient(leaf, world_quantities: dict) -> tuple[str, str] | None:
    """The (frame, half) a controlled expression's leaf moves along; None for one the motion does not move.

    A world quantity's component moves along its own axis, in the frame it is seen in; a derived
    scalar along the gradient its view publishes, in the frame its carrier is stated against. A
    constant, a snapshot or a configured value has no gradient. Raises ValueError for a moved leaf
    with no direction to state: a whole subspace, a joint, or a scalar whose view publishes none.
    """
    if isinstance(leaf, (QuantityLeaf, ContextRef)) and leaf.bare is not None:
        return None
    quantity = leaf.quantity
    if isinstance(quantity, ContextQuantityAlias):
        quantity = quantity.ref
    if isinstance(quantity, ContextQuantity):
        if not isinstance(quantity.value, DerivedScalarValue):
            return None
        derived = derived_view_constraint(quantity)
        form = view_form(derived)
        directed = form in (ViewForm.DistanceFrom, ViewForm.ProjectionOn) or (
            form in ANGLE_VIEW_FORMS and not alignment_is_pointwise(derived)
        )
        carrier = constraint_carrier(derived, world_quantities)
        if not directed or carrier is None:
            raise ValueError(f"'{quantity.name}' is a scalar whose view publishes no direction")
        half = "angular" if form in ANGLE_VIEW_FORMS else "linear"
        return geo_prop(carrier.props, "wrt"), half
    quantity = resolve_world_quantity(quantity, world_quantities)
    if quantity.type in JOINT_SCALAR_TYPES:
        raise ValueError(f"'{quantity.name}' is a joint, which moves in no Cartesian direction")
    subspace = str(leaf.subspace) if leaf.subspace is not None else None
    if leaf.axis is None or subspace not in LINEAR_PARTS | ANGULAR_PARTS:
        raise ValueError(f"'{quantity.name}' names a whole subspace; select one axis of it")
    return quantity_axis_frame(quantity), "linear" if subspace in LINEAR_PARTS else "angular"


def constraint_carrier(constraint: ConstraintSpecification, world_quantities: dict) -> WorldQuantity | None:
    """The world quantity a constraint acts on: the view's own, or the pose a binary view is stated as.

    A binary view's carrier is a pose named after its motion and constraint, in the frames its
    error and command resolve in; None for a clock, or an operand nothing in scope declares.
    """
    view = constraint.view
    if view.elapsed is not None:
        return None
    form = view_form(constraint)
    if form is None or form == ViewForm.Norm:
        return resolve_world_quantity(view.quantity, world_quantities) if view.quantity is not None else None
    binary = view.binary
    motion = owning_motion(constraint)
    if form == ViewForm.DistanceBetween:
        start = distance_operand(binary.left, world_quantities)
        end = distance_operand(binary.right, world_quantities)
        start_frame, end_frame = pose_frame_names(start)[0], pose_frame_names(end)[0]
        pairs = [
            GeoPropPair(None, GeometricPropKey.Of, frame=end_frame),
            GeoPropPair(None, GeometricPropKey.Wrt, frame=start_frame),
            GeoPropPair(None, GeometricPropKey.AsSeenBy, frame=start_frame),
        ]
        return WorldQuantity(motion, WorldQuantityType.Pose, f"distance-{motion.name}-{constraint.name}", GeometricProps(None, pairs))
    if form in ANGLE_VIEW_FORMS:
        # Only the first operand moves; a plane stands in by its normal.
        planes = (form == ViewForm.PlaneAngle, form != ViewForm.Alignment)
        frames = []
        for ref, is_plane in zip((binary.left, binary.right), planes):
            operand = ref.ref if isinstance(ref, ContextQuantityAlias) else ref
            direction = geo_prop_value(operand.props, GeometricPropKey.Normal) if is_plane else operand
            direction = direction.ref if isinstance(direction, ContextQuantityAlias) else direction
            frames.append(geo_prop(direction.props, "as-seen-by") or geo_prop(direction.props, "wrt"))
        return existing_world_pose(world_quantities, frames[0], frames[1])
    table = GEOMETRIC_DISTANCE_OPS if form == ViewForm.DistanceFrom else GEOMETRIC_PROJECTION_OPS
    op_type = table[(geometric_operand_kind(binary.left), geometric_operand_kind(binary.right))]
    stem = f"geo-distance-{motion.name}-{constraint.name}"
    primitive = binary.right.ref if isinstance(binary.right, ContextQuantityAlias) else binary.right
    if op_type in _LINE_LINE_OPS:
        line_a = binary.left.ref if isinstance(binary.left, ContextQuantityAlias) else binary.left
        direction = geo_prop_value(line_a.props, GeometricPropKey.Along)
        direction = direction.ref if isinstance(direction, ContextQuantityAlias) else direction
        frame = geo_prop(direction.props, "as-seen-by")
        pairs = [
            GeoPropPair(None, GeometricPropKey.Of, frame=geo_prop(primitive.props, "of")),
            GeoPropPair(None, GeometricPropKey.Wrt, frame=frame),
            GeoPropPair(None, GeometricPropKey.AsSeenBy, frame=frame),
        ]
        return WorldQuantity(motion, WorldQuantityType.Pose, stem, GeometricProps(None, pairs))
    point = distance_operand(binary.left, world_quantities)
    point_wrt = pose_frame_names(point)[1]
    primitive_frame = geo_prop(primitive.props, "of")
    # A line riding the point's measurement frame has that frame's origin, so no pose to read.
    if op_type == "PointLineToLinearDistance" and primitive_frame == point_wrt:
        return WorldQuantity(motion, WorldQuantityType.Pose, stem, point.props)
    origin = existing_world_pose(world_quantities, primitive_frame, point_wrt)
    return WorldQuantity(motion, WorldQuantityType.Pose, stem, origin.props) if origin is not None else None


def pose_frame_source(quantity: ContextQuantity, constraints: list, world_quantities: dict):
    """The (of, wrt, as-seen-by) names a literal pose stating none takes, and the quantity naming them.

    In this order: a path it ends, its relative base, a component reference, or the world pose an
    equality constraint compares it with. None when nothing states them.
    """
    value = quantity.value
    # (frame names, quantity naming them) candidates.
    sources = []
    for sibling in quantity.parent.declaration:
        if isinstance(sibling, ContextQuantity) and isinstance(sibling.value, PathValue):
            endpoints = path_pose_endpoints(sibling)
            if quantity in endpoints:
                sources += [(pose_frame_names(endpoint), endpoint) for endpoint in endpoints]
    relative = value.orientation.relative
    if relative is not None and relative.base.quantity is not None:
        sources.append((pose_frame_names(relative.base.quantity), relative.base.quantity))
    for ref in (value.position.ref, value.orientation.ref):
        if ref is not None and ref.quantity is not None:
            sources.append((pose_frame_names(ref.quantity), ref.quantity))
    for constraint in constraints:
        reference = constraint.expr.reference if isinstance(constraint.expr, EqualityConstraint) else None
        if reference is None or reference.quantity is None or reference.quantity.name != quantity.name:
            continue
        target = constraint_carrier(constraint, world_quantities)
        if target is not None:
            sources.append((pose_frame_names(target), target))
            break
    return next(((names, source) for names, source in sources if names is not None), None)
