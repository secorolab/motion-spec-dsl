# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The scalars constraints read, the map:Views exposing them, and the operators computing them."""

from __future__ import annotations

from rdf_utils.namespace import NS_MM_GEOM_REL, NS_MM_QUDT_QTY
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import RDF
from rdflib.term import URIRef

from motion_spec_dsl.classes.constraints import ANGLE_VIEW_FORMS, ConstraintSpecification, ViewForm, view_form
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_RELATION,
    ContextQuantityAlias,
    QuantityType,
    WorldQuantityType,
    geo_prop,
    pose_frame_names,
    scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import alignment_is_pointwise, constraint_view_subspace
from motion_spec_dsl.classes.motion_spec import GuardedMotion
from motion_spec_dsl.classes.views import resolve_world_quantity
from motion_spec_dsl.rdf.common import alignment_id, constraint_scalar_id, gradient_scalar_id, scalar_id
from motion_spec_dsl.rdf.emission import (
    Emission,
    add_quantity,
    declared_uri,
    emit_quantity_kind,
    emit_view,
    owned_uri,
)
from motion_spec_dsl.rdf.geometry import emit_direction_coordinate, view_node
from motion_spec_dsl.rdf.model import WORLD_SPECS
from motion_spec_dsl.rdf.plans import (
    angle_plan,
    distance_plan,
    distance_relation,
    emit_distance_operand_selection,
    geometric_distance_plan,
    resolve_constraint_quantity,
)
from motion_spec_dsl.rdf.quantities import path_geometry_node
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR_HDL_EXT,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    GEOM_REL_EXT,
    MAP,
    MAP_EXT,
    QUDT_SCHEMA,
)


def emit_scalar_views(
    em: Emission, motion: GuardedMotion, constraints: list[ConstraintSpecification], world_qtys: dict
) -> None:
    """The scalar and map:View of each subspace a constraint views, once per motion."""
    seen = set()
    rotation = False
    for spec in constraints:
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        if qty is None:
            continue
        subspace = constraint_view_subspace(spec)
        axis = str(spec.view.axis) if spec.view.axis is not None else None
        pose = qty.type == WorldQuantityType.Pose
        # A norm names no axis, so the sweep below would never emit its operator.
        if view_form(spec) == ViewForm.Norm:
            key = (constraint_scalar_id(qty, spec),)
            if key not in seen:
                seen.add(key)
                view_node(em, spec.view, motion)
            continue
        spec_views = WORLD_SPECS.get(qty.type)
        if spec_views is None:
            continue
        world_view = spec_views.views.get(subspace)
        if pose and subspace == "rotation" and axis is None:
            rotation = True
            continue
        if pose and subspace == "pose":
            key = (qty.name, "pose", None)
            if key not in seen:
                seen.add(key)
                pose_scalar = owned_uri(em, scalar_id(qty, "pose", None), motion)
                add_quantity(em, pose_scalar, QuantityType.Pose)
                of_v = geo_prop(qty.props, "of")
                wrt_v = geo_prop(qty.props, "wrt")
                if of_v and wrt_v:
                    em.graph.add((pose_scalar, GEOM_REL.of, owned_uri(em, of_v, qty)))
                    em.graph.add((pose_scalar, GEOM_REL["with-respect-to"], owned_uri(em, wrt_v, qty)))
                    em.graph.add((pose_scalar, GEOM_COORD["as-seen-by"], owned_uri(em, wrt_v, qty)))
            continue
        if pose and subspace in {"position", "orientation"} and axis is None:
            key = (qty.name, subspace, None)
            if key not in seen:
                seen.add(key)
                view_node(em, spec.view, motion)
            continue
        if axis is None or world_view is None or (qty.name, subspace, axis) in seen:
            continue
        seen.add((qty.name, subspace, axis))
        sid = scalar_id(qty, subspace, axis)
        scalar_node = owned_uri(em, sid, motion)
        add_quantity(em, scalar_node, scalar_type(qty, subspace, axis))
        view = owned_uri(em, f"view-{sid}", motion)
        emit_view(em, view)
        if pose:
            map_subspace = MAP_EXT.orientation if subspace == "rotation" else MAP_EXT.position
        else:
            em.graph.add((view, RDF.type, world_view.view_type))
            map_subspace = MAP[world_view.subspace]
        em.graph.add((view, MAP.superobject, URIRef(qty.uri)))
        em.graph.add((view, MAP.subobject, scalar_node))
        em.graph.add((view, MAP.subspace, map_subspace))
        em.graph.add((view, MAP.axis, MAP[axis]))
    for spec in constraints:
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        if qty is None:
            continue
        subspace = constraint_view_subspace(spec)
        axis = str(spec.view.axis) if spec.view.axis is not None else None
        if scalar_type(qty, subspace, None) in (QuantityType.Angle, QuantityType.PlaneAngle):
            add_quantity(em, owned_uri(em, scalar_id(qty, subspace, axis), motion), scalar_type(qty, subspace, axis))
    if rotation:
        add_quantity(em, owned_uri(em, f"rotation-{motion.name}", motion), QuantityType.Angle)


def emit_map_operations(
    em: Emission, motion, constraints: list[ConstraintSpecification], world_qtys: dict
) -> None:
    """The geometry operators computing each constraint's scalar from the poses and directions it reads."""
    rotation_pose = next(
        (
            spec.view.quantity.name
            for spec in constraints
            if spec.view.elapsed is None
            and resolve_constraint_quantity(em, spec, world_qtys) is not None
            and constraint_view_subspace(spec) == "rotation"
            and spec.view.axis is None
        ),
        None,
    )
    if rotation_pose is not None:
        rotation_id = f"rotation-{motion.name}"
        op_node = owned_uri(em, f"compute-{rotation_id}", motion)
        em.graph.add((op_node, RDF.type, GEOM_OP_EXT.PoseToAngularDistance))
        em.graph.add((op_node, GEOM_OP.pose, owned_uri(em, rotation_pose, motion)))
        em.graph.add((op_node, GEOM_OP_EXT["angular-distance"], owned_uri(em, rotation_id, motion)))
    angle_ops = set()
    for spec in constraints:
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        if qty is None or qty.type != WorldQuantityType.Pose:
            continue
        subspace = constraint_view_subspace(spec)
        if subspace != "rotation" or spec.view.axis is None:
            continue
        axis = str(spec.view.axis)
        angle_id = scalar_id(qty, subspace, axis)
        if angle_id in angle_ops:
            continue
        angle_ops.add(angle_id)
        op_node = owned_uri(em, f"compute-{angle_id}", motion)
        em.graph.add((op_node, RDF.type, GEOM_OP.PoseToAngleAroundAxis))
        em.graph.add((op_node, GEOM_OP.pose, owned_uri(em, qty.name, motion)))
        em.graph.add((op_node, GEOM_OP.angle, owned_uri(em, angle_id, motion)))
        em.graph.add((op_node, GEOM_OP.axis, GEOM_OP[axis]))
    for spec in constraints:
        if view_form(spec) != ViewForm.DistanceBetween or spec.view.axis is not None:
            continue
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        if constraint_view_subspace(spec) != "distance":
            continue
        distance_id = scalar_id(qty, "distance", None)
        if distance_id in em.emitted_distance_ops:
            continue
        em.emitted_distance_ops.add(distance_id)
        plan = distance_plan(em, spec, world_qtys)
        distance_node = owned_uri(em, distance_id, motion)
        add_quantity(em, distance_node, QuantityType.Distance)
        em.graph.add((distance_node, RDF.type, GEOM_COORD.DistanceReference))
        relation = distance_relation(
            em,
            em.linear_distance_relations,
            "linear-distance",
            plan.relation_a,
            plan.relation_b,
            GEOM_REL.PointToPointDistance,
        )
        em.graph.add((distance_node, GEOM_COORD.of, relation))
        emit_distance_operand_selection(em, distance_node, plan)
    # Table IIb: the angles share the rotated direction; relation, operator and error differ.
    for spec in constraints:
        form = view_form(spec)
        if form not in ANGLE_VIEW_FORMS:
            continue
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        angle_id = alignment_id(qty, spec)
        if angle_id in em.emitted_alignment_ops:
            continue
        em.emitted_alignment_ops.add(angle_id)
        plan = angle_plan(em, spec, world_qtys)
        reference_frame = owned_uri(em, geo_prop(plan.target.props, "wrt"), motion)
        reference = URIRef(plan.reference.uri)
        rotated_node = owned_uri(em, f"{angle_id}-rotated", motion)
        emit_direction_coordinate(em, rotated_node, reference_frame)
        rotate_op = owned_uri(em, f"compute-{angle_id}-rotated", motion)
        em.graph.add((rotate_op, RDF.type, GEOM_OP.RotateDirectionDistalToProximalWithPose))
        em.graph.add((rotate_op, GEOM_OP.pose, URIRef(plan.target.uri)))
        em.graph.add((rotate_op, GEOM_OP["from"], URIRef(plan.moving.uri)))
        em.graph.add((rotate_op, GEOM_OP.to, rotated_node))
        theta_node = owned_uri(em, angle_id, motion)
        add_quantity(em, theta_node, QuantityType.Angle)
        # Fixed-axis solver rows are decided from this frame downstream.
        em.graph.add((theta_node, GEOM_COORD["as-seen-by"], reference_frame))
        if form == ViewForm.PlaneAngle:
            relation_type = GEOM_REL_EXT.PlanePlaneAngularDistance
        elif form == ViewForm.IncidentAngle:
            relation_type = GEOM_REL_EXT.DirectionPlaneAngularDistance
        else:
            relation_type = GEOM_REL_EXT.DirectionDirectionAngularDistance
        relation = distance_relation(
            em, em.angular_distance_relations, "angular-distance", plan.relation_a, plan.relation_b, relation_type
        )
        em.graph.add((theta_node, GEOM_COORD.of, relation))
        angle_op = owned_uri(em, f"compute-{angle_id}", motion)
        if form == ViewForm.IncidentAngle:
            gradient_node = owned_uri(em, gradient_scalar_id(qty, spec), motion)
            emit_direction_coordinate(em, gradient_node, reference_frame)
            em.graph.add((angle_op, RDF.type, GEOM_OP_EXT.DirectionPlaneToAngularDistance))
            em.graph.add((angle_op, GEOM_OP.in1, rotated_node))
            em.graph.add((angle_op, GEOM_OP.in2, reference))
            em.graph.add((angle_op, GEOM_OP.angle, theta_node))
            em.graph.add((angle_op, GEOM_OP_EXT.gradient, gradient_node))
            continue
        em.graph.add((angle_op, RDF.type, GEOM_OP.PlanarAngleFromDirections))
        em.graph.add((angle_op, GEOM_OP["from-directions"], rotated_node))
        em.graph.add((angle_op, GEOM_OP["from-directions"], reference))
        em.graph.add((angle_op, GEOM_OP.angle, theta_node))
        if form == ViewForm.Alignment and alignment_is_pointwise(spec):
            # A point target (2 DOF) is driven by the cross-product error.
            vector_node = owned_uri(em, f"{angle_id}-error", motion)
            add_quantity(em, vector_node, QuantityType.FreeVector)
            em.graph.add((vector_node, RDF.type, GEOM_COORD.VectorXYZ))
            em.graph.remove((vector_node, QUDT_SCHEMA.unit, None))
            em.graph.add((vector_node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            em.graph.add((vector_node, GEOM_COORD["as-seen-by"], reference_frame))
            vector_op = owned_uri(em, f"compute-{angle_id}-error", motion)
            em.graph.add((vector_op, RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))
            em.graph.add((vector_op, GEOM_OP.in1, rotated_node))
            em.graph.add((vector_op, GEOM_OP.in2, reference))
            em.graph.add((vector_op, GEOM_OP.out, vector_node))
            continue
        # A cone or plane-plane angle (1 DOF) is driven along theta's gradient, from ordered in1/in2.
        gradient_node = owned_uri(em, gradient_scalar_id(qty, spec), motion)
        emit_direction_coordinate(em, gradient_node, reference_frame)
        gradient_op = owned_uri(em, f"compute-{angle_id}-gradient", motion)
        em.graph.add((gradient_op, RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
        em.graph.add((gradient_op, GEOM_OP.in1, rotated_node))
        em.graph.add((gradient_op, GEOM_OP.in2, reference))
        em.graph.add((gradient_op, GEOM_OP_EXT.gradient, gradient_node))
    # Table IIa: point-plane, point-line and line-line distances and projections.
    for spec in constraints:
        if view_form(spec) not in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
            continue
        qty = resolve_constraint_quantity(em, spec, world_qtys)
        distance_id = scalar_id(qty, constraint_view_subspace(spec), None)
        if distance_id in em.emitted_geometric_distance_ops:
            continue
        em.emitted_geometric_distance_ops.add(distance_id)
        plan = geometric_distance_plan(em, spec, world_qtys)
        body_line = plan.op_type == "PointBodyLineToLinearDistance"
        distance_node = owned_uri(em, distance_id, motion)
        add_quantity(em, distance_node, QuantityType.Distance)
        relation_name = GEOMETRIC_DISTANCE_RELATION[plan.op_type]
        # The body-line case is comp-rob2b's own PointLineCollinearity relation.
        relation_type = NS_MM_GEOM_REL[relation_name] if body_line else GEOM_REL_EXT[relation_name]
        relation = distance_relation(
            em, em.linear_distance_relations, "linear-distance", plan.relation_a, plan.relation_b, relation_type
        )
        em.graph.add((distance_node, GEOM_COORD.of, relation))
        gradient_frame = owned_uri(em, plan.gradient_frame, motion)
        gradient_node = owned_uri(em, gradient_scalar_id(qty, spec), motion)
        emit_direction_coordinate(em, gradient_node, gradient_frame)
        op_node = owned_uri(em, f"compute-{distance_id}", motion)
        em.graph.add((op_node, RDF.type, GEOM_OP_EXT[plan.op_type]))
        em.graph.add((op_node, GEOM_OP.in1, URIRef(plan.in1)))
        em.graph.add((op_node, GEOM_OP.in2, URIRef(plan.in2)))
        if plan.direction is not None:
            em.graph.add((op_node, GEOM_OP.direction, URIRef(plan.direction)))
        if plan.pose is not None:
            em.graph.add((op_node, GEOM_OP.pose, URIRef(plan.pose)))
        em.graph.add((op_node, GEOM_OP.distance, distance_node))
        em.graph.add((op_node, GEOM_OP_EXT.gradient, gradient_node))
        if body_line:
            # Tilting a body-fixed line sweeps it across the point: an angular term of its own.
            moment_node = owned_uri(em, f"{gradient_scalar_id(qty, spec)}-moment", motion)
            emit_direction_coordinate(em, moment_node, gradient_frame)
            em.graph.add((op_node, GEOM_OP_EXT["gradient-moment"], moment_node))


def emit_path_following(em: Emission, constraints: list[ConstraintSpecification], world_qtys: dict) -> None:
    """One projection per path a motion's constraints follow: where on it the frame already is.

    It replaces a commanded path parameter: nothing upstream writes the parameter, so nothing
    can outrun the robot.
    """
    for spec in constraints:
        operand = spec.view.moving or spec.view.on or spec.view.progress
        if operand is None:
            continue
        path = operand.path.quantity
        path = path.ref if isinstance(path, ContextQuantityAlias) else path
        projection_node = declared_uri(f"projection-{path.name}", path)
        if projection_node in em.path_projections:
            continue
        em.path_projections.add(projection_node)
        moved = resolve_world_quantity(operand.moved, world_qtys)
        path_node = path_geometry_node(path)
        as_seen_by = owned_uri(em, geo_prop(moved.props, "as-seen-by") or geo_prop(moved.props, "wrt"), moved)
        parameter_node = declared_uri(f"{path.name}-s", path)
        em.graph.add((parameter_node, RDF.type, QUDT_SCHEMA.Quantity))
        emit_quantity_kind(em, parameter_node, NS_MM_QUDT_QTY["Dimensionless"])
        em.graph.add((parameter_node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
        speed_node = declared_uri(f"{path.name}-along-speed", path)
        add_quantity(em, speed_node, QuantityType.LinearVelocity)
        # Only the projection reads the robot; the operators below are functions of its parameter.
        em.graph.add((projection_node, RDF.type, GEOM_OP_EXT.PathProjection))
        em.graph.add((projection_node, GEOM_OP_EXT.path, path_node))
        em.graph.add((projection_node, GEOM_OP.pose, URIRef(moved.uri)))
        em.graph.add((projection_node, GEOM_OP_EXT["path-parameter"], parameter_node))
        frame_node = declared_uri(f"frame-{path.name}", path)
        em.graph.add((frame_node, RDF.type, GEOM_OP_EXT.PathTangentFrame))
        em.graph.add((frame_node, GEOM_OP_EXT.path, path_node))
        em.graph.add((frame_node, GEOM_OP_EXT["path-parameter"], parameter_node))
        for term in ("tangent", "normal-a", "normal-b"):
            direction_node = declared_uri(f"{path.name}-{term}", path)
            em.graph.add((direction_node, RDF.type, QUDT_SCHEMA.Quantity))
            emit_direction_coordinate(em, direction_node, as_seen_by)
            em.graph.add((frame_node, GEOM_OP_EXT[term], direction_node))
        # The speed along the path is the measured twist of the followed frame onto the tangent.
        of_frame, wrt_frame, _ = pose_frame_names(moved)
        twist = next(
            quantity
            for quantity in world_qtys.values()
            if quantity.type == WorldQuantityType.VelocityTwist
            and geo_prop(quantity.props, "of") == of_frame
            and geo_prop(quantity.props, "wrt") == wrt_frame
        )
        along_node = declared_uri(f"along-{path.name}", path)
        em.graph.add((along_node, RDF.type, GEOM_OP_EXT.TwistToLinearVelocityAlong))
        em.graph.add((along_node, GEOM_OP["in"], URIRef(twist.uri)))
        em.graph.add((along_node, GEOM_OP.direction, declared_uri(f"{path.name}-tangent", path)))
        em.graph.add((along_node, GEOM_OP_EXT["along-speed"], speed_node))
        # The pose the path carries there is what the setpoint follows.
        evaluator_node = declared_uri(f"evaluator-{path.name}", path)
        em.graph.add((evaluator_node, RDF.type, GEOM_OP_EXT.PathEvaluator))
        em.graph.add((evaluator_node, RDF.type, CSTR_HDL_EXT.SetpointGenerator))
        em.graph.add((evaluator_node, GEOM_OP_EXT.path, path_node))
        em.graph.add((evaluator_node, GEOM_OP_EXT["path-parameter"], parameter_node))
        em.graph.add((evaluator_node, GEOM_OP.out, URIRef(f"{path.uri}/reference")))
