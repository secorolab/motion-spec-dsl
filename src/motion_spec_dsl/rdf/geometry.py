# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Geometric relations, coordinates and the map views that name their components."""

from __future__ import annotations

import math
from typing import Any

from rdf_utils.models.vocab import (
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_QUATERNION,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_QK_LENGTH,
)
from rdf_utils.namespace import NS_MM_GEOM_REL, NS_MM_QUDT_QTY
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import PROV, RDF, RDFS, XSD
from rdflib.term import BNode, Literal, URIRef

from motion_spec_dsl.classes.context import (
    NORM_SCALAR_TYPES,
    ContextQuantity,
    ContextQuantityAlias,
    DistanceBetweenView,
    QuantityType,
    View,
    WorldQuantity,
    WorldQuantityType,
    pose_frame_names,
    scalar_type,
)
from motion_spec_dsl.classes.controller_semantics import SUBSPACE_ALIAS
from motion_spec_dsl.classes.coordinates import const_value
from motion_spec_dsl.classes.units import ANGLE_UNITS, DSL_UNITS
from motion_spec_dsl.rdf.common import norm_id, scalar_id
from motion_spec_dsl.rdf.emission import Emission, add_quantity, emit_view, owned_uri
from motion_spec_dsl.rdf.model import GEOM_DOMAIN_SPLIT, WORLD_SPECS
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_COORD,
    RBDYN_ENT,
)

# Views of a whole quantity: the quantity itself names it.
_WHOLE_VIEW_TYPES = {
    WorldQuantityType.Pose,
    WorldQuantityType.VelocityTwist,
    WorldQuantityType.Wrench,
    WorldQuantityType.JointPosition,
    WorldQuantityType.JointVelocity,
    WorldQuantityType.JointForce,
}


def emit_geom_relation(
    em: Emission,
    coord_node: URIRef,
    domain: str,
    of_node: URIRef | None,
    wrt_node: URIRef | None,
    as_seen_by: URIRef | None = None,
    qkinds: tuple[URIRef, ...] = (),
) -> URIRef:
    """Split a geometric quantity into comp-rob2b's relation and coordinate.

    The relation is a fact about a frame pair, so every coordinate over one (domain, of, wrt)
    shares it; one missing `of` or `wrt` keeps a relation of its own.
    """
    ref_type, coord_type, of_pred, rel_type = GEOM_DOMAIN_SPLIT[domain]
    rel_node = (
        URIRef(f"{of_node}-{wrt_node}-{domain}-rel")
        if of_node is not None and wrt_node is not None
        else URIRef(f"{coord_node}-{domain}-rel")
    )
    em.graph.add((rel_node, RDF.type, rel_type))
    em.graph.add((rel_node, RDF.type, QUDT_SCHEMA.Quantity))
    if of_node is not None:
        em.graph.add((rel_node, URI_GEOM_PRED_OF, of_node))
    if wrt_node is not None:
        em.graph.add((rel_node, URI_GEOM_PRED_WRT, wrt_node))
    for qkind in qkinds:
        em.graph.add((rel_node, QUDT_SCHEMA.hasQuantityKind, qkind))
    em.graph.add((coord_node, RDF.type, ref_type))
    em.graph.add((coord_node, RDF.type, coord_type))
    em.graph.add((coord_node, of_pred, rel_node))
    if as_seen_by is not None:
        em.graph.add((coord_node, URI_GEOM_PRED_SEEN_BY, as_seen_by))
    em.component_relations[(coord_node, domain)] = rel_node
    return rel_node


def frame_origin(em: Emission, frame: URIRef) -> URIRef:
    """A frame's origin Point, the established `<frame>-origin` one when the scene states none."""
    origin = em.graph.value(frame, URI_GEOM_PRED_ORIGIN)
    if not isinstance(origin, URIRef):
        origin = URIRef(f"{frame}-origin")
        em.graph.add((frame, URI_GEOM_PRED_ORIGIN, origin))
    em.graph.add((frame, RDF.type, URI_GEOM_TYPE_FRAME))
    em.graph.add((origin, RDF.type, URI_GEOM_TYPE_POINT))
    return origin


def emit_orientation_type(em: Emission, node: URIRef, orientation) -> None:
    """Type an orientation by the representation authored, with the angle kind and unit it implies.

    Euler angles are angles in their authored unit, quaternion and direction-cosine components
    dimensionless; no authored orientation means extrinsic xyz Euler, the KDL backend's RPY.
    """
    quat = orientation.quat if orientation is not None else None
    direction_cosine = orientation.direction_cosine if orientation is not None else None
    euler = orientation.euler if orientation is not None else None
    if quat is not None:
        em.graph.add((node, RDF.type, URI_GEOM_TYPE_QUATERNION))
    elif direction_cosine is not None:
        em.graph.add((node, RDF.type, URI_GEOM_TYPE_DIRECTION_COSINE_XYZ))
    else:
        em.graph.add((node, RDF.type, URI_GEOM_TYPE_EULER_ANGLES))
        if euler is not None and all(element.ref is None for element in euler.angles.values):
            em.graph.add((node, RDF.type, URI_GEOM_TYPE_ANGLES_ABG))
        extrinsic = euler.extrinsic if euler is not None else True
        em.graph.add((node, RDF.type, URI_GEOM_TYPE_EXTRINSIC if extrinsic else URI_GEOM_TYPE_INTRINSIC))
        em.graph.add((node, URI_GEOM_PRED_AXES_SEQ, Literal(euler.axes if euler else "xyz")))
    if quat is None and direction_cosine is None:
        em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.PlaneAngle))
        em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[euler.unit if euler else "rad"].iri))
    else:
        em.graph.remove((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.PlaneAngle))
        for angle_unit in ANGLE_UNITS:
            em.graph.remove((node, QUDT_SCHEMA.unit, angle_unit))


def emit_combined_pose_coordinate(
    em: Emission, node: URIRef, pose_relation: URIRef, orientation=None
) -> None:
    """Complete a pose coordinate whose position and orientation share one node."""
    pose_of, pose_wrt, pose_asb = em.frame_coords[node]
    position_relation = emit_geom_relation(
        em,
        node,
        "position",
        frame_origin(em, pose_of),
        frame_origin(em, pose_wrt),
        pose_asb,
        (URI_QUDT_QK_LENGTH,),
    )
    orientation_relation = emit_geom_relation(
        em, node, "orientation", pose_of, pose_wrt, pose_asb, (QUDT_QKIND.PlaneAngle,)
    )
    em.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    if orientation is not None:
        emit_orientation_type(em, node, orientation)
    em.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
    em.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
    em.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
    em.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))


def emit_declared_pose_frame_metadata(em: Emission, node: URIRef, quantity) -> URIRef | None:
    """Attach a context pose's authored of, wrt and as-seen-by frames; None when it states none."""
    frames = pose_frame_names(quantity)
    if frames is None:
        return None
    of_node, wrt_node, seen_by_node = (owned_uri(em, name, quantity) for name in frames)
    relation = emit_geom_relation(
        em, node, "pose", of_node, wrt_node, seen_by_node, (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH)
    )
    em.frame_coords[node] = (of_node, wrt_node, seen_by_node)
    return relation


def emit_snapshot_geometry_metadata(em: Emission, node: URIRef, quantity: ContextQuantity) -> None:
    """Give a snapshot-with-offset result the frames of the quantity it is stated as.

    The offset's output carries no frames of its own; a derived goal authored `of` the gripper is
    a goal for the gripper, and a path joins only endpoints that are poses of one body.
    """
    if quantity.type == QuantityType.Pose:
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        pose_relation = emit_declared_pose_frame_metadata(em, node, quantity)
        if pose_relation is not None:
            emit_combined_pose_coordinate(em, node, pose_relation)
        return
    frames = pose_frame_names(quantity)
    if frames is None:
        return
    of_frame, wrt_frame, as_seen_by = frames
    em.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    emit_geom_relation(
        em,
        node,
        "position",
        owned_uri(em, of_frame, quantity),
        owned_uri(em, wrt_frame, quantity),
        owned_uri(em, as_seen_by, quantity),
        (URI_QUDT_QK_LENGTH,),
    )


def emit_direction_coordinate(
    em: Emission, node: URIRef, as_seen_by: URIRef, vector: tuple[float, float, float] | None = None
) -> None:
    """A unit direction coordinate, dimensionless, optionally with its components.

    A quantity too, so arithmetic over directions -- an expression's gradient -- takes it as one.
    """
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((node, RDF.type, NS_MM_GEOM_REL["Direction"]))
    em.graph.add((node, RDF.type, GEOM_COORD.DirectionCoordinate))
    em.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
    em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, NS_MM_QUDT_QTY["Dimensionless"]))
    em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
    em.graph.add((node, GEOM_COORD["as-seen-by"], as_seen_by))
    if vector is not None:
        for predicate, component in zip((GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z), vector):
            em.graph.add((node, predicate, Literal(float(component), datatype=XSD.double)))


def emit_zero_position_coordinate(
    em: Emission, node: URIRef, point_node: URIRef, as_seen_by: URIRef
) -> None:
    """A zero position at POINT_NODE: where a commanded force acts."""
    em.graph.add((point_node, RDF.type, GEOM_ENT.Point))
    em.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
    em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
    emit_geom_relation(em, node, "position", point_node, point_node, as_seen_by, (URI_QUDT_QK_LENGTH,))
    for predicate in (GEOM_COORD.x, GEOM_COORD.y, GEOM_COORD.z):
        em.graph.add((node, predicate, Literal(0.0, datatype=XSD.double)))


def emit_wrench_coordinate(
    em: Emission,
    node: URIRef,
    reference_point: URIRef,
    as_seen_by: URIRef,
    acts_on: URIRef | None = None,
) -> URIRef:
    """A wrench relation and its coordinate."""
    relation = URIRef(f"{node}-wrench-rel")
    em.graph.add((relation, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((relation, RDF.type, RBDYN_ENT.Wrench))
    em.graph.add((relation, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Torque))
    em.graph.add((relation, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Force))
    em.graph.add((relation, RBDYN_ENT["reference-point"], reference_point))
    if acts_on is not None:
        em.graph.add((relation, RBDYN_ENT["acts-on"], acts_on))
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((node, RDF.type, RBDYN_COORD.WrenchReference))
    em.graph.add((node, RDF.type, RBDYN_COORD.WrenchCoordinate))
    em.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
    em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["N-M"]))
    em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.N))
    em.graph.add((node, RBDYN_COORD["of-wrench"], relation))
    em.graph.add((node, RBDYN_COORD["as-seen-by"], as_seen_by))
    return relation


def emit_pose_to_direction(
    em: Emission, direction_node: URIRef, pose: WorldQuantity, as_seen_by: URIRef, motion, stem: str
) -> URIRef:
    """The unit direction a pose's translation points along, recomputed every cycle."""
    emit_direction_coordinate(em, direction_node, as_seen_by)
    op_node = owned_uri(em, f"compute-direction-{stem}", motion)
    em.graph.add((op_node, RDF.type, GEOM_OP.PoseToDirection))
    em.graph.add((op_node, GEOM_OP.pose, URIRef(pose.uri)))
    em.graph.add((op_node, GEOM_OP.direction, direction_node))
    return direction_node


def record_coord_selection(
    em: Emission,
    activity: URIRef,
    relation: URIRef,
    candidates,
    chosen: URIRef,
    policy: URIRef,
    policy_label: str,
) -> None:
    """Record a coordinate choice as PROV: the activity used the relation and every candidate.

    The chosen candidate is its qualified usage; a selection generates nothing, and recording it
    again changes nothing.
    """
    graph = em.graph
    graph.add((activity, RDF.type, PROV.Activity))
    graph.add((relation, RDF.type, PROV.Entity))
    graph.add((activity, PROV.used, relation))
    for candidate in candidates:
        graph.add((candidate, RDF.type, PROV.Entity))
        graph.add((activity, PROV.used, candidate))
    graph.add((activity, PROV.wasAssociatedWith, policy))
    graph.add((policy, RDF.type, PROV.SoftwareAgent))
    graph.add((policy, RDF.type, PROV.Agent))
    graph.add((policy, RDFS.label, Literal(policy_label)))
    if not any(
        (usage, PROV.entity, chosen) in graph for usage in graph.objects(activity, PROV.qualifiedUsage)
    ):
        usage = BNode()
        graph.add((activity, PROV.qualifiedUsage, usage))
        graph.add((usage, RDF.type, PROV.Usage))
        graph.add((usage, PROV.entity, chosen))


def record_pose_component(em: Emission, pose_node: URIRef, component: str, coordinate: URIRef) -> None:
    """Record which position or orientation coordinate a pose is built from.

    The pooled relation is shared by every pose over the same frames; the DSL knows which
    coordinate is this pose's, so it records it rather than leaving it to be guessed by name.
    """
    predicate = GEOM_COORD[f"of-{component}"]
    relation = em.graph.value(coordinate, predicate)
    if relation is None:
        return
    record_coord_selection(
        em,
        URIRef(f"{pose_node}-{component}-selection"),
        relation,
        list(em.graph.subjects(predicate, relation)),
        coordinate,
        owned_uri(em, "coord-policy/pose-component", None),
        "authored pose component",
    )


def emit_angle_normalization(em: Emission, owner_node: URIRef, angle_range, owner, name: str) -> None:
    """The interval an angle is moved onto by whole turns: not a clamp to its nearest edge."""
    node = owned_uri(em, name, owner)
    em.graph.add((node, RDF.type, ALGO_EXT.AngularNormalization))
    scale = math.pi / 180.0 if angle_range.unit == "deg" else 1.0
    for edge, bound_expr in (("lower", angle_range.lower), ("upper", angle_range.upper)):
        bound = owned_uri(em, f"{name}-{edge}", owner)
        em.graph.add((bound, RDF.type, QUDT_SCHEMA.Quantity))
        value = const_value(bound_expr.value) * scale
        em.graph.add((bound, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
        em.graph.add((bound, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        em.graph.add((bound, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.Angle))
        em.graph.add((node, ALGO_EXT[f"{edge}-bound"], bound))
    em.graph.add((owner_node, ALGO_EXT.normalization, node))


def map_view(
    em: Emission,
    view_uri: URIRef,
    view_type: URIRef | None,
    superobject: URIRef,
    subobject: URIRef,
    subspace: URIRef,
    axis: str | None = None,
) -> None:
    """The map:View exposing SUPEROBJECT's SUBSPACE (and AXIS) as SUBOBJECT."""
    emit_view(em, view_uri)
    if view_type is not None:
        em.graph.add((view_uri, RDF.type, view_type))
    em.graph.add((view_uri, MAP.superobject, superobject))
    em.graph.add((view_uri, MAP.subobject, subobject))
    em.graph.add((view_uri, MAP.subspace, subspace))
    if axis is not None:
        em.graph.add((view_uri, MAP.axis, MAP[axis]))


def component_view(em: Emission, coord_node: URIRef, domain: str) -> URIRef:
    """The view a constraint operand names for a coordinate's component, never the pooled relation."""
    if (coord_node, domain) not in em.component_views:
        raise ValueError(f"coordinate '{coord_node}' has no {domain} view to name as an operand")
    return em.component_views[(coord_node, domain)]


def register_pose_part_view(
    em: Emission, scalar_uri: URIRef, quantity: WorldQuantity, part: str, owner
) -> None:
    """Expose a pose's whole position or orientation through its coordinate view."""
    if scalar_uri in em.emitted_pose_parts:
        return
    em.emitted_pose_parts.add(scalar_uri)
    pose_node = URIRef(quantity.uri)
    view_uri = owned_uri(em, f"view-{scalar_id(quantity, part, None)}", owner)
    if view_uri not in em.emitted_views:
        map_view(
            em,
            view_uri,
            MAP_EXT.PoseCoordinateView,
            pose_node,
            em.component_relations[(pose_node, part)],
            MAP_EXT[part],
        )
    em.component_views.setdefault((pose_node, part), view_uri)


def register_pose_component_view(
    em: Emission, scalar_uri: URIRef, quantity: WorldQuantity, mapped_subspace: str, axis: str, owner
) -> None:
    """One axis of a pose's position (a distance) or orientation (an angle)."""
    view_uri = owned_uri(em, f"view-{scalar_id(quantity, mapped_subspace, axis)}", owner)
    view = WORLD_SPECS[WorldQuantityType.Pose].views.get(mapped_subspace)
    if view_uri in em.emitted_views or view is None:
        return
    add_quantity(em, scalar_uri, scalar_type(quantity, mapped_subspace, axis))
    subspace = MAP_EXT.orientation if view.subspace == "rotation" else MAP_EXT.position
    map_view(em, view_uri, None, URIRef(quantity.uri), scalar_uri, subspace, axis)


def register_world_component_view(
    em: Emission,
    scalar_uri: URIRef,
    quantity: WorldQuantity,
    mapped_subspace: str,
    axis: str | None,
    owner,
) -> None:
    """One subspace of a twist or wrench: one axis, or the whole 3-vector."""
    spec = WORLD_SPECS.get(quantity.type)
    view = spec.views.get(mapped_subspace) if spec is not None else None
    if view is None:
        return
    add_quantity(em, scalar_uri, scalar_type(quantity, mapped_subspace, axis))
    view_uri = owned_uri(em, f"view-{scalar_id(quantity, mapped_subspace, axis)}", owner)
    if view_uri not in em.emitted_views:
        map_view(em, view_uri, view.view_type, URIRef(quantity.uri), scalar_uri, MAP[view.subspace], axis)


def norm_view_node(em: Emission, view: View, owner) -> URIRef:
    """The norm a `norm of <q>.<subspace> [across <d>]` view names, with the operator filling it."""
    quantity = view.quantity
    raw = str(view.subspace)
    mapped = raw if quantity.type == WorldQuantityType.Pose and raw == "position" else SUBSPACE_ALIAS.get(raw, raw)
    vector_uri = owned_uri(em, scalar_id(quantity, mapped, None), owner)
    if quantity.type == WorldQuantityType.Pose:
        register_pose_part_view(em, vector_uri, quantity, "position", owner)
        in_node = component_view(em, URIRef(quantity.uri), "position")
    else:
        register_world_component_view(em, vector_uri, quantity, mapped, None, owner)
        in_node = vector_uri
    across = view.norm.across.quantity if view.norm.across is not None else None
    if isinstance(across, ContextQuantityAlias):
        across = across.ref
    identifier = norm_id(quantity, mapped, across.name if across is not None else None)
    norm_uri = owned_uri(em, identifier, owner)
    add_quantity(em, norm_uri, NORM_SCALAR_TYPES[scalar_type(quantity, mapped, None)])
    op_node = owned_uri(em, f"compute-{identifier}", owner)
    if op_node not in em.emitted_norm_ops:
        em.emitted_norm_ops.add(op_node)
        em.graph.add((op_node, RDF.type, GEOM_OP_EXT.VectorNorm))
        em.graph.add((op_node, GEOM_OP["in"], in_node))
        if across is not None:
            em.graph.add((op_node, GEOM_OP.direction, URIRef(across.uri)))
        em.graph.add((op_node, GEOM_OP_EXT.norm, norm_uri))
    return norm_uri


def view_node(em: Emission, view: Any, owner) -> URIRef:
    """The node a view, expression leaf or message field reads: the quantity, a component view, or a scalar."""
    if isinstance(view, View) and isinstance(view.binary, DistanceBetweenView):
        return owned_uri(em, f"distance-{view.binary.left.name}-{view.binary.right.name}", owner)
    if isinstance(view, View) and view.norm is not None:
        return norm_view_node(em, view, owner)
    quantity = view.quantity
    if not isinstance(quantity, WorldQuantity):
        return owned_uri(em, quantity.name, owner)
    if view.subspace is None and quantity.type in _WHOLE_VIEW_TYPES:
        return URIRef(quantity.uri)
    subspace = str(view.subspace)
    axis = str(view.axis) if view.axis is not None else None
    pose = quantity.type == WorldQuantityType.Pose
    if pose and subspace in {"position", "orientation"} and axis is None:
        mapped = subspace
    else:
        mapped = SUBSPACE_ALIAS.get(subspace, subspace)
    scalar_uri = owned_uri(em, scalar_id(quantity, mapped, axis), owner)
    if pose and mapped in {"distance", "rotation"} and axis is not None:
        register_pose_component_view(em, scalar_uri, quantity, mapped, axis, owner)
    elif pose and mapped in {"distance", "position"} and axis is None:
        register_pose_part_view(em, scalar_uri, quantity, "position", owner)
    elif pose and mapped in {"orientation", "rotation"} and axis is None:
        register_pose_part_view(em, scalar_uri, quantity, "orientation", owner)
    elif axis is not None or (quantity.type == WorldQuantityType.Wrench and mapped in {"force", "torque"}):
        register_world_component_view(em, scalar_uri, quantity, mapped, axis, owner)
    if pose and axis is None and mapped in {"distance", "position"}:
        return component_view(em, URIRef(quantity.uri), "position")
    if pose and axis is None and mapped in {"orientation", "rotation"}:
        return component_view(em, URIRef(quantity.uri), "orientation")
    return scalar_uri


def emit_profile_view_node(em: Emission, view: View, owner) -> URIRef:
    """A profile or admittance input's node, with the view of the one axis it selects."""
    node = view_node(em, view, owner)
    quantity = view.quantity
    if not isinstance(quantity, WorldQuantity) or view.axis is None:
        return node
    subspace = SUBSPACE_ALIAS.get(str(view.subspace), str(view.subspace))
    axis = str(view.axis)
    spec = WORLD_SPECS.get(quantity.type)
    world_view = spec.views.get(subspace) if spec is not None else None
    if world_view is None:
        return node
    add_quantity(em, node, scalar_type(quantity, subspace, axis))
    view_uri = owned_uri(em, f"view-{scalar_id(quantity, subspace, axis)}", owner)
    if view_uri in em.emitted_views:
        return node
    if quantity.type == WorldQuantityType.Pose:
        map_view(
            em,
            view_uri,
            None,
            URIRef(quantity.uri),
            node,
            MAP_EXT.orientation if subspace == "rotation" else MAP_EXT.position,
            axis,
        )
    else:
        map_view(em, view_uri, world_view.view_type, URIRef(quantity.uri), node, MAP[world_view.subspace], axis)
    return node
