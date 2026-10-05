# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""The operands a constraint's view operator reads, and the nodes they need, built once per constraint.

The carrier each plan holds is `classes.views.constraint_carrier`'s, which validation reads too.
"""

from __future__ import annotations

from rdf_utils.models.geom_rel import PoseModel
from rdf_utils.models.vocab import URI_GEOM_PRED_OF_POSE, URI_QUDT_QK_LENGTH
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import RDF
from rdflib.term import URIRef

from motion_spec_dsl.classes.constraints import (
    ANGLE_VIEW_FORMS,
    ConstraintSpecification,
    ViewForm,
    view_form,
)
from motion_spec_dsl.classes.context import (
    GEOMETRIC_DISTANCE_OPS,
    GEOMETRIC_PROJECTION_OPS,
    ContextQuantity,
    ContextQuantityAlias,
    GeometricPropKey,
    QuantityType,
    WorldQuantity,
    geo_prop,
    geo_prop_value,
    geometric_operand_kind,
    pose_frame_names,
)
from motion_spec_dsl.classes.views import (
    constraint_carrier,
    derived_view_constraint,
    distance_operand,
    existing_world_pose,
    owning_motion,
)
from motion_spec_dsl.rdf.common import (
    AlignmentPlan,
    DistancePlan,
    GeometricDistancePlan,
)
from motion_spec_dsl.rdf.emission import (
    Emission,
    declared_uri,
    model_local_path,
    owned_uri,
)
from motion_spec_dsl.rdf.geometry import frame_origin, record_coord_selection
from motion_spec_dsl.rdf_parser.vocab import (
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    QUDT_QKIND,
    QUDT_SCHEMA,
)


def derived_scalar_spec(em: Emission, quantity: ContextQuantity) -> ConstraintSpecification:
    """A spec scalar's view as a constraint, one per quantity so its plan is built once."""
    if quantity not in em.derived_specs:
        em.derived_specs[quantity] = derived_view_constraint(quantity)
    return em.derived_specs[quantity]


def derived_scalar_quantity(em: Emission, quantity: ContextQuantity, world_qtys) -> WorldQuantity | None:
    """The carrier a spec scalar defined by a binary view resolves to, its ops emitted once."""
    if quantity not in em.derived_scalars:
        em.derived_scalars[quantity] = resolve_constraint_quantity(
            em, derived_scalar_spec(em, quantity), world_qtys
        )
    return em.derived_scalars[quantity]


def resolve_constraint_quantity(em: Emission, spec: ConstraintSpecification, world_qtys) -> WorldQuantity | None:
    """The world quantity a constraint acts on: a plan's carrier, the view's quantity, or None for a clock."""
    if spec.view.elapsed is not None:
        return None
    form = view_form(spec)
    if form == ViewForm.DistanceBetween:
        return distance_plan(em, spec, world_qtys).target
    if form in ANGLE_VIEW_FORMS:
        return angle_plan(em, spec, world_qtys).target
    if form in (ViewForm.DistanceFrom, ViewForm.ProjectionOn):
        return geometric_distance_plan(em, spec, world_qtys).target
    return constraint_carrier(spec, world_qtys)


def distance_plan(em: Emission, spec: ConstraintSpecification, world_qtys) -> DistancePlan:
    """A `distance between` relation's pose endpoints, their origin Points, and its motion-qualified carrier."""
    if spec in em.plans:
        return em.plans[spec]
    start = distance_operand(spec.view.binary.left, world_qtys)
    end = distance_operand(spec.view.binary.right, world_qtys)
    start_frame = pose_frame_names(start)[0]
    end_frame = pose_frame_names(end)[0]
    target = constraint_carrier(spec, world_qtys)
    # The Point is the frame's origin: a live pose and its snapshot share it, and the PROV
    # selection, not the Point, tells which coordinate each endpoint names.
    relation_a = str(frame_origin(em, owned_uri(em, start_frame, start)))
    relation_b = str(frame_origin(em, owned_uri(em, end_frame, end)))
    plan = DistancePlan(start, end, target, relation_a, relation_b)
    em.plans[spec] = plan
    return plan


def emit_distance_operand_selection(em: Emission, distance_node: URIRef, plan: DistancePlan) -> None:
    """Record, as PROV, which pose coordinate each distance endpoint names.

    Its two origin Points cannot tell a live pose from its own snapshot over the same frame.
    """
    policy = owned_uri(em, "coord-policy/distance-operand", None)
    for role, operand in (("start", plan.start), ("end", plan.end)):
        chosen = URIRef(operand.uri)
        relation = em.graph.value(chosen, URI_GEOM_PRED_OF_POSE)
        if relation is None:
            continue
        record_coord_selection(
            em,
            URIRef(f"{distance_node}-{role}-selection"),
            relation,
            PoseModel(relation, em.graph).coordinate_ids,
            chosen,
            policy,
            "distance operand: the coordinate the constraint names",
        )


def angle_plan(em: Emission, spec: ConstraintSpecification, world_qtys) -> AlignmentPlan:
    """An angle's directions, a plane standing in by its normal, and the declared pose relating them.

    Only the first operand moves; the pose is read at runtime, so it must be a declared world pose.
    """
    if spec in em.plans:
        return em.plans[spec]
    form = view_form(spec)
    planes = (form == ViewForm.PlaneAngle, form != ViewForm.Alignment)
    directions, relations = [], []
    for ref, is_plane in zip((spec.view.binary.left, spec.view.binary.right), planes):
        operand = ref.ref if isinstance(ref, ContextQuantityAlias) else ref
        direction = geo_prop_value(operand.props, GeometricPropKey.Normal) if is_plane else operand
        directions.append(direction.ref if isinstance(direction, ContextQuantityAlias) else direction)
        relations.append(str(operand.uri))
    moving, reference = directions
    plan = AlignmentPlan(moving, reference, constraint_carrier(spec, world_qtys), relations[0], relations[1])
    em.plans[spec] = plan
    return plan


def geometric_distance_plan(em: Emission, spec: ConstraintSpecification, world_qtys) -> GeometricDistancePlan:
    """A Table II expression's operator, operands and carrier, over declared world poses."""
    if spec in em.plans:
        return em.plans[spec]
    binary = spec.view.binary
    a_ref, b_ref = binary.left, binary.right
    table = GEOMETRIC_DISTANCE_OPS if view_form(spec) == ViewForm.DistanceFrom else GEOMETRIC_PROJECTION_OPS
    op_type = table[(geometric_operand_kind(a_ref), geometric_operand_kind(b_ref))]
    motion = owning_motion(spec)
    stem = f"geo-distance-{motion.name}-{spec.name}"
    primitive = b_ref.ref if isinstance(b_ref, ContextQuantityAlias) else b_ref
    primitive_key = GeometricPropKey.Normal if primitive.type == QuantityType.Plane else GeometricPropKey.Along
    primitive_direction = geo_prop_value(primitive.props, primitive_key)
    if isinstance(primitive_direction, ContextQuantityAlias):
        primitive_direction = primitive_direction.ref
    if op_type in ("LineLineToLinearDistance", "LineOnLineProjection"):
        # A line-line op differences the two line origins through a PoseDiffEvaluator.
        line_a = a_ref.ref if isinstance(a_ref, ContextQuantityAlias) else a_ref
        dir_a = geo_prop_value(line_a.props, GeometricPropKey.Along)
        dir_a = dir_a.ref if isinstance(dir_a, ContextQuantityAlias) else dir_a
        frame = geo_prop(dir_a.props, "as-seen-by")
        origin_a = existing_world_pose(world_qtys, geo_prop(line_a.props, "of"), frame)
        origin_b = existing_world_pose(world_qtys, geo_prop(primitive.props, "of"), frame)
        pose_diff = owned_uri(em, f"{stem}-pose-diff", motion)
        diff_op = owned_uri(em, f"compute-{stem}-pose-diff", motion)
        em.graph.add((diff_op, RDF.type, GEOM_OP_EXT.PoseDiffEvaluator))
        em.graph.add((diff_op, GEOM_OP.in1, URIRef(origin_a.uri)))
        em.graph.add((diff_op, GEOM_OP.in2, URIRef(origin_b.uri)))
        em.graph.add((diff_op, GEOM_OP.out, pose_diff))
        # The shape the PoseDiffEvaluator function writes into.
        point_node = declared_uri(f"point-{stem}-origin", motion)
        em.graph.add((point_node, RDF.type, GEOM_ENT.Point))
        em.graph.add((pose_diff, RDF.type, GEOM_COORD.PoseDifferenceCoordinate))
        em.graph.add((pose_diff, RDF.type, GEOM_COORD.VectorXYZ))
        em.graph.add((pose_diff, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.PlaneAngle))
        em.graph.add((pose_diff, QUDT_SCHEMA.hasQuantityKind, URI_QUDT_QK_LENGTH))
        em.graph.add((pose_diff, GEOM_REL["reference-point"], point_node))
        em.graph.add((pose_diff, GEOM_COORD["as-seen-by"], owned_uri(em, frame, motion)))
        em.graph.add((pose_diff, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        em.graph.add((pose_diff, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        plan = GeometricDistancePlan(
            op_type=op_type,
            in1=str(dir_a.uri),
            in2=str(primitive_direction.uri),
            direction=None,
            pose=str(pose_diff),
            diff_in1=str(origin_a.uri),
            diff_in2=str(origin_b.uri),
            relation_a=str(line_a.uri),
            relation_b=str(primitive.uri),
            gradient_frame=frame,
            target=constraint_carrier(spec, world_qtys),
        )
        em.plans[spec] = plan
        return plan
    point = distance_operand(a_ref, world_qtys)
    point_of, point_wrt, _ = pose_frame_names(point)
    primitive_frame = geo_prop(primitive.props, "of")
    # A line riding the point's measurement frame has that frame's origin, so no pose to read.
    body_line = op_type == "PointLineToLinearDistance" and primitive_frame == point_wrt
    origin = None if body_line else existing_world_pose(world_qtys, primitive_frame, point_wrt)
    plan = GeometricDistancePlan(
        op_type="PointBodyLineToLinearDistance" if body_line else op_type,
        in1=str(point.uri),
        in2=str(point.uri if body_line else origin.uri),
        direction=str(primitive_direction.uri),
        pose=None,
        diff_in1=None,
        diff_in2=None,
        relation_a=str(frame_origin(em, owned_uri(em, point_of, motion))),
        relation_b=str(primitive.uri),
        gradient_frame=point_wrt,
        target=constraint_carrier(spec, world_qtys),
    )
    em.plans[spec] = plan
    return plan


def distance_relation(
    em: Emission, cache: dict, kind: str, a_uri: str, b_uri: str, relation_type: URIRef
) -> URIRef:
    """The relation two entities stand in, minted once per pair and shared by every motion measuring it."""
    key = (str(a_uri), str(b_uri))
    if key not in cache:
        name = "-".join([kind, model_local_path(em, a_uri), model_local_path(em, b_uri)])
        node = owned_uri(em, name, None)
        em.graph.add((node, RDF.type, relation_type))
        em.graph.add((node, GEOM_REL["between-entities"], URIRef(a_uri)))
        em.graph.add((node, GEOM_REL["between-entities"], URIRef(b_uri)))
        cache[key] = node
    return cache[key]
