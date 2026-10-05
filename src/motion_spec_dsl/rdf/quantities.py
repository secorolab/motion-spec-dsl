# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""World and context quantities: their kinds, units, frames, coordinates and values."""

from __future__ import annotations

from typing import Any

from rdf_utils.collection import add_literal_list_pred
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_W,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_QK_LENGTH,
    URI_TIME_PRED_AFTER_EVT,
    URI_TIME_PRED_OF_CONSTRAINT,
    URI_TIME_TYPE_AFTER_EVT,
    URI_TIME_TYPE_TC,
)
from rdf_utils.namespace import NS_MM_QUDT_QTY
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import PROV, RDF, SDO, XSD
from rdflib.term import Literal, URIRef
from scene_dsl.classes.distrib import DistributionRef
from scene_dsl.rdf.distrib import add_sampled_quantity

from motion_spec_dsl.classes.context import (
    ConfigValue,
    ContextQuantity,
    ContextQuantityAlias,
    DerivedScalarValue,
    DirectionBetween,
    GeometricPropKey,
    Measure,
    QOpNode,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    SampledValue,
    SnapshotValue,
    VectorXYZ,
    WorldQuantity,
    WorldQuantityType,
    geo_prop,
    geo_prop_events,
    geo_prop_value,
    op_tree,
    path_pose_endpoints,
    pose_frame_names,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    PoseCoordinate,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.motion_spec import ContextDeclReference
from motion_spec_dsl.classes.path import AdmittanceSpec, ProfileSpec
from motion_spec_dsl.classes.units import DSL_UNITS, QUDT_KIND_BY_QUANTITY_TYPE
from motion_spec_dsl.classes.views import existing_world_pose, pose_frame_source
from motion_spec_dsl.rdf.common import JOINT_TYPES
from motion_spec_dsl.rdf.emission import (
    Emission,
    add_quantity,
    declared_uri,
    emit_quantity_kind,
    emit_scalar_quantity,
    emit_view,
    owned_uri,
)
from motion_spec_dsl.rdf.expressions import (
    emit_context_ref_node,
    emit_expression_gradient,
    emit_op_tree,
)
from motion_spec_dsl.rdf.geometry import (
    emit_angle_normalization,
    emit_combined_pose_coordinate,
    emit_declared_pose_frame_metadata,
    emit_direction_coordinate,
    emit_geom_relation,
    emit_orientation_type,
    emit_snapshot_geometry_metadata,
    emit_wrench_coordinate,
    frame_origin,
    record_pose_component,
    view_node,
)
from motion_spec_dsl.rdf.model import CONTEXT_COMPOSITE_WORLD_TYPE, SCALAR_UNIT, WORLD_SPECS
from motion_spec_dsl.rdf_parser.vocab import (
    AGN,
    CSTR,
    EST,
    EXEC,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_EXT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_PATH,
    GEOM_REL,
    KC_STAT,
    MAP,
    MAP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_COORD,
    SOSA,
    TIME,
)

_COORDINATE_PREDICATES = {
    "x": URI_GEOM_PRED_X,
    "y": URI_GEOM_PRED_Y,
    "z": URI_GEOM_PRED_Z,
    "w": URI_GEOM_PRED_W,
    "alpha": URI_GEOM_PRED_ALPHA,
    "beta": URI_GEOM_PRED_BETA,
    "gamma": URI_GEOM_PRED_GAMMA,
}
_DIRECTION_COSINE_PREDICATES = (
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
)


def path_geometry_node(path: ContextQuantity) -> URIRef:
    """The node carrying a path's shape and that shape's inputs."""
    value = path.value
    if value.lerp is not None:
        shape = "lerp"
    elif value.circle is not None:
        shape = "circle"
    elif value.arc is not None:
        shape = "arc"
    elif value.helix is not None:
        shape = "helix"
    else:
        shape = "figure8"
    return declared_uri(f"{shape}-{path.name}", path)


def add_literal_list_once(em: Emission, node: URIRef, predicate: URIRef, values: tuple) -> None:
    """An RDF list mints fresh blank nodes, so a shared context emitted per motion lists it once."""
    if (node, predicate, None) not in em.graph:
        add_literal_list_pred(em.graph, node, predicate, values)


def emit_duration_measure(em: Emission, node: URIRef, value: Measure) -> None:
    """A time:Duration whose magnitude qudt carries, in the unit it was written in.

    time:unitType bottoms out at seconds, so `10 ms` would have to be rescaled to be said there.
    """
    em.graph.add((node, RDF.type, TIME.Duration))
    emit_scalar_quantity(em, node, value.value, NS_MM_QUDT_QTY["Time"], DSL_UNITS[value.unit].iri)


def emit_context_members(em: Emission, block_node: URIRef, block: Any) -> None:
    """A block's own declared quantities as members of it; references and aliases declare none."""
    em.graph.add((block_node, RDF.type, PROV.Collection))
    for ctx in block.context:
        if isinstance(ctx, ContextDeclReference):
            continue
        for item in ctx.declaration:
            if not isinstance(item, ContextQuantityAlias):
                em.graph.add((block_node, PROV.hadMember, URIRef(item.uri)))


def emit_world_quantities(em: Emission, world_qtys: dict[str, WorldQuantity]) -> None:
    """Each world quantity's typing, frames, and how it is observed."""
    for qty in world_qtys.values():
        spec = WORLD_SPECS.get(qty.type)
        if spec is None:
            continue
        node = URIRef(qty.uri)
        props = qty.props
        # What the run observes, as opposed to what the model states.
        em.graph.add((node, RDF.type, SOSA.ObservableProperty))
        if qty.type == WorldQuantityType.Pose:
            em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
            em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
            of_node, wrt_node, seen_by_node = (owned_uri(em, name, qty) for name in pose_frame_names(qty))
            em.frame_coords[node] = (of_node, wrt_node, seen_by_node)
            pose_relation = emit_geom_relation(
                em, node, "pose", of_node, wrt_node, seen_by_node, (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH)
            )
            emit_combined_pose_coordinate(em, node, pose_relation)
            continue
        if qty.type == WorldQuantityType.Wrench:
            ft_sensor = geo_prop_value(props, "ft-sensor")
            estimated = next(
                (pair for pair in props.pairs if pair.key == "estimated-from" and pair.agent is not None),
                None,
            ) if props is not None else None
            sensor_frame = str(ft_sensor.frame.uri) if ft_sensor is not None else None
            reference_name = geo_prop(props, "ref-point") or sensor_frame
            reference_point = (
                owned_uri(em, reference_name, qty)
                if reference_name
                else declared_uri(f"point-{qty.name}-origin", qty)
            )
            em.graph.add((reference_point, RDF.type, GEOM_ENT.Point))
            acts_on = geo_prop(props, "of")
            emit_wrench_coordinate(
                em,
                node,
                reference_point,
                owned_uri(em, geo_prop(props, "as-seen-by") or sensor_frame, qty),
                owned_uri(em, acts_on, qty) if acts_on else None,
            )
            if ft_sensor is not None:
                observation = URIRef(f"{node}-observation")
                em.graph.add((observation, RDF.type, SOSA.Observation))
                em.graph.add((observation, SOSA.observedProperty, node))
                em.graph.add((observation, SOSA.madeBySensor, URIRef(str(ft_sensor.uri))))
            if estimated is not None:
                observer = URIRef(f"{node}-observer")
                em.graph.add((observer, RDF.type, EST.MomentumObserver))
                em.graph.add((observer, AGN["of-agent"], URIRef(str(estimated.agent.uri))))
                em.graph.add((node, EST["estimated-by"], observer))
                gain = emit_scalar_quantity(
                    em, URIRef(f"{observer}-gain"), estimated.observer.gain, QUDT_QKIND.Frequency, QUDT_UNIT.HZ
                )
                em.graph.add((observer, EST["estimation-gain"], gain))
                filter_constant = emit_scalar_quantity(
                    em,
                    URIRef(f"{observer}-filter"),
                    estimated.observer.filter,
                    QUDT_QKIND.Dimensionless,
                    QUDT_UNIT.UNITLESS,
                )
                em.graph.add((observer, EST["filter-constant"], filter_constant))
            # The tare samples what the source reads unloaded, on each named occurrence.
            for event in geo_prop_events(props, "re-tare-on"):
                schedule_node = URIRef(f"{node}-retare-{event.name}-schedule")
                em.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_AFTER_EVT))
                em.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_TC))
                em.graph.add((schedule_node, URI_TIME_PRED_OF_CONSTRAINT, node))
                em.graph.add((schedule_node, URI_TIME_PRED_AFTER_EVT, URIRef(event.uri)))
            continue
        for rdf_type in spec.rdf_types:
            em.graph.add((node, RDF.type, rdf_type))
        for qkind in spec.kinds:
            emit_quantity_kind(em, node, qkind)
        for unit in spec.units:
            em.graph.add((node, QUDT_SCHEMA.unit, unit))
        of_v = geo_prop(props, "of")
        wrt_v = geo_prop(props, "wrt")
        rp_v = geo_prop(props, "ref-point")
        asb_v = geo_prop(props, "as-seen-by")
        joint = geo_prop(props, "joint")
        if qty.type in JOINT_TYPES and joint:
            em.graph.add((node, KC_STAT["of-joint"], owned_uri(em, joint, qty)))
        normalization = geo_prop_value(props, "normalization")
        if qty.type == WorldQuantityType.JointPosition and normalization is not None:
            emit_angle_normalization(em, node, normalization, qty, f"norm-{qty.name}")
        if of_v:
            em.graph.add((node, GEOM_REL.of, owned_uri(em, of_v, qty)))
        if wrt_v:
            em.graph.add((node, GEOM_REL["with-respect-to"], owned_uri(em, wrt_v, qty)))
        if rp_v:
            em.graph.add((node, GEOM_REL["reference-point"], owned_uri(em, rp_v, qty)))
        elif qty.type == WorldQuantityType.VelocityTwist:
            point_node = declared_uri(f"point-{qty.name}-origin", qty)
            em.graph.add((point_node, RDF.type, GEOM_ENT.Point))
            em.graph.add((node, GEOM_REL["reference-point"], point_node))
        seen_by = asb_v or (wrt_v if qty.type == WorldQuantityType.VelocityTwist else None)
        if seen_by:
            em.graph.add((node, GEOM_COORD["as-seen-by"], owned_uri(em, seen_by, qty)))


def emit_context_composite_metadata(em: Emission, node: URIRef, quantity: ContextQuantity) -> None:
    """The typing, kinds, units and frames of a twist, wrench, pose or acceleration context value."""
    world_type = CONTEXT_COMPOSITE_WORLD_TYPE.get(quantity.type)
    if world_type is None and quantity.type != QuantityType.AccelerationTwist:
        return
    if quantity.type == QuantityType.Pose:
        em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        emit_combined_pose_coordinate(em, node, emit_declared_pose_frame_metadata(em, node, quantity))
        return
    # A snapshot or reference with no frames of its own is stated in its source's.
    props, owner = quantity.props, quantity
    if props is None and isinstance(quantity.value, (SnapshotValue, ReferenceValue)):
        source = quantity.value.source.quantity
        if isinstance(source, WorldQuantity):
            props, owner = source.props, source
    if quantity.type == QuantityType.Wrench:
        reference_name = geo_prop(props, "ref-point")
        reference_point = (
            owned_uri(em, reference_name, owner)
            if reference_name
            else declared_uri(f"point-{quantity.name}-origin", quantity)
        )
        em.graph.add((reference_point, RDF.type, GEOM_ENT.Point))
        acts_on = geo_prop(props, "of")
        emit_wrench_coordinate(
            em,
            node,
            reference_point,
            owned_uri(em, geo_prop(props, "as-seen-by"), owner),
            owned_uri(em, acts_on, owner) if acts_on else None,
        )
        return
    if world_type is not None:
        spec = WORLD_SPECS[world_type]
        for rdf_type in spec.rdf_types:
            em.graph.add((node, RDF.type, rdf_type))
        for qkind in spec.kinds:
            em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, qkind))
        for unit in spec.units:
            em.graph.add((node, QUDT_SCHEMA.unit, unit))
    else:
        em.graph.add((node, RDF.type, GEOM_REL.AccelerationTwist))
        em.graph.add((node, RDF.type, GEOM_COORD.AccelerationTwistCoordinate))
        em.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
        em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.AngularAcceleration))
        em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.LinearAcceleration))
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["RAD-PER-SEC2"]))
        em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC2"]))
    of_v = geo_prop(props, "of")
    wrt_v = geo_prop(props, "wrt")
    rp_v = geo_prop(props, "ref-point")
    asb_v = geo_prop(props, "as-seen-by") or wrt_v
    if of_v:
        em.graph.add((node, GEOM_REL.of, owned_uri(em, of_v, owner)))
    if wrt_v:
        em.graph.add((node, GEOM_REL["with-respect-to"], owned_uri(em, wrt_v, owner)))
    point_node = (
        owned_uri(em, rp_v, owner) if rp_v else declared_uri(f"point-{quantity.name}-origin", quantity)
    )
    em.graph.add((point_node, RDF.type, GEOM_ENT.Point))
    em.graph.add((node, GEOM_REL["reference-point"], point_node))
    if asb_v:
        em.graph.add((node, GEOM_COORD["as-seen-by"], owned_uri(em, asb_v, owner)))


def emit_coordinate_components(
    em: Emission,
    container_node: URIRef,
    superobject: URIRef,
    coords,
    labels: list[str],
    component_kind: Any,
    subspace: URIRef,
    unit: str | None,
    quantity: ContextQuantity,
    name_prefix: str,
) -> None:
    """Literal components on the coordinate, or, when any references a quantity, one view per axis.

    UNIT is the authored one, never rescaled; None means dimensionless, as a quaternion's.
    """
    # A wrench is its own container for both subspaces; its unit pair is the caller's.
    if unit is not None and container_node != superobject:
        em.graph.remove((container_node, QUDT_SCHEMA.unit, None))
        em.graph.add((container_node, QUDT_SCHEMA.unit, DSL_UNITS[unit].iri))
    elements = list(zip(labels, coords.values))
    if all(element.ref is None for _, element in elements):
        for label, element in elements:
            em.graph.add(
                (container_node, _COORDINATE_PREDICATES[label], Literal(float(element.value), datatype=XSD.double))
            )
        return
    for label, element in elements:
        if element.ref is not None:
            subobject = emit_context_ref_node(em, element.ref, quantity, label)
        else:
            subobject = URIRef(f"{name_prefix}.{label}")
            if unit is None:
                em.graph.add((subobject, RDF.type, QUDT_SCHEMA.Quantity))
                emit_quantity_kind(em, subobject, QUDT_KIND_BY_QUANTITY_TYPE[component_kind])
            else:
                add_quantity(em, subobject, component_kind)
                em.graph.remove((subobject, QUDT_SCHEMA.unit, None))
                em.graph.add((subobject, QUDT_SCHEMA.unit, DSL_UNITS[unit].iri))
            em.graph.add((subobject, QUDT_SCHEMA.value, Literal(float(element.value), datatype=XSD.double)))
        view = URIRef(f"{name_prefix}.{label}-view")
        emit_view(em, view)
        em.graph.add((view, MAP.superobject, superobject))
        em.graph.add((view, MAP.subobject, subobject))
        em.graph.add((view, MAP.subspace, subspace))
        em.graph.add((view, MAP.axis, MAP[label]))


def emit_pose_parts(
    em: Emission, node: URIRef, quantity: ContextQuantity, pose_relation: URIRef | None, orientation
) -> tuple[URIRef, URIRef]:
    """A stated pose's position and orientation coordinates, their relations, and their views."""
    position_node = URIRef(f"{quantity.uri}.position")
    orientation_node = URIRef(f"{quantity.uri}.orientation")
    em.graph.add((position_node, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((position_node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    em.graph.add((position_node, QUDT_SCHEMA.unit, QUDT_UNIT.M))
    em.graph.add((orientation_node, RDF.type, QUDT_SCHEMA.Quantity))
    if orientation is None or orientation.relative is None:
        emit_orientation_type(em, orientation_node, orientation)
    pose_of, pose_wrt, pose_asb = em.frame_coords.get(node, (None, None, None))
    position_relation = emit_geom_relation(
        em,
        position_node,
        "position",
        frame_origin(em, pose_of) if pose_of is not None else None,
        frame_origin(em, pose_wrt) if pose_wrt is not None else None,
        pose_asb or pose_wrt,
        (URI_QUDT_QK_LENGTH,),
    )
    orientation_relation = emit_geom_relation(
        em, orientation_node, "orientation", pose_of, pose_wrt, pose_asb or pose_wrt
    )
    record_pose_component(em, node, "position", position_node)
    record_pose_component(em, node, "orientation", orientation_node)
    if pose_relation is not None:
        em.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_POSITION_REF))
        em.graph.add((pose_relation, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
        em.graph.add((pose_relation, URI_GEOM_PRED_OF_POSITION, position_relation))
        em.graph.add((pose_relation, URI_GEOM_PRED_OF_ORIENT, orientation_relation))
    for coord, subobject, subspace, label in (
        (position_node, position_relation, MAP_EXT.position, "position"),
        (orientation_node, orientation_relation, MAP_EXT.orientation, "orientation"),
    ):
        # Named from this quantity: the relation pools by frame pair, and each pose needs its view.
        view = URIRef(f"{node}-{label}-view")
        emit_view(em, view)
        em.graph.add((view, RDF.type, MAP_EXT.PoseCoordinateView))
        em.graph.add((view, MAP.superobject, node))
        em.graph.add((view, MAP.subobject, subobject))
        em.graph.add((view, MAP.subspace, subspace))
        em.component_views.setdefault((coord, label), view)
    return position_node, orientation_node


def emit_config_pose_quantity(em: Emission, node: URIRef, quantity: ContextQuantity) -> None:
    """A pose the deployment's config file states: a literal pose's relation and views, minus values.

    Its frames are those of the quantity it is stated for, and its key in the file rides on
    schema:identifier. The file is read once, so the declaration must be a shared one.
    """
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT.UNITLESS))
    em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS["m"].iri))
    em.graph.add((node, EXEC["has-resource"], em.config_resource))
    em.graph.add((node, SDO.identifier, Literal(quantity.value.key)))
    pose_relation = emit_declared_pose_frame_metadata(em, node, quantity)
    emit_pose_parts(em, node, quantity, pose_relation, None)


def emit_pose_value_quantity(
    em: Emission, node: URIRef, quantity: ContextQuantity, constraints: list, world_qtys: dict
) -> None:
    """A literal pose: its relation and coordinate, its parts by map:View, and each part's values.

    A pose stating no frames takes those `pose_frame_source` finds; validation ensures there are some.
    """
    value = quantity.value
    position, orientation = value.position, value.orientation
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    em.graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[position.unit or "m"].iri))
    angle_unit = orientation.euler.unit if orientation.euler is not None else "rad"
    em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[angle_unit].iri))
    pose_relation = emit_declared_pose_frame_metadata(em, node, quantity)
    if pose_relation is None:
        frames = pose_frame_source(quantity, constraints, world_qtys)
        frame_nodes = tuple(owned_uri(em, name, frames[1]) for name in frames[0])
        em.frame_coords[node] = frame_nodes
        pose_relation = emit_geom_relation(
            em, node, "pose", *frame_nodes, (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH)
        )
    position_node, orientation_node = emit_pose_parts(em, node, quantity, pose_relation, orientation)
    for part, ref, subspace, part_node in (
        ("position", position.ref, MAP_EXT.position, position_node),
        ("orientation", orientation.ref if orientation.relative is None else None, MAP_EXT.orientation, orientation_node),
    ):
        if ref is None:
            continue
        view = URIRef(f"{part_node}-ref-view")
        emit_view(em, view)
        em.graph.add((view, RDF.type, MAP_EXT.PoseCoordinateView))
        em.graph.add((view, MAP.superobject, node))
        em.graph.add((view, MAP.subobject, emit_context_ref_node(em, ref, quantity, part)))
        em.graph.add((view, MAP.subspace, subspace))
    if position.ref is None:
        emit_coordinate_components(
            em,
            position_node,
            node,
            position.coords,
            ["x", "y", "z"],
            QuantityType.Distance,
            MAP_EXT.position,
            position.unit,
            quantity,
            f"{quantity.uri}.position",
        )
    if orientation.relative is not None:
        emit_relative_orientation(em, orientation_node, orientation.relative, quantity)
    elif orientation.ref is not None:
        return
    elif orientation.quat is not None:
        emit_coordinate_components(
            em,
            orientation_node,
            node,
            orientation.quat.xyzw,
            ["x", "y", "z", "w"],
            QuantityType.Dimensionless,
            MAP_EXT.orientation,
            None,
            quantity,
            f"{quantity.uri}.orientation",
        )
    elif orientation.direction_cosine is not None:
        cosine = orientation.direction_cosine
        for coords, predicate in zip(
            (cosine.x_axis, cosine.y_axis, cosine.z_axis), _DIRECTION_COSINE_PREDICATES
        ):
            add_literal_list_once(em, orientation_node, predicate, tuple(float(e.value) for e in coords.values))
    else:
        euler = orientation.euler
        symbolic = any(element.ref is not None for element in euler.angles.values)
        emit_coordinate_components(
            em,
            orientation_node,
            node,
            euler.angles,
            list(euler.axes) if symbolic else ["alpha", "beta", "gamma"],
            QuantityType.Angle,
            MAP_EXT.orientation,
            euler.unit,
            quantity,
            f"{quantity.uri}.orientation",
        )


def emit_relative_orientation(em: Emission, orientation_node: URIRef, relative, quantity) -> None:
    """An orientation composed from a base and a delta by `geom-op-ext:ComposeOrientation`.

    The base is never decomposed. The delta's basis decides the order: intrinsic Euler angles turn
    in the base's body frame (base * delta), extrinsic ones in its reference basis (delta * base).
    """
    base = relative.base.quantity
    base_node = URIRef((base.ref if isinstance(base, ContextQuantityAlias) else base).uri)
    of_frame, _wrt_frame, base_as_seen_by = pose_frame_names(base)
    of_frame_node = owned_uri(em, of_frame, base)
    base_as_seen_by_node = owned_uri(em, base_as_seen_by, base)
    # Bound to the pose itself: the backend reads the composed rotation off the frame.
    em.graph.add((orientation_node, GEOM_REL.of, of_frame_node))
    em.graph.add((orientation_node, GEOM_COORD["as-seen-by"], base_as_seen_by_node))
    if relative.frame is not None:
        delta_basis = owned_uri(em, str(relative.frame.uri), quantity)
    elif relative.euler.extrinsic:
        delta_basis = base_as_seen_by_node
    else:
        delta_basis = of_frame_node
    delta_node = URIRef(f"{quantity.uri}.orientation-delta")
    em.graph.add((delta_node, RDF.type, QUDT_SCHEMA.Quantity))
    emit_orientation_type(em, delta_node, relative)
    in1, in2 = (base_node, delta_node) if delta_basis == of_frame_node else (delta_node, base_node)
    composition = URIRef(f"{quantity.uri}.orientation-composition")
    em.graph.add((composition, RDF.type, GEOM_OP_EXT.ComposeOrientation))
    em.graph.add((composition, GEOM_OP.in1, in1))
    em.graph.add((composition, GEOM_OP.in2, in2))
    em.graph.add((composition, GEOM_OP.composite, orientation_node))
    if relative.direction_cosine is not None:
        cosine = relative.direction_cosine
        for coords, predicate in zip(
            (cosine.x_axis, cosine.y_axis, cosine.z_axis), _DIRECTION_COSINE_PREDICATES
        ):
            add_literal_list_once(em, delta_node, predicate, tuple(float(e.value) for e in coords.values))
        return
    if relative.quat is not None:
        coords, labels, unit = relative.quat.xyzw, ("x", "y", "z", "w"), None
    else:
        coords, labels, unit = relative.euler.angles, ("alpha", "beta", "gamma"), relative.euler.unit
    if unit is not None:
        em.graph.remove((delta_node, QUDT_SCHEMA.unit, None))
        em.graph.add((delta_node, QUDT_SCHEMA.unit, DSL_UNITS[unit].iri))
    for label, element in zip(labels, coords.values):
        em.graph.add(
            (delta_node, _COORDINATE_PREDICATES[label], Literal(float(element.value), datatype=XSD.double))
        )


def emit_two_subspace_coordinate(em: Emission, node: URIRef, quantity: ContextQuantity) -> None:
    """A literal twist or wrench: its two named subspace vectors, each in its own unit."""
    value = quantity.value
    if isinstance(value, VelocityTwistCoordinate):
        subspaces = (
            ("angular-velocity", value.angular, value.angular_unit, GEOM_COORD["angular-velocity"], QuantityType.AngularVelocity),
            ("linear-velocity", value.linear, value.linear_unit, GEOM_COORD["linear-velocity"], QuantityType.LinearVelocity),
        )
    elif isinstance(value, AccelerationTwistCoordinate):
        subspaces = (
            ("angular-acceleration", value.angular, value.angular_unit, GEOM_COORD["angular-acceleration"], QuantityType.AngularAcceleration),
            ("linear-acceleration", value.linear, value.linear_unit, GEOM_COORD["linear-acceleration"], QuantityType.LinearAcceleration),
        )
    else:
        subspaces = (
            ("torque", value.torque, value.torque_unit, RBDYN_COORD.torque, QuantityType.Torque),
            ("force", value.force, value.force_unit, RBDYN_COORD.force, QuantityType.Force),
        )
    # The container's unit pair says which scale its subspace numbers are on.
    em.graph.remove((node, QUDT_SCHEMA.unit, None))
    for _label, _coords, unit, _predicate, _kind in subspaces:
        em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[unit].iri))
    for label, coords, unit, predicate, kind in subspaces:
        if isinstance(value, WrenchCoordinate) and all(element.ref is None for element in coords.values):
            add_literal_list_once(em, node, predicate, tuple(float(e.value) for e in coords.values))
            continue
        container = node
        if not isinstance(value, WrenchCoordinate):
            container = URIRef(f"{quantity.uri}.{label}")
            em.graph.add((container, RDF.type, QUDT_SCHEMA.Quantity))
            em.graph.add((container, RDF.type, GEOM_COORD.VectorXYZ))
            emit_quantity_kind(em, container, QUDT_KIND_BY_QUANTITY_TYPE[kind])
            em.graph.add((container, QUDT_SCHEMA.unit, SCALAR_UNIT.get(kind, QUDT_UNIT.UNITLESS)))
            em.graph.add((node, predicate, container))
        emit_coordinate_components(
            em, container, node, coords, ["x", "y", "z"], kind, MAP[label], unit, quantity, f"{quantity.uri}.{label}"
        )


def emit_direction_quantity(em: Emission, node: URIRef, quantity: ContextQuantity, world_qtys: dict) -> None:
    """A direction: a unit vector in its as-seen-by frame, literal or computed from a pose each cycle."""
    as_seen_by_name = geo_prop(quantity.props, "as-seen-by") or geo_prop(quantity.props, "wrt")
    as_seen_by = owned_uri(em, as_seen_by_name, quantity)
    em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    # A direction is itself the structural unit vector a line or plane names.
    em.graph.add((node, RDF.type, GEOM_ENT.UnitVector))
    value = quantity.value
    if not isinstance(value, DirectionBetween):
        vector = tuple(float(e.value) for e in value.coords.values) if isinstance(value, VectorXYZ) else None
        emit_direction_coordinate(em, node, as_seen_by, vector)
        return
    emit_direction_coordinate(em, node, as_seen_by)
    # The pose relating the two frames is the model's to declare: normalizing its translation
    # is the direction.
    pose = existing_world_pose(world_qtys, str(value.to_frame.uri), str(value.from_frame.uri))
    op_node = declared_uri("compute-direction", quantity)
    em.graph.add((op_node, RDF.type, GEOM_OP.PoseToDirection))
    em.graph.add((op_node, GEOM_OP.pose, URIRef(pose.uri)))
    em.graph.add((op_node, GEOM_OP.direction, node))


def emit_structural_primitive(em: Emission, node: URIRef, quantity: ContextQuantity) -> None:
    """A line or plane: its `of` frame's origin and the unit vector its direction quantity carries."""
    is_plane = quantity.type == QuantityType.Plane
    vector_name = geo_prop(quantity.props, GeometricPropKey.Normal if is_plane else GeometricPropKey.Along)
    vector_node = owned_uri(em, vector_name, quantity)
    em.graph.add((vector_node, RDF.type, GEOM_ENT.UnitVector))
    em.graph.add((node, RDF.type, GEOM_EXT.Plane if is_plane else GEOM_EXT.Line))
    frame = owned_uri(em, geo_prop(quantity.props, GeometricPropKey.Of), quantity)
    em.graph.add((node, URI_GEOM_PRED_ORIGIN, frame_origin(em, frame)))
    em.graph.add((node, GEOM_EXT.normal if is_plane else GEOM_EXT.direction, vector_node))


def emit_path_quantity(em: Emission, quantity: ContextQuantity) -> None:
    """A path's geometry and the setpoint its evaluator produces; traversal is the driver's."""
    value = quantity.value
    if value.lerp is not None:
        path_type = GEOM_PATH.LinearPath
        inputs = [("start", GEOM_PATH.start, value.lerp.start), ("goal", GEOM_PATH.goal, value.lerp.goal)]
        start = value.lerp.start.quantity
        # Start and goal share their kind.
        value_kind = QUDT_KIND_BY_QUANTITY_TYPE.get(start.type) if start is not None else None
    elif value.circle is not None:
        path_type, value_kind = GEOM_PATH.Circle, URI_GEOM_TYPE_POSE
        inputs = [
            ("start", GEOM_PATH.start, value.circle.start),
            ("center", GEOM_PATH.center, value.circle.center),
            ("plane-normal", GEOM_PATH["plane-normal"], value.circle.plane_normal),
        ]
    elif value.arc is not None:
        path_type, value_kind = GEOM_PATH.Arc, URI_GEOM_TYPE_POSE
        inputs = [
            ("start", GEOM_PATH.start, value.arc.start),
            ("end", GEOM_PATH.end, value.arc.end),
            ("amplitude", GEOM_PATH.amplitude, value.arc.amplitude),
            ("plane-normal", GEOM_PATH["plane-normal"], value.arc.plane_normal),
        ]
    elif value.helix is not None:
        path_type, value_kind = GEOM_PATH.Helix, URI_GEOM_TYPE_POSE
        inputs = [
            ("start", GEOM_PATH.start, value.helix.start),
            ("center", GEOM_PATH.center, value.helix.center),
            ("axis", GEOM_PATH.axis, value.helix.axis),
            ("pitch", GEOM_PATH.pitch, value.helix.pitch),
            ("revolutions", GEOM_PATH.revolutions, value.helix.revolutions),
        ]
    else:
        path_type, value_kind = GEOM_PATH.Figure8, URI_GEOM_TYPE_POSE
        inputs = [
            ("anchor", GEOM_PATH.anchor, value.figure8.anchor),
            ("radius", GEOM_PATH.radius, value.figure8.radius),
            ("plane-normal", GEOM_PATH["plane-normal"], value.figure8.plane_normal),
        ]
    reference = URIRef(f"{quantity.uri}/reference")
    em.graph.add((reference, RDF.type, QUDT_SCHEMA.Quantity))
    if value_kind is not None and value_kind != URI_GEOM_TYPE_POSE:
        em.graph.add((reference, QUDT_SCHEMA.hasQuantityKind, value_kind))
    if value_kind == URI_GEOM_TYPE_POSE:
        em.graph.add((reference, QUDT_SCHEMA.unit, QUDT_UNIT.RAD))
        em.graph.add((reference, QUDT_SCHEMA.unit, QUDT_UNIT.M))
        pose_relation = emit_declared_pose_frame_metadata(em, reference, quantity)
        if pose_relation is None:
            # The one frame tuple the path's pose endpoints share.
            endpoint, frames = next(
                (endpoint, frames)
                for endpoint in path_pose_endpoints(quantity)
                if (frames := pose_frame_names(endpoint)) is not None
            )
            frame_nodes = tuple(owned_uri(em, name, endpoint) for name in frames)
            em.frame_coords[reference] = frame_nodes
            pose_relation = emit_geom_relation(
                em, reference, "pose", *frame_nodes, (QUDT_QKIND.PlaneAngle, URI_QUDT_QK_LENGTH)
            )
        emit_combined_pose_coordinate(em, reference, pose_relation)
    path_node = path_geometry_node(quantity)
    em.graph.add((path_node, RDF.type, GEOM_PATH.Path))
    em.graph.add((path_node, RDF.type, path_type))
    for suffix, predicate, ref in inputs:
        em.graph.add((path_node, predicate, emit_context_ref_node(em, ref, quantity, suffix)))
    if value.figure8 is not None:
        em.graph.add((path_node, GEOM_PATH.form, GEOM_PATH[value.figure8.form]))


def emit_context_quantities(
    em: Emission, context_quantities: dict[str, ContextQuantity], constraints: list, world_qtys: dict
) -> None:
    """Every context quantity: its kind, unit, frames, and value or how it is computed."""
    for quantity in context_quantities.values():
        node = URIRef(quantity.uri)
        value = quantity.value
        if quantity.type == ReferenceGeneratorType.Path:
            emit_path_quantity(em, quantity)
            continue
        if quantity.type == QuantityType.Direction:
            emit_direction_quantity(em, node, quantity, world_qtys)
            continue
        if quantity.type in (QuantityType.Line, QuantityType.Plane):
            emit_structural_primitive(em, node, quantity)
            continue
        # A profile's or an admittance's op is emitted where a constraint binds it.
        if quantity.type in (ReferenceGeneratorType.VelocityProfile, ReferenceGeneratorType.Admittance):
            if isinstance(value, (ProfileSpec, AdmittanceSpec)):
                em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
                em.graph.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND.LinearVelocity))
                em.graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT["M-PER-SEC"]))
            continue
        if isinstance(value, SampledValue):
            # The number is drawn per generation downstream.
            if quantity.type == QuantityType.Duration:
                em.graph.add((node, RDF.type, TIME.Duration))
                qkind = NS_MM_QUDT_QTY["Time"]
            else:
                qkind = QUDT_KIND_BY_QUANTITY_TYPE[quantity.type]
            em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
            emit_quantity_kind(em, node, qkind)
            em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[value.unit].iri))
            add_sampled_quantity(em.graph, node, DistributionRef(parent=None, distribution=value.distribution))
            continue
        if quantity.type == QuantityType.Duration and isinstance(value, Measure):
            emit_duration_measure(em, node, value)
            continue
        if isinstance(value, ConfigValue):
            emit_config_pose_quantity(em, node, quantity)
            continue
        if isinstance(value, PoseCoordinate):
            emit_pose_value_quantity(em, node, quantity, constraints, world_qtys)
            continue
        kind_unit = SCALAR_UNIT.get(quantity.type, QUDT_UNIT.UNITLESS)
        em.graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
        if quantity.type != QuantityType.Pose:
            emit_quantity_kind(em, node, QUDT_KIND_BY_QUANTITY_TYPE[quantity.type])
        if quantity.type == QuantityType.Orientation:
            em.graph.add((node, RDF.type, GEOM_REL.Orientation))
            em.graph.add((node, RDF.type, GEOM_COORD.OrientationCoordinate))
            emit_orientation_type(em, node, None)
        emit_context_composite_metadata(em, node, quantity)
        if value is None:
            if quantity.type == QuantityType.PathParameter:
                em.graph.add((node, QUDT_SCHEMA.value, Literal(0.0, datatype=XSD.double)))
            continue
        if isinstance(value, ReferenceValue):
            tree = op_tree(value.expr)
            if isinstance(tree, QOpNode):
                emit_op_tree(em, tree, quantity, node, node)
                if quantity in em.driven_quantities:
                    emit_expression_gradient(em, tree, node, quantity, world_qtys)
            else:
                em.graph.add((node, CSTR["reference-value"], emit_context_ref_node(em, value.source, quantity, "source")))
            em.graph.add((node, QUDT_SCHEMA.unit, kind_unit))
            continue
        if isinstance(value, SnapshotValue):
            # A snapshot derives from another quantity: no sosa:Observation, which needs a sensor.
            source_view = view_node(em, value.source, quantity)
            tree = op_tree(value)
            snap_source = source_view
            if isinstance(tree, QOpNode):
                # Owned by this quantity's IRI: two motions' same-named quantities keep their ops apart.
                out_node = URIRef(f"{node}-{tree.op}-out")
                emit_op_tree(em, tree, quantity, out_node, out_node)
                em.graph.add((out_node, RDF.type, QUDT_SCHEMA.Quantity))
                # A pose's kind and unit pair come with its geometry below.
                if quantity.type != QuantityType.Pose:
                    emit_quantity_kind(em, out_node, QUDT_KIND_BY_QUANTITY_TYPE[quantity.type])
                    em.graph.add((out_node, QUDT_SCHEMA.unit, kind_unit))
                # A position or pose offset keeps its source's frames; a scalar offset stays scalar.
                if quantity.type in {QuantityType.Position, QuantityType.Pose}:
                    emit_snapshot_geometry_metadata(em, out_node, quantity)
                snap_source = out_node
            em.graph.add((node, PROV.wasDerivedFrom, snap_source))
            schedule_node = URIRef(f"{node}-schedule")
            em.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_AFTER_EVT))
            em.graph.add((schedule_node, RDF.type, URI_TIME_TYPE_TC))
            em.graph.add((schedule_node, URI_TIME_PRED_OF_CONSTRAINT, node))
            em.graph.add((schedule_node, URI_TIME_PRED_AFTER_EVT, URIRef(value.trigger.uri)))
            em.graph.add((node, QUDT_SCHEMA.unit, kind_unit))
            if quantity.type == QuantityType.Pose:
                emit_declared_pose_frame_metadata(em, node, quantity)
            elif quantity.type == QuantityType.Position:
                emit_snapshot_geometry_metadata(em, node, quantity)
            continue
        if isinstance(value, (VelocityTwistCoordinate, AccelerationTwistCoordinate, WrenchCoordinate)):
            emit_two_subspace_coordinate(em, node, quantity)
            continue
        # A scalar the run computes: its ops are emitted when something first asks for it.
        if isinstance(value, DerivedScalarValue):
            em.graph.add((node, QUDT_SCHEMA.unit, kind_unit))
            em.derived_scalar_declarations.append((quantity, node))
            continue
        em.graph.add((node, QUDT_SCHEMA.unit, DSL_UNITS[value.unit].iri))
        if isinstance(value, Measure):
            em.graph.add((node, QUDT_SCHEMA.value, Literal(float(value.value), datatype=XSD.double)))
        elif isinstance(value, VectorXYZ):
            em.graph.add((node, RDF.type, GEOM_COORD.VectorXYZ))
            for label, element in zip(("x", "y", "z"), value.coords.values):
                component = (
                    emit_context_ref_node(em, element.ref, quantity, label)
                    if element.ref is not None
                    else Literal(float(element.value), datatype=XSD.double)
                )
                em.graph.add((node, GEOM_COORD[label], component))
