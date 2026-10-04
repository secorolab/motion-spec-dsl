# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Quantity expressions as ALGO operator chains, and the nodes context references resolve to."""

from __future__ import annotations

from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib.namespace import RDF
from rdflib.term import URIRef

from motion_spec_dsl.classes.context import (
    BODY_LINE_DISTANCE_OP,
    ConfigValue,
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    Measure,
    QOpNode,
    QuantityType,
    ReferenceGeneratorType,
    ReferenceValue,
    WorldQuantity,
    op_tree,
)
from motion_spec_dsl.classes.coordinates import PoseCoordinate
from motion_spec_dsl.classes.dimensions import infer, resolve_leaf
from motion_spec_dsl.classes.units import DSL_UNITS, QUDT_KIND_BY_QUANTITY_TYPE
from motion_spec_dsl.classes.views import POSE_KINDS, context_subspace_kind, leaf_gradient, owning_motion
from motion_spec_dsl.rdf.common import AXIS_VECTORS, ExpressionPlan, GeometricDistancePlan, gradient_scalar_id
from motion_spec_dsl.rdf.emission import (
    Emission,
    add_quantity,
    declared_uri,
    emit_quantity_kind,
    emit_scalar_quantity,
    emit_view,
    owned_uri,
)
from motion_spec_dsl.rdf.geometry import emit_direction_coordinate, view_node
from motion_spec_dsl.rdf.model import SCALAR_UNIT
from motion_spec_dsl.rdf.plans import derived_scalar_quantity, derived_scalar_spec
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    GEOM_COORD,
    GEOM_OP,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
)

# The map subspace of a pose or path's whole position or orientation.
_POSE_PART_SUBSPACES = {"position": MAP_EXT.position, "orientation": MAP_EXT.orientation}
# (view type, map subspace) per (context quantity kind, subspace token).
_COMPOSITE_PART_VIEWS = {
    (QuantityType.VelocityTwist, "linvel"): (MAP_EXT.VelocityTwistCoordinateView, MAP["linear-velocity"]),
    (QuantityType.VelocityTwist, "angvel"): (MAP_EXT.VelocityTwistCoordinateView, MAP["angular-velocity"]),
    (QuantityType.AccelerationTwist, "linacc"): (
        MAP_EXT.AccelerationTwistCoordinateView,
        MAP["linear-acceleration"],
    ),
    (QuantityType.AccelerationTwist, "angacc"): (
        MAP_EXT.AccelerationTwistCoordinateView,
        MAP["angular-acceleration"],
    ),
    (QuantityType.Wrench, "force"): (MAP_EXT.WrenchCoordinateView, MAP.force),
    (QuantityType.Wrench, "torque"): (MAP_EXT.WrenchCoordinateView, MAP.torque),
}
_OP_TYPES = {
    "add": ALGO_EXT.Addition,
    "multiply": ALGO_EXT.Multiplication,
    "subtract": ALGO_EXT.Subtraction,
    "divide": ALGO_EXT.Division,
}


def emit_op_tree(
    em: Emission, tree: QOpNode, owner: Any, stem: URIRef, out_node: URIRef, path: str = ""
) -> None:
    """The operator chain computing TREE into OUT_NODE; interior results are named by DFS position."""
    op_node = URIRef(f"{stem}-{tree.op}-{path}" if path else f"{stem}-{tree.op}")
    operand_nodes = []
    for index, operand in enumerate(tree.operands):
        child_path = f"{path}-{index}" if path else str(index)
        if not isinstance(operand, QOpNode):
            operand_nodes.append(emit_expr_leaf(em, operand, owner, f"expr-{child_path}"))
            continue
        operand_out = URIRef(f"{stem}-{operand.op}-{child_path}-out")
        emit_op_tree(em, operand, owner, stem, operand_out, child_path)
        qty_type = infer(operand)
        em.graph.add((operand_out, RDF.type, QUDT_SCHEMA.Quantity))
        emit_quantity_kind(em, operand_out, QUDT_KIND_BY_QUANTITY_TYPE[qty_type])
        em.graph.add((operand_out, QUDT_SCHEMA.unit, SCALAR_UNIT.get(qty_type, QUDT_UNIT.UNITLESS)))
        operand_nodes.append(operand_out)
    em.graph.add((op_node, RDF.type, _OP_TYPES[tree.op]))
    if tree.op == "subtract":
        em.graph.add((op_node, ALGO_EXT.minuend, operand_nodes[0]))
        em.graph.add((op_node, ALGO_EXT.subtrahend, operand_nodes[1]))
    elif tree.op == "divide":
        em.graph.add((op_node, ALGO_EXT.dividend, operand_nodes[0]))
        em.graph.add((op_node, ALGO_EXT.divisor, operand_nodes[1]))
    else:
        for operand_node in operand_nodes:
            em.graph.add((op_node, ALGO_EXT["in"], operand_node))
    em.graph.add((op_node, ALGO_EXT.out, out_node))


def emit_expr_leaf(em: Emission, leaf: Any, owner: Any, suffix: str) -> URIRef:
    """An expression leaf's node: a world quantity's view, or a context reference typed as inferred."""
    if isinstance(leaf.quantity, WorldQuantity):
        return view_node(em, leaf, owner)
    return emit_context_ref_node(em, leaf, owner, suffix, resolve_leaf(leaf))


def emit_context_ref_view_node(em: Emission, quantity: ContextQuantity, subspace: str | None, axis: str | None) -> URIRef:
    """The scalar and map:View for a subspace of a context quantity, emitted once.

    A whole position or orientation is named by its view; every other subspace by its scalar.
    """
    scalar_kind = context_subspace_kind(quantity, subspace, axis)
    if scalar_kind is None:
        return URIRef(quantity.uri)
    pose_part = quantity.type in POSE_KINDS and subspace in _POSE_PART_SUBSPACES
    # A bare axis names a component of a 3-vector: only the axis link tells it apart.
    if subspace is None:
        view_type, view_subspace = None, None
    elif pose_part:
        view_type, view_subspace = MAP_EXT.PoseCoordinateView, _POSE_PART_SUBSPACES[subspace]
    else:
        view_type, view_subspace = _COMPOSITE_PART_VIEWS[(quantity.type, subspace)]
    suffix = ".".join(part for part in (subspace, axis) if part is not None)
    node = URIRef(f"{quantity.uri}.{suffix}")
    path = quantity.type == ReferenceGeneratorType.Path
    super_node = URIRef(f"{quantity.uri}/reference") if path else URIRef(quantity.uri)
    view_target = node
    component_coord = None
    if pose_part and axis is None:
        # A pose stated coordinate-wise owns its parts; a snapshot or reference borrows its source's.
        owns_parts = isinstance(quantity.value, (PoseCoordinate, ConfigValue))
        component_coord = URIRef(f"{quantity.uri}.{subspace}") if owns_parts else super_node
        if (component_coord, subspace) in em.component_views:
            return em.component_views[(component_coord, subspace)]
        view_target = em.component_relations[(component_coord, subspace)]
    if view_target == node:
        add_quantity(em, node, scalar_kind)
    view = URIRef(f"{quantity.uri}.view-{suffix}")
    if view not in em.emitted_views:
        emit_view(em, view)
        if view_type is not None and (axis is None or quantity.type not in POSE_KINDS):
            em.graph.add((view, RDF.type, view_type))
        em.graph.add((view, MAP.superobject, super_node))
        em.graph.add((view, MAP.subobject, view_target))
        if view_subspace is not None:
            em.graph.add((view, MAP.subspace, view_subspace))
        if axis is not None:
            em.graph.add((view, MAP.axis, MAP[axis]))
    if component_coord is not None:
        em.component_views.setdefault((component_coord, subspace), view)
        return view
    return view_target


def emit_context_ref_node(
    em: Emission, ref: ContextRef, owner: Any, suffix: str, scalar_t: Any = None
) -> URIRef:
    """A context reference's node: its literal, its expression, a subspace view, or the quantity.

    SCALAR_T types a literal when OWNER is no typed quantity, a motion for a threshold.
    """
    if isinstance(ref.bare, Measure):
        kind = scalar_t if scalar_t is not None else owner.type
        return emit_scalar_quantity(
            em,
            declared_uri(suffix, owner),
            ref.bare.value,
            QUDT_KIND_BY_QUANTITY_TYPE.get(kind) or QUDT_QKIND[kind],
            DSL_UNITS[ref.bare.unit].iri,
        )
    # An expression leaf takes this path too, and has no expression of its own.
    if isinstance(ref, ContextRef) and ref.expr is not None:
        return emit_inline_expr(em, ref.expr, owner, suffix)
    quantity = ref.quantity
    if not isinstance(quantity, ContextQuantity):
        return owned_uri(em, quantity.name, owner)
    if isinstance(quantity, ContextQuantityAlias):
        quantity = quantity.ref
    axis = str(ref.axis) if ref.axis is not None else None
    if ref.subspace is not None or axis is not None:
        subspace = str(ref.subspace) if ref.subspace is not None else None
        return emit_context_ref_view_node(em, quantity, subspace, axis)
    value = quantity.value
    if isinstance(value, ReferenceValue) and not isinstance(op_tree(value.expr), QOpNode):
        return emit_context_ref_node(em, value.source, owner, suffix, scalar_t)
    if quantity.type == ReferenceGeneratorType.Path:
        return URIRef(f"{quantity.uri}/reference")
    return URIRef(quantity.uri)


def emit_inline_expr(em: Emission, expr: Any, owner: Any, suffix: str) -> URIRef:
    """A parenthesized expression in a reference slot: a quantity the consuming declaration owns."""
    tree = op_tree(expr)
    if not isinstance(tree, QOpNode):
        return emit_expr_leaf(em, tree, owner, suffix)
    inferred = infer(tree)
    root = owned_uri(em, suffix, owner)
    emit_op_tree(em, tree, owner, root, root)
    em.graph.add((root, RDF.type, QUDT_SCHEMA.Quantity))
    emit_quantity_kind(em, root, QUDT_KIND_BY_QUANTITY_TYPE[inferred])
    em.graph.add((root, QUDT_SCHEMA.unit, SCALAR_UNIT.get(inferred, QUDT_UNIT.UNITLESS)))
    return root


def _vector_op(em: Emission, op_type: URIRef, inputs: list, out: URIRef) -> URIRef:
    """OUT as the result of one ALGO operator over INPUTS, a direction in the frame of the first
    direction among them."""
    frame = next(
        frame for _predicate, node in inputs if (frame := em.graph.value(node, GEOM_COORD["as-seen-by"]))
    )
    emit_direction_coordinate(em, out, frame)
    op_node = URIRef(f"{out}-op")
    em.graph.add((op_node, RDF.type, op_type))
    for predicate, node in inputs:
        em.graph.add((op_node, predicate, node))
    em.graph.add((op_node, ALGO_EXT.out, out))
    return out


def _gradient_term(em: Emission, node: Any, part: str, stem: URIRef, path: str, owner: Any, world_qtys: dict):
    """NODE's gradient, or for PART `moment` its angular companion, emitted under STEM; None where
    the motion does not move NODE."""
    name = URIRef(f"{stem}-{part}-{path or 'root'}")
    if not isinstance(node, QOpNode):
        moved = leaf_gradient(node, world_qtys)
        if moved is None:
            return None
        frame = owned_uri(em, moved[0], owner)
        quantity = node.quantity.ref if isinstance(node.quantity, ContextQuantityAlias) else node.quantity
        if not isinstance(quantity, ContextQuantity):
            if part == "moment":
                return None
            emit_direction_coordinate(em, name, frame, AXIS_VECTORS[str(node.axis)])
            return name
        # A derived scalar's view publishes its gradient; a body-fixed line also its angular term.
        derived = derived_scalar_spec(em, quantity)
        gradient_id = gradient_scalar_id(derived_scalar_quantity(em, quantity, world_qtys), derived)
        plan = em.plans.get(derived)
        body_line = isinstance(plan, GeometricDistancePlan) and plan.op_type == BODY_LINE_DISTANCE_OP
        if part == "moment" and not body_line:
            return None
        term = owned_uri(em, gradient_id if part == "gradient" else f"{gradient_id}-moment", owning_motion(derived))
        emit_direction_coordinate(em, term, frame)
        return term
    operand_paths = [f"{path}-{index}" if path else str(index) for index in range(len(node.operands))]
    gradients = [
        _gradient_term(em, operand, part, stem, operand_path, owner, world_qtys)
        for operand, operand_path in zip(node.operands, operand_paths)
    ]
    # The value each operand computes into, named as `emit_op_tree` names it.
    values = [
        URIRef(f"{stem}-{operand.op}-{operand_path}-out")
        if isinstance(operand, QOpNode)
        else emit_expr_leaf(em, operand, owner, f"expr-{operand_path}")
        for operand, operand_path in zip(node.operands, operand_paths)
    ]
    if node.op == "multiply":
        # The product rule: each moved factor's gradient, scaled by every other factor.
        terms = [
            _vector_op(
                em,
                ALGO_EXT.Multiplication,
                [(ALGO_EXT["in"], other) for other in values[:index] + values[index + 1 :]]
                + [(ALGO_EXT["in"], gradient)],
                URIRef(f"{name}-product-{index}"),
            )
            for index, gradient in enumerate(gradients)
            if gradient is not None
        ]
    elif node.op == "divide" and gradients[0] is not None:
        # Validation keeps a moved term out of the divisor.
        terms = [
            _vector_op(
                em,
                ALGO_EXT.Division,
                [(ALGO_EXT.dividend, gradients[0]), (ALGO_EXT.divisor, values[1])],
                URIRef(f"{name}-quotient"),
            )
        ]
    elif node.op == "subtract" and gradients[1] is not None:
        minus_one = emit_scalar_quantity(
            em,
            URIRef(f"{stem}-minus-one"),
            -1.0,
            QUDT_KIND_BY_QUANTITY_TYPE[QuantityType.Dimensionless],
            QUDT_UNIT.UNITLESS,
        )
        negated = _vector_op(
            em,
            ALGO_EXT.Multiplication,
            [(ALGO_EXT["in"], minus_one), (ALGO_EXT["in"], gradients[1])],
            URIRef(f"{name}-negated"),
        )
        terms = [term for term in (gradients[0], negated) if term is not None]
    else:
        terms = [gradient for gradient in gradients if gradient is not None]
    if len(terms) < 2:
        return next(iter(terms), None)
    return _vector_op(em, ALGO_EXT.Addition, [(ALGO_EXT["in"], term) for term in terms], name)


def emit_expression_gradient(em: Emission, tree: QOpNode, stem: URIRef, owner: Any, world_qtys: dict) -> None:
    """The unit gradient a controlled expression is driven along, and the norm it was divided by.

    An expression's gradient is the same arithmetic over its terms' gradients -- a sum's is the sum
    of its terms', a product's the product rule's, a quotient's the dividend's over the divisor --
    so it is emitted as that arithmetic and recomputed every cycle: a derived scalar's own gradient
    moves with the robot. A body-fixed line also sweeps an angular term, combined the same way. The
    expression's top operator names the result, as a geometric operator names the gradient of the
    value it writes.
    """
    if stem in em.expression_plans:
        return
    gradient = _gradient_term(em, tree, "gradient", stem, "", owner, world_qtys)
    moment = _gradient_term(em, tree, "moment", stem, "", owner, world_qtys)
    frame = em.graph.value(gradient, GEOM_COORD["as-seen-by"])
    norm = URIRef(f"{stem}-gradient-norm")
    add_quantity(em, norm, QuantityType.Dimensionless)
    norm_op = URIRef(f"{norm}-op")
    em.graph.add((norm_op, RDF.type, GEOM_OP_EXT.VectorNorm))
    em.graph.add((norm_op, GEOM_OP["in"], gradient))
    em.graph.add((norm_op, GEOM_OP_EXT.norm, norm))
    top_op = URIRef(f"{stem}-{tree.op}")
    em.graph.add((top_op, GEOM_OP_EXT.norm, norm))
    units = {}
    for suffix, total in (("gradient", gradient), ("gradient-moment", moment)):
        if total is None:
            continue
        units[suffix] = _vector_op(
            em,
            ALGO_EXT.Division,
            [(ALGO_EXT.dividend, total), (ALGO_EXT.divisor, norm)],
            URIRef(f"{stem}-{suffix}"),
        )
        em.graph.add((top_op, GEOM_OP_EXT[suffix], units[suffix]))
    em.expression_plans[stem] = ExpressionPlan(
        units["gradient"], units.get("gradient-moment"), norm, frame
    )
