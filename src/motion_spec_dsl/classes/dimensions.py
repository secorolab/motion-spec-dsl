# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Dimension algebra over quantity-expression trees, shared by validation and RDF emission."""

from __future__ import annotations

from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    QOpNode,
    QuantityLeaf,
    QuantityType,
    ReferenceGeneratorType,
    WorldQuantity,
    WorldQuantityType,
    op_tree,
)
from motion_spec_dsl.classes.units import DSL_UNITS

Vector = tuple[int, int, int, int, int]  # (mass, length, time, angle, current)

# Every scalar kind an expression can produce, as its dimension exponents.
DIMENSION_VECTOR: dict[QuantityType, Vector] = {
    QuantityType.Dimensionless: (0, 0, 0, 0, 0),
    QuantityType.PathParameter: (0, 0, 0, 0, 0),
    QuantityType.Duration: (0, 0, 1, 0, 0),
    QuantityType.Length: (0, 1, 0, 0, 0),
    QuantityType.Distance: (0, 1, 0, 0, 0),
    QuantityType.Position: (0, 1, 0, 0, 0),
    QuantityType.Angle: (0, 0, 0, 1, 0),
    QuantityType.PlaneAngle: (0, 0, 0, 1, 0),
    QuantityType.Orientation: (0, 0, 0, 1, 0),
    QuantityType.LinearVelocity: (0, 1, -1, 0, 0),
    QuantityType.AngularVelocity: (0, 0, -1, 1, 0),
    QuantityType.LinearAcceleration: (0, 1, -2, 0, 0),
    QuantityType.AngularAcceleration: (0, 0, -2, 1, 0),
    QuantityType.LinearJerk: (0, 1, -3, 0, 0),
    QuantityType.Force: (1, 1, -2, 0, 0),
    QuantityType.Torque: (1, 2, -2, 0, 0),
    QuantityType.Mass: (1, 0, 0, 0, 0),
}

# Several kinds share a vector; later entries win, so a product names the plain scalar.
_VECTOR_PRIORITY = (
    QuantityType.Position,
    QuantityType.Distance,
    QuantityType.Length,
    QuantityType.Orientation,
    QuantityType.PlaneAngle,
    QuantityType.Angle,
    QuantityType.Dimensionless,
    QuantityType.PathParameter,
    QuantityType.Duration,
    QuantityType.LinearVelocity,
    QuantityType.AngularVelocity,
    QuantityType.LinearAcceleration,
    QuantityType.AngularAcceleration,
    QuantityType.LinearJerk,
    QuantityType.Force,
    QuantityType.Torque,
    QuantityType.Mass,
)
VECTOR_QUANTITY_TYPE: dict[Vector, QuantityType] = {
    DIMENSION_VECTOR[qty_type]: qty_type for qty_type in _VECTOR_PRIORITY
}

# Points and rotations, not scalars: `+`/`-` only between two of the same kind, never `*`/`/`.
GEOMETRY_TYPES = {QuantityType.Position, QuantityType.Pose, QuantityType.Orientation}

# Whole vector kinds that stay on their whole-quantity RDF operations, never in an expression.
_UNSUPPORTED_CONTEXT_TYPES = {
    QuantityType.VelocityTwist,
    QuantityType.AccelerationTwist,
    QuantityType.Wrench,
    QuantityType.Direction,
    QuantityType.FreeVector,
}

# The kind of one component a bare axis (`<q>.x`) names on a 3-vector context quantity.
VECTOR_COMPONENT_TYPE: dict[QuantityType, QuantityType] = {
    QuantityType.Direction: QuantityType.Dimensionless,
    QuantityType.FreeVector: QuantityType.Dimensionless,
    QuantityType.Position: QuantityType.Distance,
    QuantityType.LinearVelocity: QuantityType.LinearVelocity,
    QuantityType.Force: QuantityType.Force,
    QuantityType.Torque: QuantityType.Torque,
}

_SUBSPACE_TYPE: dict[str, QuantityType] = {
    "linvel": QuantityType.LinearVelocity,
    "angvel": QuantityType.AngularVelocity,
    "linacc": QuantityType.LinearAcceleration,
    "angacc": QuantityType.AngularAcceleration,
    "force": QuantityType.Force,
    "torque": QuantityType.Torque,
}

_JOINT_TYPES = {
    WorldQuantityType.JointPosition: QuantityType.Angle,
    WorldQuantityType.JointVelocity: QuantityType.AngularVelocity,
    WorldQuantityType.JointForce: QuantityType.Torque,
}


class DimensionError(ValueError):
    """Dimension inference failed for a quantity-expression subexpression."""

    def __init__(self, message: str, expr: object) -> None:
        super().__init__(message)
        self.expr = expr


def _pose_subspace_type(subspace, axis, leaf) -> QuantityType:
    if subspace is None:
        return QuantityType.Pose
    if subspace == "position":
        return QuantityType.Distance if axis is not None else QuantityType.Position
    if subspace == "orientation":
        return QuantityType.Angle if axis is not None else QuantityType.Orientation
    raise DimensionError(f"a pose has no '{subspace}' subspace", leaf)


def _selected_subspace_type(subspace, axis, leaf, whole_label: str) -> QuantityType:
    if subspace is None:
        raise DimensionError(
            f"a whole {whole_label} is not supported in a quantity expression -- "
            "select a scalar subspace axis",
            leaf,
        )
    if axis is None:
        raise DimensionError("select a scalar subspace axis", leaf)
    if subspace not in _SUBSPACE_TYPE:
        raise DimensionError(f"a {whole_label} has no '{subspace}' subspace", leaf)
    return _SUBSPACE_TYPE[subspace]


def resolve_leaf(leaf) -> QuantityType:
    """The kind of an expression leaf: a measure's unit, or a quantity's view or declared type."""
    bare = leaf.bare if isinstance(leaf, (QuantityLeaf, ContextRef)) else None
    if bare is not None:
        unit = DSL_UNITS.get(bare.unit)
        if unit is None or unit.vector is None:
            raise DimensionError(f"'{bare.unit}' has no known dimension", leaf)
        return VECTOR_QUANTITY_TYPE.get(unit.vector, QuantityType.Dimensionless)
    quantity, subspace, axis = leaf.quantity, leaf.subspace, leaf.axis
    if isinstance(quantity, WorldQuantity):
        if quantity.type in _JOINT_TYPES:
            return _JOINT_TYPES[quantity.type]
        if quantity.type == WorldQuantityType.Pose:
            return _pose_subspace_type(subspace, axis, leaf)
        return _selected_subspace_type(subspace, axis, leaf, str(quantity.type))
    if not isinstance(quantity, ContextQuantity):
        raise DimensionError("expression leaf resolves to no quantity", leaf)
    if isinstance(quantity, ContextQuantityAlias):
        quantity = quantity.ref
    qty_type = quantity.type
    if isinstance(qty_type, ReferenceGeneratorType):
        raise DimensionError(
            f"'{quantity.name}' is a {qty_type} reference generator -- "
            "an expression combines quantities",
            leaf,
        )
    if subspace is None and axis is not None:
        if qty_type not in VECTOR_COMPONENT_TYPE:
            raise DimensionError(f"a {qty_type} has no axis components", leaf)
        return VECTOR_COMPONENT_TYPE[qty_type]
    if qty_type == QuantityType.Pose:
        return _pose_subspace_type(subspace, axis, leaf)
    if qty_type in _UNSUPPORTED_CONTEXT_TYPES:
        return _selected_subspace_type(subspace, axis, leaf, str(qty_type))
    if subspace is not None:
        raise DimensionError(f"a {qty_type} has no '{subspace}' subspace", leaf)
    return qty_type


def infer(expr, resolve=resolve_leaf) -> QuantityType:
    """The kind an expression, op tree or leaf evaluates to under the typing rules."""
    node = op_tree(expr)
    if not isinstance(node, QOpNode):
        return resolve(node)
    operand_types = [infer(operand, resolve) for operand in node.operands]
    first = operand_types[0]
    if node.op in ("add", "subtract"):
        # A geometry kind collapses no spellings: every operand must be exactly the same kind.
        geometry = any(qty_type in GEOMETRY_TYPES for qty_type in operand_types)
        first_vector = DIMENSION_VECTOR.get(first)
        for other in operand_types[1:]:
            same = other == first if geometry else DIMENSION_VECTOR[other] == first_vector
            if not same:
                raise DimensionError(
                    f"'{first}' and '{other}' cannot be added or subtracted -- "
                    "they are different kinds of quantity",
                    node,
                )
        return first if geometry else VECTOR_QUANTITY_TYPE[first_vector]
    divisor = node.operands[-1]
    if (
        node.op == "divide"
        and isinstance(divisor, QuantityLeaf)
        and divisor.bare is not None
        and divisor.bare.value == 0.0
    ):
        raise DimensionError("division by zero", node)
    for qty_type in operand_types:
        if qty_type in GEOMETRY_TYPES:
            raise DimensionError(
                f"'{qty_type}' is a geometry kind -- multiply and divide need a scalar "
                "subspace axis on each operand",
                node,
            )
    vectors = [DIMENSION_VECTOR[qty_type] for qty_type in operand_types]
    if node.op == "multiply":
        vector = tuple(sum(exponents) for exponents in zip(*vectors))
    else:
        vector = tuple(a - b for a, b in zip(vectors[0], vectors[1]))
    if vector not in VECTOR_QUANTITY_TYPE:
        raise DimensionError(
            f"dimension {vector} from '{node.op}' of {operand_types} names no known quantity kind",
            node,
        )
    return VECTOR_QUANTITY_TYPE[vector]


def same_scalar_dimension(declared, inferred: QuantityType) -> bool:
    """Whether a declared kind accepts an inferred one: identical, or one scalar dimension."""
    if declared == inferred or not isinstance(declared, QuantityType):
        return True
    if declared in GEOMETRY_TYPES or inferred in GEOMETRY_TYPES:
        return False
    declared_vector = DIMENSION_VECTOR.get(declared)
    return declared_vector is not None and declared_vector == DIMENSION_VECTOR.get(inferred)
