# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Dimension algebra over quantity-expression trees.

A pure, dependency-neutral home for the typing rules `+`/`-`/`*`/`/` obey: validation (does
this expression type-check) and RDF emission (what kind/unit does an interior op node carry)
call the same `infer`/`resolve_leaf` so the two never drift apart.
"""

from __future__ import annotations

from motion_spec_dsl.classes.context import (
    ContextQuantity,
    QOpNode,
    QuantityType,
    ReferenceGeneratorType,
    WorldQuantity,
    WorldQuantityType,
    _resolved_context_quantity,
)

Vector = tuple[int, int, int, int, int]  # (mass, length, time, angle, current)

# The table this plan's semantic contract states (§1): every scalar quantity kind an
# expression can produce, as its (mass, length, time, angle, current) exponents.
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
    QuantityType.ElectricCurrent: (0, 0, 0, 0, 1),
}

# Reverse lookup for a `*`/`/` result's derived vector. Several kinds share a vector; later
# entries win, so a product/quotient names the plain scalar over the geometry-flavored one.
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
    QuantityType.ElectricCurrent,
)
VECTOR_QUANTITY_TYPE: dict[Vector, QuantityType] = {
    DIMENSION_VECTOR[qty_type]: qty_type for qty_type in _VECTOR_PRIORITY
}

# A bare measure's authored unit, by dimension -- the DSL token set from `classes/units.py`.
# `Hz` is left out: no QuantityType names a frequency, so it was never a valid quantity value.
_UNIT_VECTOR: dict[str, Vector] = {
    "m": (0, 1, 0, 0, 0),
    "cm": (0, 1, 0, 0, 0),
    "mm": (0, 1, 0, 0, 0),
    "rad": (0, 0, 0, 1, 0),
    "deg": (0, 0, 0, 1, 0),
    "m/s": (0, 1, -1, 0, 0),
    "cm/s": (0, 1, -1, 0, 0),
    "rad/s": (0, 0, -1, 1, 0),
    "deg/s": (0, 0, -1, 1, 0),
    "m/s^2": (0, 1, -2, 0, 0),
    "rad/s^2": (0, 0, -2, 1, 0),
    "deg/s^2": (0, 0, -2, 1, 0),
    "m/s^3": (0, 1, -3, 0, 0),
    "N": (1, 1, -2, 0, 0),
    "Nm": (1, 2, -2, 0, 0),
    "s": (0, 0, 1, 0, 0),
    "ms": (0, 0, 1, 0, 0),
    "1": (0, 0, 0, 0, 0),
    "kg": (1, 0, 0, 0, 0),
    "A": (0, 0, 0, 0, 1),
}

# Position/Pose/Orientation are points and rotations, not scalars: `+`/`-` between two of the
# same kind is exactly the pre-expression offset behavior; `*`/`/` on any of them is an error.
GEOMETRY_TYPES = frozenset({QuantityType.Position, QuantityType.Pose, QuantityType.Orientation})

# Whole vector/tensor kinds an expression never produces or consumes -- they stay on their
# existing whole-quantity RDF operations (e.g. rbdyn-op:AddWrench).
_UNSUPPORTED_CONTEXT_TYPES = frozenset(
    {
        QuantityType.VelocityTwist,
        QuantityType.AccelerationTwist,
        QuantityType.Wrench,
        QuantityType.Direction,
        QuantityType.FreeVector,
    }
)

# A subspace of these composite kinds resolves to the same scalar QuantityType whether or not
# an axis is picked (an established quirk this plan does not change, e.g. `.force` and
# `.force.x` both type Force) -- so a scalar selector has to be checked structurally.
# A bare axis (`<q>.x`) names one component of a context quantity that is itself a 3-vector --
# no subspace, because the whole quantity is the subspace. A direction's and a free vector's
# components are unitless; every other vector's carry the scalar spelling of its own dimension.
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
    raise DimensionError(f"a pose has no '{subspace}' subspace.", leaf)


def _selected_subspace_type(subspace, axis, leaf, *, whole_label: str) -> QuantityType:
    if subspace is None:
        raise DimensionError(
            f"a whole {whole_label} is not supported in a quantity expression; "
            "select a scalar subspace axis.",
            leaf,
        )
    if axis is None:
        raise DimensionError("select a scalar subspace axis.", leaf)
    scalar = _SUBSPACE_TYPE.get(subspace)
    if scalar is None:
        raise DimensionError(f"a {whole_label} has no '{subspace}' subspace.", leaf)
    return scalar


def _world_leaf_type(world_type: WorldQuantityType, subspace, axis, leaf) -> QuantityType:
    if world_type == WorldQuantityType.JointPosition:
        return QuantityType.Angle
    if world_type == WorldQuantityType.JointVelocity:
        return QuantityType.AngularVelocity
    if world_type == WorldQuantityType.JointCurrent:
        return QuantityType.ElectricCurrent
    if world_type == WorldQuantityType.Pose:
        return _pose_subspace_type(subspace, axis, leaf)
    return _selected_subspace_type(subspace, axis, leaf, whole_label=str(world_type))


def _context_leaf_type(qty_type: QuantityType, subspace, axis, leaf) -> QuantityType:
    if subspace is None and axis is not None:
        component = VECTOR_COMPONENT_TYPE.get(qty_type)
        if component is None:
            raise DimensionError(f"a {qty_type} has no axis components.", leaf)
        return component
    if qty_type == QuantityType.Pose:
        return _pose_subspace_type(subspace, axis, leaf)
    if qty_type in _UNSUPPORTED_CONTEXT_TYPES:
        return _selected_subspace_type(subspace, axis, leaf, whole_label=str(qty_type))
    if subspace is not None:
        raise DimensionError(f"a {qty_type} has no '{subspace}' subspace.", leaf)
    return qty_type


def resolve_leaf(leaf) -> QuantityType:
    """The QuantityType of a quantity-expression leaf: a bare measure's unit, a world-quantity
    view's subspace/axis, or a context quantity's declared (or subspace-selected) type.

    Raises `DimensionError` for anything an expression cannot combine: a reference generator,
    an unrecognized unit or subspace, or a whole wrench/twist with no scalar axis picked.
    """
    bare = getattr(leaf, "bare", None)
    if bare is not None:
        vector = _UNIT_VECTOR.get(bare.unit)
        if vector is None:
            raise DimensionError(f"'{bare.unit}' has no known dimension.", leaf)
        return VECTOR_QUANTITY_TYPE.get(vector, QuantityType.Dimensionless)

    quantity = getattr(leaf, "quantity", None)
    subspace = getattr(leaf, "subspace", None)
    axis = getattr(leaf, "axis", None)
    if isinstance(quantity, WorldQuantity):
        return _world_leaf_type(quantity.type, subspace, axis, leaf)
    if isinstance(quantity, ContextQuantity):
        quantity = _resolved_context_quantity(quantity)
        if isinstance(quantity.type, ReferenceGeneratorType):
            raise DimensionError(
                f"'{quantity.name}' is a {quantity.type} reference generator, not a quantity "
                "an expression can combine.",
                leaf,
            )
        return _context_leaf_type(quantity.type, subspace, axis, leaf)
    raise DimensionError("expression leaf resolves to no quantity.", leaf)


def _combine(op: str, vectors: list[Vector]) -> Vector:
    if op == "multiply":
        result: Vector = (0, 0, 0, 0, 0)
        for vector in vectors:
            result = (
                result[0] + vector[0],
                result[1] + vector[1],
                result[2] + vector[2],
                result[3] + vector[3],
                result[4] + vector[4],
            )
        return result
    dividend, divisor = vectors
    return (
        dividend[0] - divisor[0],
        dividend[1] - divisor[1],
        dividend[2] - divisor[2],
        dividend[3] - divisor[3],
        dividend[4] - divisor[4],
    )


def _infer_add_subtract(node: QOpNode, operand_types: list[QuantityType]) -> QuantityType:
    first = operand_types[0]
    geometry = first in GEOMETRY_TYPES or any(t in GEOMETRY_TYPES for t in operand_types)
    if geometry:
        # A geometry kind is a point/rotation, not a scalar: no vector, no collapsing several
        # spellings into one -- every operand must be exactly the same kind.
        for other in operand_types[1:]:
            if other != first:
                raise DimensionError(
                    f"'{first}' and '{other}' cannot be added or subtracted -- "
                    "they are different kinds of quantity.",
                    node,
                )
        return first
    # Several scalar kinds share one physical dimension under different authored spellings
    # (a length and a projected distance are both metres) -- same vector is "same kind", and
    # the result takes the vector's canonical spelling, same as a `*`/`/` result would.
    first_vector = DIMENSION_VECTOR[first]
    for other in operand_types[1:]:
        if DIMENSION_VECTOR[other] != first_vector:
            raise DimensionError(
                f"'{first}' and '{other}' cannot be added or subtracted -- "
                "they are different kinds of quantity.",
                node,
            )
    return VECTOR_QUANTITY_TYPE[first_vector]


def _infer_op(node: QOpNode, resolve_leaf) -> QuantityType:
    operand_types = [_infer_node(operand, resolve_leaf) for operand in node.operands]
    if node.op in ("add", "subtract"):
        return _infer_add_subtract(node, operand_types)

    if node.op == "divide":
        divisor_bare = getattr(node.operands[1], "bare", None)
        if divisor_bare is not None and divisor_bare.value == 0.0:
            raise DimensionError("division by zero.", node)

    for qty_type in operand_types:
        if qty_type in GEOMETRY_TYPES:
            raise DimensionError(
                f"'{qty_type}' is a geometry kind; multiply/divide need a scalar subspace "
                "axis on each operand instead.",
                node,
            )
    vector = _combine(node.op, [DIMENSION_VECTOR[qty_type] for qty_type in operand_types])
    result = VECTOR_QUANTITY_TYPE.get(vector)
    if result is None:
        raise DimensionError(
            f"dimension {vector} from '{node.op}' of {operand_types} names no known quantity kind.",
            node,
        )
    return result


def _infer_node(node, resolve_leaf) -> QuantityType:
    if isinstance(node, QOpNode):
        return _infer_op(node, resolve_leaf)
    return resolve_leaf(node)


def infer(expr, resolve_leaf=resolve_leaf) -> QuantityType:
    """The result QuantityType of a quantity-expression tree (a `QExpr` or an already-normalized
    op tree/leaf), applying the typing rules in full.
    """
    tree = expr.as_op_tree() if hasattr(expr, "as_op_tree") else expr
    return _infer_node(tree, resolve_leaf)


def same_scalar_dimension(declared, inferred: QuantityType) -> bool:
    """Whether a declared/slot kind accepts an inferred one: identical, or two scalar
    spellings of one physical dimension (a length and a projected distance are both metres).
    A geometry kind accepts only itself; a non-QuantityType slot (legacy subspace string)
    is not enforced here.
    """
    if declared == inferred:
        return True
    if not isinstance(declared, QuantityType):
        return True
    if declared in GEOMETRY_TYPES or inferred in GEOMETRY_TYPES:
        return False
    declared_vector = DIMENSION_VECTOR.get(declared)
    return declared_vector is not None and declared_vector == DIMENSION_VECTOR.get(inferred)
