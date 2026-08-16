# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to coordinate grammar rules."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


_BINOPS = {
    "+": lambda a, b: a + b,
    "-": lambda a, b: a - b,
    "*": lambda a, b: a * b,
    "/": lambda a, b: a / b,
}


def const_value(node) -> float:
    """An authored constant expression as a number: literals and `pi` under `+ - * /`."""
    if isinstance(node, (int, float)):  # already folded
        return float(node)
    if hasattr(node, "head"):  # ConstExpr / ConstTerm: a head and (op, operand) pairs
        value = const_value(node.head)
        for step in node.tail:
            value = _BINOPS[step.op](value, const_value(step.operand))
        return value
    if hasattr(node, "atom"):  # ConstFactor: an atom under an optional unary minus
        value = const_value(node.atom)
        return -value if node.neg else value
    if getattr(node, "group", None) is not None:
        return const_value(node.group)
    if getattr(node, "pi", False):
        return math.pi
    return float(node.value)


@dataclass
class CoordinateElement:
    parent: object
    value: float | None = None
    ref: object | None = None

    def __post_init__(self) -> None:
        if self.value is not None:
            self.value = const_value(self.value)


@dataclass
class Coordinates:
    parent: object
    values: list[CoordinateElement] = field(default_factory=list)


@dataclass
class PositionCoordinate:
    parent: object
    ref: object | None = None
    coords: Coordinates | None = None
    unit: str = ""


@dataclass
class EulerAngles:
    parent: object
    axes: str = "xyz"
    extrinsic: bool = False
    angles: Coordinates | None = None
    unit: str = "rad"


@dataclass
class Quaternion:
    parent: object
    xyzw: Coordinates | None = None


@dataclass
class DirectionCosineXYZ:
    parent: object
    x_axis: Coordinates | None = None
    y_axis: Coordinates | None = None
    z_axis: Coordinates | None = None


@dataclass
class RelativeOrientation:
    """A base orientation turned by a delta, composed rather than decomposed."""

    parent: object
    base: object | None = None
    frame: object | None = None
    euler: EulerAngles | None = None
    quat: Quaternion | None = None
    direction_cosine: DirectionCosineXYZ | None = None


@dataclass
class OrientationCoordinate:
    parent: object
    ref: object | None = None
    relative: RelativeOrientation | None = None
    euler: EulerAngles | None = None
    quat: Quaternion | None = None
    direction_cosine: DirectionCosineXYZ | None = None


@dataclass
class VelocityTwistCoordinate:
    parent: object
    angular: Coordinates | None = None
    angular_unit: str = ""
    linear: Coordinates | None = None
    linear_unit: str = ""


@dataclass
class AccelerationTwistCoordinate:
    parent: object
    angular: Coordinates | None = None
    angular_unit: str = ""
    linear: Coordinates | None = None
    linear_unit: str = ""


@dataclass
class WrenchCoordinate:
    parent: object
    torque: Coordinates | None = None
    torque_unit: str = ""
    force: Coordinates | None = None
    force_unit: str = ""


@dataclass
class PoseCoordinate:
    parent: object
    position: PositionCoordinate
    orientation: OrientationCoordinate
