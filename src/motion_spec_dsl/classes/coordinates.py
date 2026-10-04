# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to coordinate grammar rules."""

from __future__ import annotations

import math
import operator

CONST_OPS = {"+": operator.add, "-": operator.sub, "*": operator.mul, "/": operator.truediv}


def const_value(node) -> float:
    """An authored constant expression as a number: literals and `pi` under `+ - * /`."""
    if isinstance(node, (int, float)):
        return float(node)
    # ConstExpr and ConstTerm: a head and (op, operand) steps.
    if hasattr(node, "head"):
        value = const_value(node.head)
        for step in node.tail:
            value = CONST_OPS[step.op](value, const_value(step.operand))
        return value
    # ConstFactor: an atom under an optional unary minus.
    if hasattr(node, "atom"):
        value = const_value(node.atom)
        return -value if node.neg else value
    if node.group is not None:
        return const_value(node.group)
    if node.pi:
        return math.pi
    return float(node.value)


class CoordinateElement:
    def __init__(self, parent, value, ref) -> None:
        self.parent = parent
        self.value = const_value(value) if value is not None else None
        self.ref = ref


class Coordinates:
    def __init__(self, parent, values) -> None:
        self.parent = parent
        self.values = values


class PositionCoordinate:
    def __init__(self, parent, ref, coords, unit) -> None:
        self.parent = parent
        self.ref = ref
        self.coords = coords
        self.unit = unit


class EulerAngles:
    def __init__(self, parent, axes, extrinsic, angles, unit) -> None:
        self.parent = parent
        self.axes = axes
        self.extrinsic = extrinsic
        self.angles = angles
        self.unit = unit or "rad"


class Quaternion:
    def __init__(self, parent, xyzw) -> None:
        self.parent = parent
        self.xyzw = xyzw


class DirectionCosineXYZ:
    def __init__(self, parent, x_axis, y_axis, z_axis) -> None:
        self.parent = parent
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis


class RelativeOrientation:
    """A base orientation turned by a delta, composed rather than decomposed."""

    def __init__(self, parent, base, frame, euler, quat, direction_cosine) -> None:
        self.parent = parent
        self.base = base
        self.frame = frame
        self.euler = euler
        self.quat = quat
        self.direction_cosine = direction_cosine


class OrientationCoordinate:
    def __init__(self, parent, relative, ref, euler, quat, direction_cosine) -> None:
        self.parent = parent
        self.relative = relative
        self.ref = ref
        self.euler = euler
        self.quat = quat
        self.direction_cosine = direction_cosine


class VelocityTwistCoordinate:
    def __init__(self, parent, angular, angular_unit, linear, linear_unit) -> None:
        self.parent = parent
        self.angular = angular
        self.angular_unit = angular_unit
        self.linear = linear
        self.linear_unit = linear_unit


class AccelerationTwistCoordinate:
    def __init__(self, parent, angular, angular_unit, linear, linear_unit) -> None:
        self.parent = parent
        self.angular = angular
        self.angular_unit = angular_unit
        self.linear = linear
        self.linear_unit = linear_unit


class WrenchCoordinate:
    def __init__(self, parent, torque, torque_unit, force, force_unit) -> None:
        self.parent = parent
        self.torque = torque
        self.torque_unit = torque_unit
        self.force = force
        self.force_unit = force_unit


class PoseCoordinate:
    def __init__(self, parent, position, orientation) -> None:
        self.parent = parent
        self.position = position
        self.orientation = orientation
