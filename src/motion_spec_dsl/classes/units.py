# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""DSL unit tokens and the QUDT units they name.

A token maps to the unit it says, so the graph records what the model was written in and
a reader converts when it needs a number. Nothing here rescales a value. `rdf-utils` does
that conversion for everything geometric -- orientations come back in radians and positions
in metres however they were authored -- leaving a reader the handful of scalars that are
not coordinates.

A duration is the one place the natural vocabulary cannot hold the authored unit:
`time:unitType` bottoms out at `time:unitSecond`, so a `time:Duration` carries its
magnitude as a qudt Time-kind scalar instead. See `_emit_duration_measure`.
"""

from __future__ import annotations

from typing import Any, NamedTuple

from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_KIND
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

from motion_spec_dsl.classes.context import QuantityType
from motion_spec_dsl.rdf_parser.vocab import GEOM_REL, QKIND_EXT, RBDYN_ENT


class DslUnit(NamedTuple):
    """What one unit token names: its QUDT unit, the kinds it serves, and its dimension."""

    iri: Any
    # QUDT's hasQuantityKind (v3.5.2), Distance and PositionVector by their skos:broader Length,
    # plus the DSL's own kinds QUDT has no unit for.
    kinds: frozenset
    # (mass, length, time, angle, current) exponents; None where no QuantityType names the
    # dimension, so the token is never a quantity value.
    vector: tuple[int, int, int, int, int] | None


DSL_UNITS: dict[str, DslUnit] = {
    "m": DslUnit(
        QUDT_UNIT.M,
        frozenset({QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector}),
        (0, 1, 0, 0, 0),
    ),
    "cm": DslUnit(
        QUDT_UNIT["CentiM"],
        frozenset({QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector}),
        (0, 1, 0, 0, 0),
    ),
    "mm": DslUnit(
        QUDT_UNIT["MilliM"],
        frozenset({QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector}),
        (0, 1, 0, 0, 0),
    ),
    "rad": DslUnit(
        QUDT_UNIT["RAD"], frozenset({QUDT_KIND.Angle, QUDT_KIND.PlaneAngle}), (0, 0, 0, 1, 0)
    ),
    "deg": DslUnit(
        QUDT_UNIT["DEG"], frozenset({QUDT_KIND.Angle, QUDT_KIND.PlaneAngle}), (0, 0, 0, 1, 0)
    ),
    "m/s": DslUnit(QUDT_UNIT["M-PER-SEC"], frozenset({QUDT_KIND.LinearVelocity}), (0, 1, -1, 0, 0)),
    "cm/s": DslUnit(
        QUDT_UNIT["CentiM-PER-SEC"], frozenset({QUDT_KIND.LinearVelocity}), (0, 1, -1, 0, 0)
    ),
    "rad/s": DslUnit(
        QUDT_UNIT["RAD-PER-SEC"], frozenset({QUDT_KIND.AngularVelocity}), (0, 0, -1, 1, 0)
    ),
    "deg/s": DslUnit(
        QUDT_UNIT["DEG-PER-SEC"], frozenset({QUDT_KIND.AngularVelocity}), (0, 0, -1, 1, 0)
    ),
    "m/s^2": DslUnit(
        QUDT_UNIT["M-PER-SEC2"], frozenset({QUDT_KIND.LinearAcceleration}), (0, 1, -2, 0, 0)
    ),
    "rad/s^2": DslUnit(
        QUDT_UNIT["RAD-PER-SEC2"], frozenset({QUDT_KIND.AngularAcceleration}), (0, 0, -2, 1, 0)
    ),
    "deg/s^2": DslUnit(
        QUDT_UNIT["DEG-PER-SEC2"], frozenset({QUDT_KIND.AngularAcceleration}), (0, 0, -2, 1, 0)
    ),
    "m/s^3": DslUnit(QUDT_UNIT["M-PER-SEC3"], frozenset({QKIND_EXT.LinearJerk}), (0, 1, -3, 0, 0)),
    "N": DslUnit(QUDT_UNIT.N, frozenset({QUDT_KIND.Force}), (1, 1, -2, 0, 0)),
    "Nm": DslUnit(QUDT_UNIT["N-M"], frozenset({QUDT_KIND.Torque}), (1, 2, -2, 0, 0)),
    "s": DslUnit(QUDT_UNIT["SEC"], frozenset({QUDT_KIND.Time}), (0, 0, 1, 0, 0)),
    "ms": DslUnit(QUDT_UNIT["MilliSEC"], frozenset({QUDT_KIND.Time}), (0, 0, 1, 0, 0)),
    "Hz": DslUnit(QUDT_UNIT["HZ"], frozenset({QUDT_KIND.Frequency}), None),
    "1": DslUnit(
        QUDT_UNIT.UNITLESS,
        frozenset({QUDT_KIND.Dimensionless, QUDT_KIND.FreeVector}),
        (0, 0, 0, 0, 0),
    ),
    "kg": DslUnit(QUDT_UNIT["KiloGM"], frozenset({QUDT_KIND.Mass}), (1, 0, 0, 0, 0)),
    "A": DslUnit(QUDT_UNIT.A, frozenset({QUDT_KIND.ElectricCurrent}), (0, 0, 0, 0, 1)),
}

ANGLE_UNITS: tuple[Any, ...] = (DSL_UNITS["rad"].iri, DSL_UNITS["deg"].iri)

QUDT_KIND_BY_QUANTITY_TYPE: dict[Any, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Length: QUDT_KIND.Length,
    QuantityType.Position: QUDT_KIND.PositionVector,
    QuantityType.Orientation: QUDT_KIND.PlaneAngle,
    QuantityType.Angle: QUDT_KIND.PlaneAngle,
    QuantityType.PlaneAngle: QUDT_KIND.PlaneAngle,
    QuantityType.VelocityTwist: GEOM_REL.VelocityTwist,
    QuantityType.AccelerationTwist: GEOM_REL.AccelerationTwist,
    QuantityType.Wrench: RBDYN_ENT.Wrench,
    QuantityType.Direction: QUDT_KIND["Dimensionless"],
    QuantityType.FreeVector: QUDT_KIND["FreeVector"],
    QuantityType.Dimensionless: QUDT_KIND["Dimensionless"],
    QuantityType.Duration: QUDT_KIND["Time"],
    QuantityType.PathParameter: QUDT_KIND["Dimensionless"],
    QuantityType.LinearJerk: QKIND_EXT.LinearJerk,
    QuantityType.Mass: QUDT_KIND.Mass,
    QuantityType.ElectricCurrent: QUDT_KIND.ElectricCurrent,
}


def _qudt_kind(quantity_type: Any) -> Any:
    """QUDT quantity kind for a DSL quantity type."""
    return QUDT_KIND_BY_QUANTITY_TYPE.get(quantity_type) or QUDT_KIND[quantity_type]


def _angle_unit(euler: Any) -> str:
    """The unit an Euler triple was written in; `rad` when the author left it off."""
    return getattr(euler, "unit", None) or "rad"


def _dsl_unit(unit_name: str) -> Any:
    """Map a DSL unit token to its canonical SI QUDT unit URI; raises on an unsupported token."""
    try:
        return DSL_UNITS[unit_name].iri
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc
