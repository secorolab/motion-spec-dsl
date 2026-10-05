# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""DSL unit tokens and the QUDT units they name; nothing here rescales a value."""

from __future__ import annotations

from typing import Any, NamedTuple

from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_KIND
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

from motion_spec_dsl.classes.context import QuantityType
from motion_spec_dsl.rdf_parser.vocab import GEOM_REL, QKIND_EXT, RBDYN_ENT


class DslUnit(NamedTuple):
    """What one unit token names: its QUDT unit, the kinds it serves, and its dimension."""

    iri: Any
    # QUDT hasQuantityKind (v3.5.2); Distance and PositionVector by their skos:broader Length.
    kinds: set
    # (mass, length, time, angle, current) exponents; None where no kind names the dimension.
    vector: tuple[int, int, int, int, int] | None


DSL_UNITS: dict[str, DslUnit] = {
    "m": DslUnit(
        QUDT_UNIT.M,
        {QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector},
        (0, 1, 0, 0, 0),
    ),
    "cm": DslUnit(
        QUDT_UNIT["CentiM"],
        {QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector},
        (0, 1, 0, 0, 0),
    ),
    "mm": DslUnit(
        QUDT_UNIT["MilliM"],
        {QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector},
        (0, 1, 0, 0, 0),
    ),
    "rad": DslUnit(QUDT_UNIT["RAD"], {QUDT_KIND.Angle, QUDT_KIND.PlaneAngle}, (0, 0, 0, 1, 0)),
    "deg": DslUnit(QUDT_UNIT["DEG"], {QUDT_KIND.Angle, QUDT_KIND.PlaneAngle}, (0, 0, 0, 1, 0)),
    "m/s": DslUnit(QUDT_UNIT["M-PER-SEC"], {QUDT_KIND.LinearVelocity}, (0, 1, -1, 0, 0)),
    "cm/s": DslUnit(QUDT_UNIT["CentiM-PER-SEC"], {QUDT_KIND.LinearVelocity}, (0, 1, -1, 0, 0)),
    "rad/s": DslUnit(QUDT_UNIT["RAD-PER-SEC"], {QUDT_KIND.AngularVelocity}, (0, 0, -1, 1, 0)),
    "deg/s": DslUnit(QUDT_UNIT["DEG-PER-SEC"], {QUDT_KIND.AngularVelocity}, (0, 0, -1, 1, 0)),
    "m/s^2": DslUnit(QUDT_UNIT["M-PER-SEC2"], {QUDT_KIND.LinearAcceleration}, (0, 1, -2, 0, 0)),
    "rad/s^2": DslUnit(
        QUDT_UNIT["RAD-PER-SEC2"], {QUDT_KIND.AngularAcceleration}, (0, 0, -2, 1, 0)
    ),
    "deg/s^2": DslUnit(
        QUDT_UNIT["DEG-PER-SEC2"], {QUDT_KIND.AngularAcceleration}, (0, 0, -2, 1, 0)
    ),
    "m/s^3": DslUnit(QUDT_UNIT["M-PER-SEC3"], {QKIND_EXT.LinearJerk}, (0, 1, -3, 0, 0)),
    "N": DslUnit(QUDT_UNIT.N, {QUDT_KIND.Force}, (1, 1, -2, 0, 0)),
    "Nm": DslUnit(QUDT_UNIT["N-M"], {QUDT_KIND.Torque}, (1, 2, -2, 0, 0)),
    "s": DslUnit(QUDT_UNIT["SEC"], {QUDT_KIND.Time}, (0, 0, 1, 0, 0)),
    "ms": DslUnit(QUDT_UNIT["MilliSEC"], {QUDT_KIND.Time}, (0, 0, 1, 0, 0)),
    "Hz": DslUnit(QUDT_UNIT["HZ"], {QUDT_KIND.Frequency}, None),
    "1": DslUnit(
        QUDT_UNIT.UNITLESS, {QUDT_KIND.Dimensionless, QUDT_KIND.FreeVector}, (0, 0, 0, 0, 0)
    ),
    "kg": DslUnit(QUDT_UNIT["KiloGM"], {QUDT_KIND.Mass}, (1, 0, 0, 0, 0)),
}

ANGLE_UNITS: tuple[Any, ...] = (DSL_UNITS["rad"].iri, DSL_UNITS["deg"].iri)

QUDT_KIND_BY_QUANTITY_TYPE: dict[QuantityType, Any] = {
    QuantityType.Pose: GEOM_REL.Pose,
    QuantityType.Position: QUDT_KIND.PositionVector,
    QuantityType.Orientation: QUDT_KIND.PlaneAngle,
    QuantityType.VelocityTwist: GEOM_REL.VelocityTwist,
    QuantityType.AccelerationTwist: GEOM_REL.AccelerationTwist,
    QuantityType.Wrench: RBDYN_ENT.Wrench,
    QuantityType.Direction: QUDT_KIND.Dimensionless,
    QuantityType.Length: QUDT_KIND.Length,
    QuantityType.Distance: QUDT_KIND.Distance,
    QuantityType.Angle: QUDT_KIND.PlaneAngle,
    QuantityType.PlaneAngle: QUDT_KIND.PlaneAngle,
    QuantityType.LinearVelocity: QUDT_KIND.LinearVelocity,
    QuantityType.AngularVelocity: QUDT_KIND.AngularVelocity,
    QuantityType.LinearAcceleration: QUDT_KIND.LinearAcceleration,
    QuantityType.AngularAcceleration: QUDT_KIND.AngularAcceleration,
    QuantityType.LinearJerk: QKIND_EXT.LinearJerk,
    QuantityType.Force: QUDT_KIND.Force,
    QuantityType.Torque: QUDT_KIND.Torque,
    QuantityType.Mass: QUDT_KIND.Mass,
    QuantityType.FreeVector: QUDT_KIND.FreeVector,
    QuantityType.Dimensionless: QUDT_KIND.Dimensionless,
    QuantityType.Duration: QUDT_KIND.Time,
    QuantityType.PathParameter: QUDT_KIND.Dimensionless,
}
