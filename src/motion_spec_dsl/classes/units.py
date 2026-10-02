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

from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_KIND
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

from motion_spec_dsl.rdf_parser.vocab import QKIND_EXT

DSL_UNIT: dict[str, Any] = {
    "m": QUDT_UNIT.M,
    "cm": QUDT_UNIT["CentiM"],
    "mm": QUDT_UNIT["MilliM"],
    "rad": QUDT_UNIT["RAD"],
    "deg": QUDT_UNIT["DEG"],
    "m/s": QUDT_UNIT["M-PER-SEC"],
    "cm/s": QUDT_UNIT["CentiM-PER-SEC"],
    "rad/s": QUDT_UNIT["RAD-PER-SEC"],
    "deg/s": QUDT_UNIT["DEG-PER-SEC"],
    "m/s^2": QUDT_UNIT["M-PER-SEC2"],
    "rad/s^2": QUDT_UNIT["RAD-PER-SEC2"],
    "deg/s^2": QUDT_UNIT["DEG-PER-SEC2"],
    "m/s^3": QUDT_UNIT["M-PER-SEC3"],
    "N": QUDT_UNIT.N,
    "Nm": QUDT_UNIT["N-M"],
    "s": QUDT_UNIT["SEC"],
    "ms": QUDT_UNIT["MilliSEC"],
    "Hz": QUDT_UNIT["HZ"],
    "1": QUDT_UNIT.UNITLESS,
    "kg": QUDT_UNIT["KiloGM"],
    "A": QUDT_UNIT.A,
}

_LENGTH_KINDS = frozenset({QUDT_KIND.Length, QUDT_KIND.Distance, QUDT_KIND.PositionVector})
_ANGLE_KINDS = frozenset({QUDT_KIND.Angle, QUDT_KIND.PlaneAngle})

# The quantity kinds each token's unit serves: QUDT's hasQuantityKind (v3.5.2), Distance and
# PositionVector by their skos:broader Length, plus the DSL's own kinds QUDT has no unit for.
UNIT_KINDS: dict[str, frozenset] = {
    "m": _LENGTH_KINDS,
    "cm": _LENGTH_KINDS,
    "mm": _LENGTH_KINDS,
    "rad": _ANGLE_KINDS,
    "deg": _ANGLE_KINDS,
    "m/s": frozenset({QUDT_KIND.LinearVelocity}),
    "cm/s": frozenset({QUDT_KIND.LinearVelocity}),
    "rad/s": frozenset({QUDT_KIND.AngularVelocity}),
    "deg/s": frozenset({QUDT_KIND.AngularVelocity}),
    "m/s^2": frozenset({QUDT_KIND.LinearAcceleration}),
    "rad/s^2": frozenset({QUDT_KIND.AngularAcceleration}),
    "deg/s^2": frozenset({QUDT_KIND.AngularAcceleration}),
    "m/s^3": frozenset({QKIND_EXT.LinearJerk}),
    "N": frozenset({QUDT_KIND.Force}),
    "Nm": frozenset({QUDT_KIND.Torque}),
    "s": frozenset({QUDT_KIND.Time}),
    "ms": frozenset({QUDT_KIND.Time}),
    "Hz": frozenset({QUDT_KIND.Frequency}),
    "1": frozenset({QUDT_KIND.Dimensionless, QUDT_KIND.FreeVector}),
    "kg": frozenset({QUDT_KIND.Mass}),
    "A": frozenset({QUDT_KIND.ElectricCurrent}),
}

ANGLE_UNITS: tuple[Any, ...] = (DSL_UNIT["rad"], DSL_UNIT["deg"])


def _angle_unit(euler: Any) -> str:
    """The unit an Euler triple was written in; `rad` when the author left it off."""
    return getattr(euler, "unit", None) or "rad"


def _dsl_unit(unit_name: str) -> Any:
    """Map a DSL unit token to its canonical SI QUDT unit URI; raises on an unsupported token."""
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc
