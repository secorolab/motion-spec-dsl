# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""DSL unit tokens and the QUDT units they name.

A token maps to the unit it says, so the graph records what the model was written in and
a reader converts when it needs a number. `rdf-utils` does that for everything geometric
-- orientations come back in radians and positions in metres however they were authored
-- leaving a reader the handful of scalars that are not coordinates.

A `time:Duration` is the exception: its vocabulary has no unit below the second, so a
duration is written in seconds. See `_emit_duration_measure`.
"""

from __future__ import annotations

from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

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
    "1": QUDT_UNIT.UNITLESS,
}

SECONDS_IN: dict[str, float] = {"s": 1.0, "ms": 1e-3}


def _dsl_unit(unit_name: str) -> Any:
    """Map a DSL unit token to its canonical SI QUDT unit URI; raises on an unsupported token."""
    try:
        return DSL_UNIT[unit_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc


def _si_seconds(value: float, unit_name: str) -> float:
    """A duration in seconds, for `time:Duration`, whose units stop at the second."""
    try:
        return float(value) * SECONDS_IN[unit_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported duration unit '{unit_name}'.") from exc
