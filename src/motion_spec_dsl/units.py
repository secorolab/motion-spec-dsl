# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""DSL unit tokens and their canonical SI conversion; a leaf module with no
dependency on `classes` or `motion_spec`, so both `classes` and `rdf` can import it."""

from __future__ import annotations

import math
from typing import Any

from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT

DSL_UNIT: dict[str, tuple[Any, float]] = {
    "m":       (QUDT_UNIT.M, 1.0),
    "cm":      (QUDT_UNIT.M, 0.01),
    "mm":      (QUDT_UNIT.M, 0.001),
    "rad":     (QUDT_UNIT["RAD"], 1.0),
    "deg":     (QUDT_UNIT["RAD"], math.pi / 180.0),
    "m/s":     (QUDT_UNIT["M-PER-SEC"], 1.0),
    "cm/s":    (QUDT_UNIT["M-PER-SEC"], 0.01),
    "rad/s":   (QUDT_UNIT["RAD-PER-SEC"], 1.0),
    "deg/s":   (QUDT_UNIT["RAD-PER-SEC"], math.pi / 180.0),
    "m/s^2":   (QUDT_UNIT["M-PER-SEC2"], 1.0),
    "rad/s^2": (QUDT_UNIT["RAD-PER-SEC2"], 1.0),
    "deg/s^2": (QUDT_UNIT["RAD-PER-SEC2"], math.pi / 180.0),
    "m/s^3":   (QUDT_UNIT["M-PER-SEC3"], 1.0),
    "N":       (QUDT_UNIT.N, 1.0),
    "Nm":      (QUDT_UNIT["N-M"], 1.0),
    "s":       (QUDT_UNIT["SEC"], 1.0),
    "ms":      (QUDT_UNIT["SEC"], 0.001),
    "1":       (QUDT_UNIT.UNITLESS, 1.0),
}


def _dsl_unit(unit_name: str) -> Any:
    """Map a DSL unit token to its canonical SI QUDT unit URI; raises on an unsupported token."""
    try:
        return DSL_UNIT[unit_name][0]
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc


def _si_value(value: float, unit_name: str) -> float:
    """The authored value in its canonical SI unit."""
    try:
        _, factor = DSL_UNIT[unit_name]
    except KeyError as exc:
        raise ValueError(f"Unsupported DSL unit '{unit_name}'.") from exc
    return float(value) * factor
