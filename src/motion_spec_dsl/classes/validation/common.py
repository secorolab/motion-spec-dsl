# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Shared helpers for validating parsed motion-spec models."""

from __future__ import annotations

from collections.abc import Iterable

from textx import get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.constraint_handler import ConstraintHandler
from motion_spec_dsl.classes.constraints import (
    ConstraintAlias,
    ConstraintSpecification,
    _flatten_constraint_items,
    _resolved_spec,
)
from motion_spec_dsl.classes.motion_spec import (
    GuardedMotion,
    Model,
)


def semantic_error(message: str, obj: object | None = None) -> TextXSemanticError:
    """Build a TextXSemanticError, attaching `obj`'s source location when available."""
    if obj is None:
        return TextXSemanticError(message)
    try:
        return TextXSemanticError(message, **get_location(obj))
    except Exception:
        return TextXSemanticError(message)


def motion_specs(model: Model) -> Iterable[GuardedMotion]:
    """The MotionSpecs declared in `model`."""
    return (spec for spec in model.specs if isinstance(spec, GuardedMotion))


def constraint_handlers(model: Model) -> Iterable[ConstraintHandler]:
    """The ConstraintHandlers declared in `model`."""
    return (spec for spec in model.specs if isinstance(spec, ConstraintHandler))


def motion_constraint_items(spec: GuardedMotion) -> list[ConstraintSpecification | ConstraintAlias]:
    """All section items (inline specs and aliases) by their local name."""
    return _flatten_constraint_items(
        [item for section in spec.sections for item in section.constraints]
    )


def motion_constraints(spec: GuardedMotion) -> list[ConstraintSpecification]:
    """All resolved ConstraintSpecification objects for this motion."""
    return [_resolved_spec(item) for item in motion_constraint_items(spec)]
