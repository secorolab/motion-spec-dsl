# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Shared helpers for semantic validation phases."""

from __future__ import annotations

from collections.abc import Iterable

from textx import get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.domain import (
    ConstraintAlias,
    ConstraintHandler,
    ConstraintSpecification,
    ControllerAlias,
    ControllerEntry,
    Model,
    MotionSpec,
    SolverAlias,
    SolverEntry,
    _resolved_spec,
)


def semantic_error(message: str, obj: object | None = None) -> TextXSemanticError:
    """Build a TextXSemanticError, attaching `obj`'s source location when available."""
    if obj is None:
        return TextXSemanticError(message)
    try:
        return TextXSemanticError(message, **get_location(obj))
    except Exception:
        return TextXSemanticError(message)


def motion_specs(model: Model) -> Iterable[MotionSpec]:
    """The MotionSpecs declared in `model`."""
    return (spec for spec in model.specs if isinstance(spec, MotionSpec))


def constraint_handlers(model: Model) -> Iterable[ConstraintHandler]:
    """The ConstraintHandlers declared in `model`."""
    return (spec for spec in model.specs if isinstance(spec, ConstraintHandler))


def motion_constraint_items(spec: MotionSpec) -> list[ConstraintSpecification | ConstraintAlias]:
    """All section items (inline specs and aliases) by their local name."""
    return [item for section in spec.sections for item in section.constraints]


def motion_constraints(spec: MotionSpec) -> list[ConstraintSpecification]:
    """All resolved ConstraintSpecification objects for this motion."""
    return [_resolved_spec(item) for item in motion_constraint_items(spec)]


def handler_controller_items(handler: ConstraintHandler) -> list[ControllerEntry | ControllerAlias]:
    """A handler's controller entries and aliases."""
    return list(handler.controllers)


def handler_solver_items(handler: ConstraintHandler) -> list[SolverEntry | SolverAlias]:
    """A handler's solver entries and aliases."""
    return list(handler.solvers)
