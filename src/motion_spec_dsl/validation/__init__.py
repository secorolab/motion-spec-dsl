# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Semantic validation entrypoint for fully constructed motion-spec DSL models."""

from __future__ import annotations

from motion_spec_dsl.classes.motion import Model
from motion_spec_dsl.validation.common import (
    motion_constraint_items,
    motion_constraints,
)
from motion_spec_dsl.validation.constraints import (
    validate_constraint_context_refs,
    validate_constraint_value_types,
    validate_context_quantity_values,
    validate_unique_constraint_names,
)
from motion_spec_dsl.validation.controllers import (
    validate_achd_acceleration_constraints,
    validate_controller_commands,
)
from motion_spec_dsl.validation.handlers import (
    validate_handler_constraint_assembly,
    validate_handler_requirements,
    validate_motion_spec_coverage,
)
from motion_spec_dsl.validation.names import reject_keyword_names
from motion_spec_dsl.validation.solvers import (
    validate_controller_solver_refs,
    validate_handler_control_mode_solver_compatibility,
    validate_mixed_solver_domains,
    validate_solver_limits,
    validate_supported_solver_algorithms,
)

__all__ = [
    "motion_constraint_items",
    "motion_constraints",
    "validate_model",
]


def validate_model(model: Model, metamodel=None) -> None:
    """Run every semantic validator for `model` (the textx model processor); raises on the
    first violation.
    """
    del metamodel
    reject_keyword_names(model)
    validate_unique_constraint_names(model)
    validate_constraint_context_refs(model)
    validate_context_quantity_values(model)
    validate_constraint_value_types(model)
    validate_handler_constraint_assembly(model)
    validate_handler_requirements(model)
    validate_controller_solver_refs(model)
    validate_handler_control_mode_solver_compatibility(model)
    validate_supported_solver_algorithms(model)
    validate_solver_limits(model)
    validate_mixed_solver_domains(model)
    validate_controller_commands(model)
    validate_achd_acceleration_constraints(model)
    validate_motion_spec_coverage(model)
