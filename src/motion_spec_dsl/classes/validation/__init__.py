# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Semantic validation entry point for parsed motion-spec models."""

from __future__ import annotations

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import (
    motion_constraint_items,
    motion_constraints,
)
from motion_spec_dsl.classes.validation.constraints import (
    validate_direction_cosine_components,
    validate_euler_components,
    validate_path_following,
    validate_quaternion_components,
    validate_scalar_order_relations,
    validate_static_path_geometry,
    validate_two_subspace_coordinates,
    validate_unique_constraint_names,
    validate_tolerance_defaults,
    validate_unit_kinds,
)
from motion_spec_dsl.classes.validation.handlers import (
    validate_controller_solver_assembly,
    validate_handler_constraint_assembly,
    validate_handler_requirements,
    validate_mobile_platform_solver_quantity,
)
from motion_spec_dsl.classes.validation.monitors import validate_monitor_state_blocks
from motion_spec_dsl.classes.validation.names import validate_namespace_uris

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
    validate_namespace_uris(model)
    validate_unique_constraint_names(model)
    validate_static_path_geometry(model)
    validate_path_following(model)
    validate_euler_components(model)
    validate_quaternion_components(model)
    validate_direction_cosine_components(model)
    validate_two_subspace_coordinates(model)
    validate_unit_kinds(model)
    validate_scalar_order_relations(model)
    validate_tolerance_defaults(model)
    validate_monitor_state_blocks(model)
    validate_handler_constraint_assembly(model)
    validate_handler_requirements(model)
    validate_controller_solver_assembly(model)
    validate_mobile_platform_solver_quantity(model)
