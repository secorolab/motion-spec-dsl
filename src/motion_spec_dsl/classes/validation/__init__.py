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
    validate_alignment_views,
    validate_direction_cosine_components,
    validate_euler_components,
    validate_geometric_distance_views,
    validate_line_plane_primitives,
    validate_path_following,
    validate_quaternion_components,
    validate_scalar_order_relations,
    validate_static_path_geometry,
    validate_tolerance_defaults,
    validate_two_subspace_coordinates,
    validate_unique_constraint_names,
    validate_unit_kinds,
)
from motion_spec_dsl.classes.validation.detects import (
    validate_camera_providers,
    validate_detect_targets,
    validate_goal_status_acts,
    validate_subscription_targets,
)
from motion_spec_dsl.classes.validation.expressions import (
    validate_controlled_expressions,
    validate_expression_dimensions,
)
from motion_spec_dsl.classes.validation.handlers import (
    validate_commanded_quantity_is_measured,
    validate_controller_solver_assembly,
    validate_handler_constraint_assembly,
    validate_handler_requirements,
    validate_mobile_platform_solver_quantity,
)
from motion_spec_dsl.classes.validation.monitors import validate_monitor_state_blocks
from motion_spec_dsl.classes.validation.names import validate_namespace_uris
from motion_spec_dsl.classes.validation.ros import validate_ros

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
    validate_expression_dimensions(model)
    validate_scalar_order_relations(model)
    validate_alignment_views(model)
    validate_line_plane_primitives(model)
    validate_geometric_distance_views(model)
    validate_tolerance_defaults(model)
    validate_detect_targets(model)
    validate_subscription_targets(model)
    validate_camera_providers(model)
    validate_goal_status_acts(model)
    validate_monitor_state_blocks(model)
    validate_handler_constraint_assembly(model)
    validate_handler_requirements(model)
    validate_controller_solver_assembly(model)
    validate_commanded_quantity_is_measured(model)
    validate_controlled_expressions(model)
    validate_mobile_platform_solver_quantity(model)
    validate_ros(model)
