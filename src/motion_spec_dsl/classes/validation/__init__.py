# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Semantic validation entry point for parsed motion-spec models."""

from __future__ import annotations

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.constraints import (
    validate_admittance,
    validate_alignment_targets,
    validate_bare_axis_selectors,
    validate_context_geometry,
    validate_coordinate_components,
    validate_direction_operands,
    validate_geometric_distance_views,
    validate_motion_sections,
    validate_motion_world_scope,
    validate_path_following,
    validate_resolved_views,
    validate_scalar_order_relations,
    validate_static_path_geometry,
    validate_unique_constraint_names,
    validate_unit_kinds,
    validate_view_operands,
    validate_world_quantities,
)
from motion_spec_dsl.classes.validation.detects import validate_detects
from motion_spec_dsl.classes.validation.execution_context import (
    validate_config_poses,
    validate_device_bindings,
)
from motion_spec_dsl.classes.validation.expressions import (
    validate_controlled_expressions,
    validate_expression_dimensions,
    validate_sampled_quantities,
)
from motion_spec_dsl.classes.validation.handlers import (
    validate_commanded_quantity_is_measured,
    validate_controller_commands,
    validate_controller_solver_assembly,
    validate_handler_constraint_assembly,
    validate_handler_requirements,
    validate_kinematics_solvers,
    validate_mobile_platform_solver_quantity,
)
from motion_spec_dsl.classes.validation.monitors import validate_monitor_state_blocks
from motion_spec_dsl.classes.validation.names import validate_namespace_uris
from motion_spec_dsl.classes.validation.perturbations import validate_perturbations
from motion_spec_dsl.classes.validation.ros import validate_ros


def validate_model(model: Model, metamodel=None) -> None:
    """Run every semantic validator on MODEL, the textX model processor; raise on the first error."""
    del metamodel
    validate_namespace_uris(model)
    validate_motion_sections(model)
    validate_unique_constraint_names(model)
    validate_static_path_geometry(model)
    validate_path_following(model)
    validate_coordinate_components(model)
    validate_world_quantities(model)
    validate_context_geometry(model)
    validate_unit_kinds(model)
    validate_admittance(model)
    validate_bare_axis_selectors(model)
    validate_expression_dimensions(model)
    validate_scalar_order_relations(model)
    validate_direction_operands(model)
    validate_alignment_targets(model)
    validate_geometric_distance_views(model)
    validate_view_operands(model)
    validate_motion_world_scope(model)
    validate_resolved_views(model)
    validate_detects(model)
    validate_monitor_state_blocks(model)
    validate_handler_constraint_assembly(model)
    validate_handler_requirements(model)
    validate_controller_solver_assembly(model)
    validate_kinematics_solvers(model)
    validate_commanded_quantity_is_measured(model)
    validate_controller_commands(model)
    validate_controlled_expressions(model)
    validate_mobile_platform_solver_quantity(model)
    validate_perturbations(model)
    validate_device_bindings(model)
    validate_config_poses(model)
    validate_sampled_quantities(model)
    validate_ros(model)
