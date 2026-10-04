# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate what an execution context deploys: the devices it binds and the config it reads."""

from __future__ import annotations

from scene_dsl.classes.ktree import KinematicTreeInstance
from scene_dsl.classes.scene import Agent
from scene_dsl.classes.sensors import ForceTorqueSensorSpec
from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.context import ConfigValue, ContextQuantity, QuantityType, pose_frame_names
from motion_spec_dsl.classes.motion_spec import ContextSpec, ExecutionContext, Model


def validate_device_bindings(model: Model) -> None:
    """Reject devices with no config to read an address from, or bound to what no device realizes."""
    for context in get_children_of_type(ExecutionContext, model):
        devices = context.platform.devices if context.platform.kind == "real-world" else []
        if devices and not context.config:
            raise TextXSemanticError(
                f"execution context '{context.name}' binds devices but declares no 'config' -- "
                "every bound device reads its address from it",
                **get_location(context),
            )
        bound: dict[str, str] = {}
        for binding in devices:
            target = binding.target
            if not isinstance(target, (Agent, KinematicTreeInstance, ForceTorqueSensorSpec)):
                raise TextXSemanticError(
                    f"execution context '{context.name}' binds '{binding.device}' to a "
                    f"{type(target).__name__} -- a device realizes an agent, its tree, or a sensor",
                    **get_location(binding),
                )
            if str(target.uri) in bound:
                raise TextXSemanticError(
                    f"execution context '{context.name}' binds '{target.name}' to both "
                    f"'{bound[str(target.uri)]}' and '{binding.device}' -- one element, one device",
                    **get_location(binding),
                )
            bound[str(target.uri)] = binding.device


def validate_config_poses(model: Model) -> None:
    """Reject config-read poses that are not shared, not poses, unconfigured or unframed."""
    context = next((spec for spec in model.specs if isinstance(spec, ExecutionContext)), None)
    for quantity in get_children_of_type(ContextQuantity, model):
        if not isinstance(quantity.value, ConfigValue):
            continue
        location = get_location(quantity)
        if not isinstance(quantity.parent.parent, ContextSpec):
            raise TextXSemanticError(
                f"pose '{quantity.name}' reads the deployment config outside a shared context -- "
                "it is read once for the run, not per motion",
                **location,
            )
        if quantity.type != QuantityType.Pose:
            raise TextXSemanticError(
                f"'{quantity.name}' reads the deployment config as a {quantity.type} -- the "
                "config states poses",
                **location,
            )
        if context is None or not context.config:
            raise TextXSemanticError(
                f"pose '{quantity.name}' reads the deployment config -- the exec-context "
                'declares no `config: "<file>.toml"`',
                **location,
            )
        if pose_frame_names(quantity) is None:
            raise TextXSemanticError(
                f"pose '{quantity.name}' reads the deployment config -- the quantity it is stated "
                "`for` must declare of, wrt and as-seen-by frames",
                **location,
            )
