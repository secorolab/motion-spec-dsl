# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate what an execution context deploys: the devices it binds and the config it reads."""

from __future__ import annotations

from textx import get_children_of_type

from motion_spec_dsl.classes.context import ConfigValue, ContextQuantity, QuantityType
from motion_spec_dsl.classes.motion_spec import ContextSpec, ExecutionContext, Model
from motion_spec_dsl.classes.validation.common import semantic_error
from motion_spec_dsl.rdf.common import _pose_frame_names

# What a device can realize: an agent, its kinematic tree, or a sensor.
_DEVICE_TARGETS = {"Agent", "KinematicTreeInstance", "ForceTorqueSensorSpec"}


def validate_device_bindings(model: Model) -> None:
    """Raise if a bound device has no address to read, or realizes something no device can."""
    for context in (spec for spec in model.specs if isinstance(spec, ExecutionContext)):
        devices = getattr(context.platform, "devices", None) or ()
        if devices and not context.config:
            raise semantic_error(
                f"Execution context '{context.name}' binds devices but declares no 'config'. "
                "Every bound device needs somewhere to read its address from.",
                context,
            )
        seen: dict[str, str] = {}
        for binding in devices:
            target = binding.target
            uri = getattr(target, "uri", None)
            if uri is None:
                raise semantic_error(
                    f"Execution context '{context.name}' binds '{binding.device}' to "
                    f"'{getattr(target, 'name', target)}', which is not an addressable element.",
                    binding,
                )
            if type(target).__name__ not in _DEVICE_TARGETS:
                raise semantic_error(
                    f"Execution context '{context.name}' binds '{binding.device}' to a "
                    f"{type(target).__name__}. A device realizes an agent or a sensor.",
                    binding,
                )
            if uri in seen:
                raise semantic_error(
                    f"Execution context '{context.name}' binds '{target.name}' twice: "
                    f"'{seen[uri]}' and '{binding.device}'. One element, one device.",
                    binding,
                )
            seen[uri] = binding.device


def validate_config_poses(model: Model) -> None:
    """Raise if a pose read from the deployment config cannot be read once for the run."""
    context = next((spec for spec in model.specs if isinstance(spec, ExecutionContext)), None)
    for quantity in get_children_of_type(ContextQuantity, model):
        if not isinstance(quantity.value, ConfigValue):
            continue
        if not isinstance(getattr(quantity.parent, "parent", None), ContextSpec):
            raise semantic_error(
                f"Pose '{quantity.name}' reads the deployment config, so it must be declared in "
                "a shared context: it is read once for the run, not per motion.",
                quantity,
            )
        if quantity.type != QuantityType.Pose:
            raise semantic_error(
                f"'{quantity.name}' reads the deployment config, which states poses; "
                f"a {quantity.type} cannot come from one.",
                quantity,
            )
        if context is None or not context.config:
            raise semantic_error(
                f"Pose '{quantity.name}' reads the deployment config, but the exec-context "
                'declares no `config: "<file>.toml"`.',
                quantity,
            )
        if _pose_frame_names(quantity) is None:
            raise semantic_error(
                f"Pose '{quantity.name}' reads the deployment config, so the quantity it is "
                "stated `for` must declare of/with-respect-to/as-seen-by frames.",
                quantity,
            )
