# SPDX-License-Identifier: MPL-2.0
"""Environment semantic validation."""

from __future__ import annotations

from motion_spec_dsl.domain import (
    EnvironmentAssetType,
    EnvironmentSpec,
    Model,
)
from motion_spec_dsl.validation.common import semantic_error

_ASSET_TYPE_BY_ASSEMBLY_TYPE = {
    "Object": EnvironmentAssetType.SceneObject,
    "Robot": EnvironmentAssetType.RobotAsset,
    "Attachment": EnvironmentAssetType.AttachmentAsset,
}


def validate_environment_assembly_types(model: Model) -> None:
    for spec in model.specs:
        if not isinstance(spec, EnvironmentSpec):
            continue
        for assembly in spec.assembly:
            expected = _ASSET_TYPE_BY_ASSEMBLY_TYPE[str(assembly.type)]
            if assembly.asset.type != expected:
                raise semantic_error(
                    f"Environment assembly '{assembly.name}' is declared as {assembly.type} "
                    f"but uses {assembly.asset.type} asset '{assembly.asset.name}'.",
                    assembly,
                )
            for entry in assembly.entries:
                if entry.__class__.__name__ != "EnvironmentAttachTargetEntry":
                    continue
                target = entry.target_assembly
                if target not in spec.assembly:
                    raise semantic_error(
                        f"Environment assembly '{assembly.name}' attach-to target "
                        f"'{target.name}' is not assembled in environment '{spec.name}'.",
                        entry,
                    )
