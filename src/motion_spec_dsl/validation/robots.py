# SPDX-License-Identifier: MPL-2.0
"""Robot declaration validation."""

from __future__ import annotations

from motion_spec_dsl.domain import Model, RobotSpec, RobotType
from motion_spec_dsl.validation.common import robot_specs, semantic_error


def robot_component(robot: RobotSpec, component_name: str):
    if robot.type == RobotType.MobileManipulator:
        if component_name == "base":
            return robot.base
        return next(
            (component for component in robot.manipulators if component.name == component_name),
            None,
        )
    if component_name == robot.name or component_name == "chain":
        return robot.chain
    return None


def robot_component_anchor(robot: RobotSpec, component_name: str, anchor: str) -> str:
    component = robot_component(robot, component_name)
    return getattr(component, anchor, "") if component is not None else ""


def validate_robot_specs(model: Model) -> None:
    for robot in robot_specs(model):
        if robot.type == RobotType.MobileManipulator:
            if robot.base is None:
                raise semantic_error(f"Robot '{robot.name}' requires a base component.", robot)
            if not robot.manipulators:
                raise semantic_error(
                    f"Robot '{robot.name}' requires at least one manipulator component.",
                    robot,
                )
            continue

        if robot.type == RobotType.Manipulator and (
            robot.chain is None or not robot.chain.root or not robot.chain.end
        ):
            raise semantic_error(
                f"Manipulator robot '{robot.name}' requires chain root and end.",
                robot,
            )
        if robot.type == RobotType.MobileBase and (robot.chain is None or not robot.chain.root):
            raise semantic_error(f"MobileBase robot '{robot.name}' requires chain root.", robot)
