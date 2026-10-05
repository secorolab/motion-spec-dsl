# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate perceived poses: where a result lands, whose status an until item reads, one source."""

from __future__ import annotations

from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.constraints import GoalStatusConstraint, flatten_constraint_items
from motion_spec_dsl.classes.context import GeometricProps, WorldQuantity, WorldQuantityType
from motion_spec_dsl.classes.motion_spec import GuardedMotion, Model
from motion_spec_dsl.classes.ros import RosSubscriptionDecl


def validate_detects(model: Model) -> None:
    """Reject a detect target with no world pose, a second producer of a pose, and foreign acts."""
    # A scene entity's world pose, by the entity's identity.
    subjects = {}
    for quantity in get_children_of_type(WorldQuantity, model):
        if quantity.type != WorldQuantityType.Pose or not isinstance(quantity.props, GeometricProps):
            continue
        for pair in quantity.props.pairs:
            if pair.key == "of" and pair.frame is not None:
                subjects[id(pair.frame)] = id(quantity)
    motions = get_children_of_type(GuardedMotion, model)
    observed = set()
    for motion in motions:
        declared = {id(act) for act in motion.detects}
        for act in motion.detects:
            for target in act.targets:
                if id(target.ref) not in subjects:
                    raise TextXSemanticError(
                        f"detect '{act.name}' in motion '{motion.name}' locates "
                        f"'{target.ref.name}' -- no world pose is declared 'of:' it, so the "
                        "result has nowhere to land",
                        **get_location(act),
                    )
                observed.add(subjects[id(target.ref)])
        until = motion.until.constraints if motion.until is not None else []
        for item in flatten_constraint_items(until):
            if isinstance(item, GoalStatusConstraint) and id(item.act) not in declared:
                raise TextXSemanticError(
                    f"'{item.name}' reads the status of detect '{item.act.name}' -- motion "
                    f"'{motion.name}' does not send it",
                    **get_location(item),
                )
    provided = set()
    for sub in get_children_of_type(RosSubscriptionDecl, model):
        location = get_location(sub)
        for target in sub.targets:
            if target.ref.type != WorldQuantityType.Pose:
                raise TextXSemanticError(
                    f"subscription '{sub.name}' observes '{target.ref.name}' -- it is not a pose",
                    **location,
                )
            if id(target.ref) in observed:
                raise TextXSemanticError(
                    f"'{target.ref.name}' is observed by more than one source -- a world pose "
                    "has one producer",
                    **location,
                )
            observed.add(id(target.ref))
        if len(sub.cameras) > 1:
            raise TextXSemanticError(
                f"subscription '{sub.name}' carries {len(sub.cameras)} cameras -- a channel "
                "carries one, or nothing tells the images apart",
                **location,
            )
        for target in sub.cameras:
            camera = target.ref
            if camera.cam_type != "rgb":
                raise TextXSemanticError(
                    f"subscription '{sub.name}' carries '{camera.name}', a {camera.cam_type} "
                    "camera -- only rgb is read",
                    **location,
                )
            if id(camera) in provided:
                raise TextXSemanticError(
                    f"'{camera.name}' is carried by more than one channel -- a camera has one "
                    "provider",
                    **location,
                )
            provided.add(id(camera))
