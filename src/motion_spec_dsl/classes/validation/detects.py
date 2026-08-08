# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate detect acts: where a result can land, and whose status an until item may read."""

from __future__ import annotations

from textx import get_children_of_type

from motion_spec_dsl.classes.constraints import GoalStatusConstraint, _flatten_constraint_items
from motion_spec_dsl.classes.context import (
    GeoPropPair,
    WorldQuantity,
    WorldQuantityType,
)
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import motion_specs, semantic_error


def _pose_subjects(model: Model) -> set[int]:
    """Every scene entity a world-context pose is declared `of:`, by identity."""
    subjects = set()
    for quantity in get_children_of_type(WorldQuantity, model):
        if quantity.type != WorldQuantityType.Pose:
            continue
        for pair in getattr(quantity.props, "pairs", ()) or ():
            if isinstance(pair, GeoPropPair) and pair.key == "of" and pair.frame is not None:
                subjects.add(id(pair.frame))
    return subjects


def validate_detect_targets(model: Model) -> None:
    """A detect result is a pose: every target needs a world pose declared `of:` it to land in."""
    subjects = _pose_subjects(model)
    for motion in motion_specs(model):
        for act in motion.detects:
            for target in act.targets:
                if id(target.ref) in subjects:
                    continue
                raise semantic_error(
                    f"Detect '{act.name}' in motion '{motion.name}' locates "
                    f"'{target.ref.name}', but no world pose is declared 'of:' it, so the "
                    "result has nowhere to land.",
                    act,
                )


def validate_goal_status_acts(model: Model) -> None:
    """A goal status is the outcome of an act this motion sends; another motion's act is not
    running while this one is."""
    for motion in motion_specs(model):
        declared = {id(act) for act in motion.detects}
        for item in _flatten_constraint_items(motion.until.constraints):
            if not isinstance(item, GoalStatusConstraint) or id(item.act) in declared:
                continue
            raise semantic_error(
                f"'{item.name}' reads the status of detect '{item.act.name}', which motion "
                f"'{motion.name}' does not declare.",
                item,
            )
