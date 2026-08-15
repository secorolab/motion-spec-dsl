# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate perceived poses: where a result can land, whose status an until item may read, and
that each observed object has exactly one source writing its pose."""

from __future__ import annotations

from textx import get_children_of_type

from motion_spec_dsl.classes.constraints import GoalStatusConstraint, _flatten_constraint_items
from motion_spec_dsl.classes.context import (
    GeoPropPair,
    WorldQuantity,
    WorldQuantityType,
)
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.ros import RosSubscriptionDecl
from motion_spec_dsl.classes.validation.common import motion_specs, semantic_error


def _pose_subjects(model: Model) -> dict[int, int]:
    """Every scene entity's world-pose identity, keyed by the entity identity."""
    subjects = {}
    for quantity in get_children_of_type(WorldQuantity, model):
        if quantity.type != WorldQuantityType.Pose:
            continue
        for pair in getattr(quantity.props, "pairs", ()) or ():
            if isinstance(pair, GeoPropPair) and pair.key == "of" and pair.frame is not None:
                subjects[id(pair.frame)] = id(quantity)
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


def validate_subscription_targets(model: Model) -> None:
    """A subscription names each world pose it writes, and every pose has one producer."""
    # Detects seed the set: a subscription and a detect claiming the same object is the same
    # two-producer error as two subscriptions doing so.
    subjects = _pose_subjects(model)
    observed = {
        subjects[id(target.ref)]
        for motion in motion_specs(model)
        for act in motion.detects
        for target in act.targets
        if id(target.ref) in subjects
    }

    for sub in get_children_of_type(RosSubscriptionDecl, model):
        for target in sub.targets:
            if target.ref.type != WorldQuantityType.Pose:
                raise semantic_error(
                    f"Subscription '{sub.name}' observes '{target.ref.name}', which is not a pose.",
                    sub,
                )
            if id(target.ref) in observed:
                raise semantic_error(
                    f"'{target.ref.name}' is observed by more than one source; a world pose has "
                    "one producer.",
                    sub,
                )
            observed.add(id(target.ref))


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
