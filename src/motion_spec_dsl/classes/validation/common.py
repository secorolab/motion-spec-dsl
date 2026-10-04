# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""What validators share about a motion's constraints."""

from __future__ import annotations

from motion_spec_dsl.classes.constraints import (
    ConstraintAlias,
    ConstraintSpecification,
    flatten_constraint_items,
)
from motion_spec_dsl.classes.motion_spec import GuardedMotion


def motion_constraints(motion: GuardedMotion) -> list[ConstraintSpecification]:
    """Every constraint a motion's sections hold, groups expanded and aliases resolved."""
    items = flatten_constraint_items(
        [item for section in motion.sections for item in section.constraints]
    )
    return [item.constraint if isinstance(item, ConstraintAlias) else item for item in items]
