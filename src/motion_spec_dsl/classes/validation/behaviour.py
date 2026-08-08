# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate the `bdd-behaviour` block: one server, and events the imported FSM really has."""

from __future__ import annotations

from textx import get_model

from motion_spec_dsl.classes.constraint_handler import BddBehaviour
from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import semantic_error


def validate_bdd_behaviour(model: Model) -> None:
    """Raise on a second `bdd-behaviour` block, on an event no imported FSM event loop
    declares, or on a goal event the FSM never reacts to.
    """
    blocks = [spec for spec in model.specs if isinstance(spec, BddBehaviour)]
    if len(blocks) > 1:
        raise semantic_error(
            f"Model declares {len(blocks)} 'bdd-behaviour' blocks; a runtime answers one action.",
            blocks[1],
        )
    if not blocks:
        return

    block = blocks[0]
    references = [block.goal_event, *block.exported]
    for reference in references:
        # A standalone event is monitor-owned: it never reaches the FSM, so it cannot start a
        # goal and there is nothing to export.
        if reference.event is None:
            raise semantic_error(
                f"'bdd-behaviour {block.name}' names event '{reference.name}', which no imported "
                "FSM event loop declares.",
                block,
            )

    fsm = get_model(block.goal_event.event).fsm
    declared = {id(event) for event in fsm.event_loop.events}
    for reference in references:
        if id(reference.event) not in declared:
            raise semantic_error(
                f"'bdd-behaviour {block.name}' names event '{reference.name}', which is not in "
                f"the event loop FSM '{fsm.name}' runs on.",
                block,
            )

    if not any(reaction.when is block.goal_event.event for reaction in fsm.reactions):
        raise semantic_error(
            f"'bdd-behaviour {block.name}' starts on event '{block.goal_event.name}', but FSM "
            f"'{fsm.name}' declares no reaction to it, so an accepted goal would start nothing.",
            block,
        )
