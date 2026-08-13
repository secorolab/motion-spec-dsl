# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate the `ros` block: one block, one server, and a goal event the imported FSM acts on."""

from __future__ import annotations

from textx import get_model

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.ros import Ros
from motion_spec_dsl.classes.validation.common import semantic_error


def validate_ros(model: Model) -> None:
    """Raise on a second `ros` block, on a second served action, on a goal event no imported
    FSM event loop declares, or on one the FSM never reacts to.
    """
    blocks = [spec for spec in model.specs if isinstance(spec, Ros)]
    if len(blocks) > 1:
        raise semantic_error(
            f"Model declares {len(blocks)} 'ros' blocks; a model states its ROS interface once.",
            blocks[1],
        )
    if not blocks:
        return

    servers = blocks[0].servers
    if len(servers) > 1:
        raise semantic_error(
            f"Model serves {len(servers)} actions; a runtime answers one.",
            servers[1],
        )
    for server in servers:
        _validate_goal_event(server)


def _validate_goal_event(server) -> None:
    """The event an accepted goal produces has to reach the FSM and start something there."""
    reference = server.goal_event
    # A standalone event is monitor-owned: it never reaches the FSM, so it cannot start a goal.
    if reference.event is None:
        raise semantic_error(
            f"Served action '{server.name}' names event '{reference.name}', which no imported "
            "FSM event loop declares.",
            server,
        )

    fsm = get_model(reference.event).fsm
    declared = {id(event) for event in fsm.event_loop.events}
    if id(reference.event) not in declared:
        raise semantic_error(
            f"Served action '{server.name}' names event '{reference.name}', which is not in "
            f"the event loop FSM '{fsm.name}' runs on.",
            server,
        )

    if not any(reaction.when is reference.event for reaction in fsm.reactions):
        raise semantic_error(
            f"Served action '{server.name}' starts on event '{reference.name}', but FSM "
            f"'{fsm.name}' declares no reaction to it, so an accepted goal would start nothing.",
            server,
        )
