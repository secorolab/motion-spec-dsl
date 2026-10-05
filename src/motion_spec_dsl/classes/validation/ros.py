# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Validate the `ros` block: one block, each group once, one server, and a goal event that acts."""

from __future__ import annotations

from textx import get_location, get_model
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.ros import Ros


def validate_ros(model: Model) -> None:
    """Reject a second `ros` block, group or served action, and a goal event the FSM ignores."""
    blocks = [spec for spec in model.specs if isinstance(spec, Ros)]
    if len(blocks) > 1:
        raise TextXSemanticError(
            f"the model declares {len(blocks)} 'ros' blocks -- a model states its ROS interface "
            "once",
            **get_location(blocks[1]),
        )
    if not blocks:
        return
    seen = set()
    for group in blocks[0].groups:
        if group.name in seen:
            raise TextXSemanticError(
                f"the 'ros' block declares '{group.name}' twice -- each kind of interface is "
                "one group",
                **get_location(group),
            )
        seen.add(group.name)
    servers = blocks[0].servers
    if len(servers) > 1:
        raise TextXSemanticError(
            f"the model serves {len(servers)} actions -- a runtime answers one",
            **get_location(servers[1]),
        )
    for server in servers:
        reference = server.goal_event
        location = get_location(server)
        # A standalone event is the monitor's own: it never reaches the FSM.
        if reference.event is None:
            raise TextXSemanticError(
                f"served action '{server.name}' names event '{reference.name}' -- no imported "
                "FSM event loop declares it",
                **location,
            )
        fsm = get_model(reference.event).fsm
        if not any(event is reference.event for event in fsm.event_loop.events):
            raise TextXSemanticError(
                f"served action '{server.name}' names event '{reference.name}' -- it is not in "
                f"the event loop FSM '{fsm.name}' runs on",
                **location,
            )
        if not any(reaction.when is reference.event for reaction in fsm.reactions):
            raise TextXSemanticError(
                f"served action '{server.name}' starts on event '{reference.name}' -- FSM "
                f"'{fsm.name}' declares no reaction to it, so an accepted goal starts nothing",
                **location,
            )
