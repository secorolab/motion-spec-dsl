# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate monitor state blocks: where each action may stand, and what it may say."""

from __future__ import annotations

from textx import get_children_of_type, get_location
from textx.exceptions import TextXSemanticError

from motion_spec_dsl.classes.constraint_handler import MonitorEntry
from motion_spec_dsl.classes.motion_spec import Model

# `trigger` is an edge, so it needs a state to enter; `hold` is taken while the constraint fails.
_ALLOWED_STATES = {
    "trigger": ("satisfied",),
    "hold": ("violated",),
    "flag": ("satisfied",),
    "publish": ("satisfied", "violated"),
    "result": ("satisfied", "violated"),
}

# A cancel is the client's to ask for: a run answers with what it established.
_ANSWERABLE = ("succeeded", "aborted")


def validate_monitor_state_blocks(model: Model) -> None:
    """Reject repeated or misplaced states and actions, and inconsistent publishes and answers."""
    for monitor in get_children_of_type(MonitorEntry, model):
        location = get_location(monitor)
        states = [block.state for block in monitor.states]
        repeated = [state for state in dict.fromkeys(states) if states.count(state) > 1]
        if repeated:
            raise TextXSemanticError(
                f"monitor '{monitor.name}' declares the '{repeated[0]}' state twice", **location
            )
        if monitor.fallback is not None and monitor.trigger is None:
            raise TextXSemanticError(
                f"monitor '{monitor.name}' holds a fallback motion but triggers no event -- the "
                "fallback is taken on the edge that fires",
                **location,
            )
        for block in monitor.states:
            for action in block.actions:
                if block.state not in _ALLOWED_STATES[action.kind]:
                    raise TextXSemanticError(
                        f"monitor '{monitor.name}' authors '{action.kind}' in '{block.state}' -- "
                        f"it belongs in {' or '.join(_ALLOWED_STATES[action.kind])}",
                        **location,
                    )
        for kind in ("trigger", "hold", "flag", "result"):
            if len(monitor.actions_by_kind.get(kind, [])) > 1:
                raise TextXSemanticError(
                    f"monitor '{monitor.name}' authors more than one '{kind}' action", **location
                )
        if monitor.debounce is not None and monitor.debounce.unit not in ("s", "ms"):
            raise TextXSemanticError(
                f"monitor '{monitor.name}' debounces in '{monitor.debounce.unit}' -- a debounce "
                "is a duration, in 's' or 'ms'",
                **location,
            )
        if monitor.answer is not None and monitor.answer[1].outcome not in _ANSWERABLE:
            raise TextXSemanticError(
                f"monitor '{monitor.name}' answers its goal '{monitor.answer[1].outcome}' -- a "
                f"run answers {' or '.join(_ANSWERABLE)}, and a cancel is the client's to ask for",
                **location,
            )
        published = monitor.published
        if not published:
            continue
        if len({action.topic.uri for _block, action in published}) > 1:
            raise TextXSemanticError(
                f"monitor '{monitor.name}' publishes to more than one topic -- every state of a "
                "monitor publishes the same channel",
                **location,
            )
        announced = [action for _block, action in published if action.events]
        if announced:
            # An announced event rides the event itself, whatever state the monitor is in.
            if any(action.assignments for _block, action in published):
                raise TextXSemanticError(
                    f"monitor '{monitor.name}' publishes both events and authored fields -- an "
                    "announced event is the whole payload",
                    **location,
                )
            if len(announced) > 1:
                raise TextXSemanticError(
                    f"monitor '{monitor.name}' announces events in more than one state -- the "
                    "state block does not change what is sent",
                    **location,
                )
            names = [event.name for event in announced[0].events]
            repeated = [name for name in dict.fromkeys(names) if names.count(name) > 1]
            if repeated:
                raise TextXSemanticError(
                    f"monitor '{monitor.name}' announces {', '.join(repeated)} more than once -- "
                    "one firing sends one message",
                    **location,
                )
            continue
        published_states = [block.state for block, _action in published]
        if len(set(published_states)) < len(published_states):
            raise TextXSemanticError(
                f"monitor '{monitor.name}' authors more than one 'publish' in one state",
                **location,
            )
        if "violated" in published_states and "satisfied" not in published_states:
            raise TextXSemanticError(
                f"monitor '{monitor.name}' publishes when violated only -- the violated publish "
                "is the otherwise of a satisfied one, so author the satisfied publish too",
                **location,
            )
