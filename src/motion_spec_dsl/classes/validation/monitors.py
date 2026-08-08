# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validate monitor state blocks: where each action may stand, and what it may say."""

from __future__ import annotations

from motion_spec_dsl.classes.motion_spec import Model
from motion_spec_dsl.classes.validation.common import constraint_handlers, semantic_error

# Where each action is meaningful. `trigger` is an edge, so it needs a state to enter;
# `hold` is the fallback taken while the constraint fails; `flag` reports satisfaction.
_ALLOWED_STATES = {
    "trigger": ("satisfied", "violated"),
    "hold": ("violated",),
    "flag": ("satisfied",),
    "publish": ("satisfied", "violated"),
}

# The message type's trinary contract: what each state publishes is fixed, not authored.
_PUBLISHED_VALUE = {"satisfied": "TRUE", "violated": "FALSE"}


def validate_monitor_state_blocks(model: Model) -> None:
    """Raise on a monitor whose state blocks repeat, misplace an action, author an action
    twice, or declare `publish` more than once.
    """
    for handler in constraint_handlers(model):
        for monitor in handler.monitors:
            _validate_blocks(monitor)
            _validate_actions(monitor)
            _validate_publish(monitor)


def _validate_blocks(monitor) -> None:
    seen = set()
    for block in monitor.states:
        if block.state in seen:
            raise semantic_error(
                f"Monitor '{monitor.name}' declares the '{block.state}' state twice.", monitor
            )
        seen.add(block.state)


def _validate_actions(monitor) -> None:
    for block in monitor.states:
        for action in block.actions:
            allowed = _ALLOWED_STATES[action.kind]
            if block.state not in allowed:
                raise semantic_error(
                    f"Monitor '{monitor.name}' authors '{action.kind}' in '{block.state}'; "
                    f"it belongs in {' or '.join(allowed)}.",
                    monitor,
                )
    for kind in ("trigger", "hold", "flag"):
        if len(monitor.actions(kind)) > 1:
            raise semantic_error(
                f"Monitor '{monitor.name}' authors more than one '{kind}' action.", monitor
            )
    try:
        monitor.topic
    except ValueError as error:
        raise semantic_error(str(error), monitor) from error


def _validate_publish(monitor) -> None:
    """Raise on a per-state `publish` that mixes with the monitor-level form, names a second
    topic, states a value the contract does not fix, or spells out only half the contract.
    """
    published = monitor.actions("publish")
    if not published:
        return
    if monitor.topics:
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes both per state and at monitor level; "
            "author one form or the other.",
            monitor,
        )
    for block, action in published:
        expected = _PUBLISHED_VALUE[block.state]
        if action.value != expected:
            raise semantic_error(
                f"Monitor '{monitor.name}' publishes '{action.value}' in '{block.state}'; "
                f"a {block.state} block publishes the type's {expected}; the value is the "
                "message contract, not a choice.",
                monitor,
            )
    if len({action.topic.uri for _block, action in published}) > 1:
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes to more than one topic; every state of a "
            "monitor publishes the same channel.",
            monitor,
        )
    states = {block.state for block, _action in published}
    if len(states) < len(published):
        raise semantic_error(
            f"Monitor '{monitor.name}' authors more than one 'publish' in one state.", monitor
        )
    if states != set(_PUBLISHED_VALUE):
        missing = (set(_PUBLISHED_VALUE) - states).pop()
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes only in '{states.pop()}', which spells out "
            f"half the contract; add the '{missing}' publish, or write 'publish: to <topic>'.",
            monitor,
        )
