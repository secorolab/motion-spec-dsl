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
    "trigger": ("satisfied",),
    "hold": ("violated",),
    "flag": ("satisfied",),
    "publish": ("satisfied", "violated"),
}


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


def _validate_publish(monitor) -> None:
    """Raise on a per-state `publish` that names a second topic, repeats within a state, mixes
    an occurrence with authored fields, publishes an occurrence no event produces, or states the
    otherwise without the case it is otherwise to.
    """
    published = monitor.actions("publish")
    if not published:
        return
    _validate_occurrence(monitor, published)
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
    if "violated" in states and "satisfied" not in states:
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes when violated only; the violated publish is "
            "the otherwise of the satisfied one, so author the satisfied publish too. A "
            "violated-only publish is not expressible.",
            monitor,
        )


def _validate_occurrence(monitor, published) -> None:
    """An occurrence carries the event that fired, so it stands where the trigger stands and
    leaves the whole payload to the message type.
    """
    occurrences = [block for block, action in published if action.occurrence]
    if not occurrences:
        return
    if any(action.assignments for _block, action in published):
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes both an occurrence and authored fields; the "
            "occurrence is the whole payload, so one monitor states one or the other.",
            monitor,
        )
    trigger = monitor.trigger
    if trigger is None:
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes an occurrence but triggers no event, so there "
            "is no occurrence to report.",
            monitor,
        )
    for block in occurrences:
        if block.state != trigger[0]:
            raise semantic_error(
                f"Monitor '{monitor.name}' publishes an occurrence in '{block.state}' but "
                f"triggers its event in '{trigger[0]}'; the occurrence is the event firing, so "
                "both stand in the same state.",
                monitor,
            )
