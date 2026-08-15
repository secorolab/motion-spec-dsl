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
    "result": ("satisfied", "violated"),
}

# A run answers its own goal with what it established; it does not cancel it -- a cancel is
# the client's to ask for, and the runtime reports it wherever it stops.
_ANSWERABLE = ("succeeded", "aborted")


def validate_monitor_state_blocks(model: Model) -> None:
    """Raise on a monitor whose state blocks repeat, misplace an action, author an action
    twice, or declare `publish` more than once.
    """
    for handler in constraint_handlers(model):
        for monitor in handler.monitors:
            _validate_blocks(monitor)
            _validate_actions(monitor)
            _validate_publish(monitor)
            _validate_result(monitor)


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
    for kind in ("trigger", "hold", "flag", "result"):
        if len(monitor.actions(kind)) > 1:
            raise semantic_error(
                f"Monitor '{monitor.name}' authors more than one '{kind}' action.", monitor
            )


def _validate_publish(monitor) -> None:
    """Raise on a per-state `publish` that names a second topic, repeats within a state, mixes
    announced events with authored fields, or states the otherwise without the case it is
    otherwise to.
    """
    published = monitor.actions("publish")
    if not published:
        return
    if len({action.topic.uri for _block, action in published}) > 1:
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes to more than one topic; every state of a "
            "monitor publishes the same channel.",
            monitor,
        )
    if _validate_announced_events(monitor, published):
        return
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


def _validate_result(monitor) -> None:
    """Raise on an answer that states a status the run cannot reach on its own."""
    answer = monitor.answer
    if answer is None:
        return
    _block, action = answer
    if action.outcome not in _ANSWERABLE:
        raise semantic_error(
            f"Monitor '{monitor.name}' answers its goal '{action.outcome}'; a run answers "
            f"{' or '.join(_ANSWERABLE)}, and a cancel is the client's to ask for.",
            monitor,
        )


def _validate_announced_events(monitor, published) -> bool:
    """An announced event is published on the cycle the FSM sees it, whatever state the monitor
    is in, so it is stated once per monitor and never beside authored fields.
    """
    announced = [action for _block, action in published if action.events]
    if not announced:
        return False
    if any(action.assignments for _block, action in published):
        raise semantic_error(
            f"Monitor '{monitor.name}' publishes both events and authored fields; an announced "
            "event is the whole payload, so one monitor states one or the other.",
            monitor,
        )
    if len(announced) > 1:
        raise semantic_error(
            f"Monitor '{monitor.name}' announces events in more than one state; an announced "
            "event rides the event itself, so its state block does not change what is sent.",
            monitor,
        )
    names = [event.name for event in announced[0].events]
    repeated = sorted({name for name in names if names.count(name) > 1})
    if repeated:
        raise semantic_error(
            f"Monitor '{monitor.name}' announces {', '.join(repeated)} more than once; one "
            "firing sends one message.",
            monitor,
        )

    return True
