# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Validation of user-authored names."""

from __future__ import annotations

import re
from functools import cache
from importlib.resources import files

from textx import get_location
from textx.exceptions import TextXSemanticError

_FIXED_NAME_RULES = frozenset(
    {
        "WorldContextDecl",
        "PreContextDecl",
        "SpecContextDecl",
        "PostContextDecl",
        "WhenSection",
        "WhileSection",
        "UntilSection",
        "UntilMonitorRef",
        "WhenMonitorRef",
        "ControllerParam",
    }
)


def _contained(node):
    for tx_attr in getattr(node.__class__, "_tx_attrs", {}).values():
        if not tx_attr.cont:
            continue
        value = getattr(node, tx_attr.name, None)
        for child in value if isinstance(value, list) else [value]:
            if child is not None and hasattr(child, "_tx_attrs"):
                yield child


@cache
def _grammar_keywords() -> frozenset[str]:
    grammar_dir = files("motion_spec_dsl.grammars")
    grammar_files = (
        "base.tx",
        "context.tx",
        "trajectory.tx",
        "motion_spec.tx",
        "constraint_handler.tx",
        "model.tx",
    )
    return frozenset(
        keyword
        for filename in grammar_files
        for keyword in re.findall(r'"([^\W\d][\w-]*)"', grammar_dir.joinpath(filename).read_text())
    )


def reject_keyword_names(model) -> None:
    """Reject grammar keywords used as names for user-authored objects."""
    stack = [model]
    while stack:
        node = stack.pop()
        stack.extend(_contained(node))
        if type(node).__name__ in _FIXED_NAME_RULES:
            continue
        name = getattr(node, "name", None)
        if isinstance(name, str) and name in _grammar_keywords():
            raise TextXSemanticError(
                f"'{name}' is a motion-spec keyword and cannot name this {type(node).__name__}",
                **get_location(node),
            )
