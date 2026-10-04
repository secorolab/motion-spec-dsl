# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Resolve source-model references into imported executable scenes."""

from __future__ import annotations

from textx import get_location, get_model, textx_isinstance
from textx.exceptions import TextXSemanticError
from textx.scoping import Postponed

from scene_dsl.classes.geom import Frame, IDefaultFrame
from scene_dsl.classes.ktree import KinematicTreeTemplate
from scene_dsl.langs import InstancedRefScopeProvider


def _fqn(obj) -> str:
    """Dotted path of named ancestors, textX FQN style (unnamed containers are transparent)."""
    parts = []
    while obj is not None:
        name = getattr(obj, "name", None)
        if isinstance(name, str) and name:
            parts.append(name)
        obj = getattr(obj, "parent", None)
    return ".".join(reversed(parts))


def _in_template(obj) -> bool:
    """Template internals are blueprints: instances carry the world identity."""
    while obj is not None:
        if isinstance(obj, KinematicTreeTemplate):
            return True
        obj = getattr(obj, "parent", None)
    return False


def _contained(node):
    for tx_attr in getattr(node.__class__, "_tx_attrs", {}).values():
        if not tx_attr.cont:
            continue
        value = getattr(node, tx_attr.name, None)
        for child in value if isinstance(value, list) else [value]:
            if child is not None and hasattr(child, "_tx_attrs"):
                yield child


def _as_expected(target, cls):
    """TARGET if of class CLS, a body's or tree's default frame where a frame is expected, else None."""
    if target is None:
        return None
    if textx_isinstance(target, cls):
        return target
    if cls is Frame and isinstance(target, IDefaultFrame):
        try:
            return target.default_frame
        except ValueError:
            return None
    return None


def _all_models(obj) -> list:
    model = get_model(obj)
    repo = getattr(model, "_tx_model_repository", None)
    models = list(repo.all_models) if repo is not None else []
    if not any(m is model for m in models):
        models.append(model)
    return models


class SceneRefProvider(InstancedRefScopeProvider):
    """Resolve short references from motion specs into imported executable scenes.

    scene-dsl resolves a fully qualified or instanced-tree reference; a short name falls back to
    the one element anywhere in the loaded scenes whose qualified name ends with it.
    """

    def __call__(self, obj, attr, obj_ref):
        target = super().__call__(obj, attr, obj_ref)
        if isinstance(target, Postponed) or (target is not None and not _in_template(target)):
            return target
        return self._resolve_suffix(obj, obj_ref)

    def _resolve_suffix(self, obj, obj_ref):
        name = obj_ref.obj_name
        tail = "." + name
        matched_nodes = []
        target = None
        for model in _all_models(obj):
            stack = [model]
            while stack:
                node = stack.pop()
                stack.extend(_contained(node))
                fqn = _fqn(node)
                if not (fqn == name or fqn.endswith(tail)) or _in_template(node):
                    continue
                coerced = _as_expected(node, obj_ref.cls)
                if coerced is not None:
                    matched_nodes.append(node)
                    target = coerced
        if len(matched_nodes) > 1:
            names = ", ".join(_fqn(node) for node in matched_nodes)
            raise TextXSemanticError(
                f"'{name}' is ambiguous, qualify it further: {names}",
                **get_location(obj),
            )
        return target
