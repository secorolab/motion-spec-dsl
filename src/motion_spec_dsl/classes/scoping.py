# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Resolve source-model references into imported executable scenes."""

from __future__ import annotations

from scene_dsl.classes.ktree import KinematicTreeTemplate
from scene_dsl.langs import InstancedRefScopeProvider
from textx import get_children, get_location, get_model, get_parent_of_type, textx_isinstance
from textx.exceptions import TextXSemanticError


def _fqn(obj) -> str:
    """Dotted path of named ancestors, textX FQN style (unnamed containers are transparent)."""
    parts = []
    while obj is not None:
        name = getattr(obj, "name", None)
        if isinstance(name, str) and name:
            parts.append(name)
        obj = getattr(obj, "parent", None)
    return ".".join(reversed(parts))


def _all_models(obj) -> list:
    model = get_model(obj)
    models = list(model._tx_model_repository.all_models)
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
        if target is not None:
            return target
        return self._resolve_suffix(obj, obj_ref)

    def _resolve_suffix(self, obj, obj_ref):
        name = obj_ref.obj_name
        tail = "." + name
        # Template internals are blueprints: instances carry the world identity.
        matched_nodes = [
            node
            for model in _all_models(obj)
            for node in get_children(lambda node: textx_isinstance(node, obj_ref.cls), model)
            if (_fqn(node) == name or _fqn(node).endswith(tail))
            and get_parent_of_type(KinematicTreeTemplate, node) is None
        ]
        if not matched_nodes:
            return None
        if len(matched_nodes) > 1:
            names = ", ".join(_fqn(node) for node in matched_nodes)
            raise TextXSemanticError(
                f"'{name}' is ambiguous, qualify it further: {names}",
                **get_location(obj),
            )
        (target,) = matched_nodes
        return target
