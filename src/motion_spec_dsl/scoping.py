# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Reference resolution into imported executable scenes."""

from __future__ import annotations

from textx import get_location, get_model, textx_isinstance
from textx.exceptions import TextXSemanticError
from textx.scoping import Postponed
from textx.scoping.providers import FQNImportURI

from scene_dsl.classes.geom import Frame, IDefaultFrame
from scene_dsl.classes.ktree import KinematicTreeTemplate
from scene_dsl.langs import _pending_refs, build_instance_trees


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


def _named_child(scope, segment):
    stack = list(_contained(scope))
    while stack:
        node = stack.pop()
        name = getattr(node, "name", None)
        if name == segment:
            return node
        if not (isinstance(name, str) and name):
            stack.extend(_contained(node))
    return None


def _lookup(scope, path):
    for segment in path.split("."):
        scope = _named_child(scope, segment)
        if scope is None:
            return None
    return scope


def _as_expected(target, cls):
    """`target` if it is of the expected class; its default frame when a frame is
    expected of a body/tree (scene-dsl IDefaultFrame); None otherwise."""
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


class SceneRefProvider(FQNImportURI):
    """Resolve short references from motion specs into imported executable scenes."""

    def __call__(self, obj, attr, obj_ref):
        target = self._resolve_instanced(obj, attr, obj_ref)
        if target is not None:
            return target

        target = FQNImportURI.__call__(self, obj, attr, obj_ref)
        if target is not None and not _in_template(target):
            return target
        return self._resolve_suffix(obj, obj_ref)

    def _resolve_instanced(self, obj, attr, obj_ref):
        head, _, path = obj_ref.obj_name.partition(".")
        if not path:
            return None
        for model in _all_models(obj):
            for tree in getattr(model, "ktrees", []) or []:
                if getattr(tree, "name", None) != head:
                    continue
                template = getattr(tree, "template", None)
                if template is None:
                    if getattr(tree, "bodies", None):
                        continue
                    return Postponed()
                if not isinstance(template, KinematicTreeTemplate):
                    return Postponed()
                target = _as_expected(_lookup(template, path), obj_ref.cls)
                if target is not None:
                    _pending_refs(model).append((obj, attr.name, tree, target))
                return target
        return None

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
            names = ", ".join(sorted(_fqn(node) for node in matched_nodes))
            raise TextXSemanticError(
                f"'{name}' is ambiguous, qualify it further: {names}",
                **get_location(obj),
            )
        return target


def finalize_imported_scenes(model, metamodel):
    """Build instance-tree copies in imported scene models and land recorded refs.

    textX runs an imported model's processors before its references resolve, so a
    `.scenex` reached from a `.robmot` never gets its instance trees filled -- do it
    here, once the whole repository is resolved. Must run before any processor that
    walks scene objects.
    """
    del metamodel
    repo = getattr(model, "_tx_model_repository", None)
    if repo is None:
        return
    for imported in repo.all_models:
        trees = getattr(imported, "ktrees", None)
        unfilled = any(
            getattr(t, "template", None) is not None and not getattr(t, "bodies", None)
            for t in trees or []
        )
        if unfilled or getattr(imported, "_instanced_refs", None):
            build_instance_trees(imported, None)
