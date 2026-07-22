# SPDX-License-Identifier: MPL-2.0
"""Namespace foundation for URI-bearing DSL objects, following scene-dsl's classes/common.py."""

from __future__ import annotations

from typing import Protocol

from rdflib import Namespace, URIRef


class NamespaceDeclLike(Protocol):
    """Structural type for a namespace declaration: a `name` (prefix) and a `uri`."""

    name: str
    uri: str


class IHasParent:
    def __init__(self, **kwargs) -> None:
        self.parent = kwargs.get("parent", None)
        if self.parent is None:
            raise ValueError(f"'parent' not handled for type '{self.__class__.__name__}'")


class IHasNamespace(IHasParent):
    @property
    def namespace(self) -> Namespace:
        raise NotImplementedError(
            f"'namespace' property not implemented for '{self.__class__.__name__}'"
        )


class IHasNamespaceDeclare(IHasNamespace):
    """Root of a namespace: everything below it mints IRIs under `ns`."""

    name: str
    ns_prefix: str
    _ns_obj: Namespace

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ns = kwargs.get("ns", None)
        if self.ns is None:
            raise ValueError("Namespace declaration requires 'ns'")
        self.ns_prefix = self.ns.name

        self.name = kwargs.get("name", None)
        if self.name is None:
            raise ValueError("Namespace declaration requires 'name'")

        self._ns_obj = Namespace(self.ns.uri)

    @property
    def namespace(self) -> Namespace:
        return self._ns_obj

    @property
    def uri(self) -> URIRef:
        return self._ns_obj[self.name]

    def __str__(self) -> str:
        return f"<({self.__class__.__name__}) {self.ns_prefix}:{self.name}>"


class NamedNamespaceObject(IHasNamespace):
    """A named DSL object whose URI is derived from its parent's namespace and its name."""

    name: str
    _uri: URIRef | str

    def __init__(self, parent, name: str, **kwargs):
        super().__init__(parent=parent)
        self.name = name
        self._uri = ""

    @property
    def namespace(self) -> Namespace:
        assert self.parent is not None, f"'parent' not set for '{self.__class__.__name__}'"
        return Namespace(self.parent.namespace + self.parent.name + "/")

    @property
    def uri(self) -> str:
        if self._uri == "":
            self._uri = self.namespace[self.name]
        return self._uri
