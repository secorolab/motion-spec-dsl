# SPDX-License-Identifier: MPL-2.0
"""Namespace foundation for IRI-bearing DSL objects, after scene-dsl's classes/common.py."""

from __future__ import annotations

from rdflib import Namespace, URIRef


class IHasNamespaceDeclare:
    """Root of a namespace: everything below it mints IRIs under `ns`."""

    def __init__(self, **kwargs) -> None:
        self.parent = kwargs.get("parent")
        if self.parent is None:
            raise ValueError(f"'parent' not handled for type '{self.__class__.__name__}'")
        self.ns = kwargs.get("ns")
        if self.ns is None:
            raise ValueError("a namespace declaration requires 'ns'")
        self.ns_prefix = self.ns.name
        self.name = kwargs.get("name")
        if self.name is None:
            raise ValueError("a namespace declaration requires 'name'")
        self.namespace = Namespace(self.ns.uri)
        self.uri = self.namespace[self.name]

    def __str__(self) -> str:
        return f"<({self.__class__.__name__}) {self.ns_prefix}:{self.name}>"


class NamedNamespaceObject:
    """A named DSL object whose IRI is its parent's namespace plus its parent's and its own name."""

    # An alias may read this IRI before textX has initialized the object.
    _uri: URIRef | None = None

    def __init__(self, parent, name: str) -> None:
        if parent is None:
            raise ValueError(f"'parent' not handled for type '{self.__class__.__name__}'")
        self.parent = parent
        self.name = name

    @property
    def namespace(self) -> Namespace:
        return Namespace(self.parent.namespace + self.parent.name + "/")

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri
