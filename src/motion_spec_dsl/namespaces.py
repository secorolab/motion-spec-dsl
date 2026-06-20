# SPDX-License-Identifier: MPL-2.0
# Author: Minh Nguyen

from __future__ import annotations

from typing import Any, Protocol

from rdflib.namespace import Namespace
from rdflib.term import URIRef


class NamespaceDeclLike(Protocol):
    name: str
    uri: str


class IHasNamespace(object):
    def __init__(self, **kwargs) -> None:
        self.parent = kwargs.get("parent", None)
        assert self.parent is not None, f"'parent' not handled for type '{self.__class__.__name__}'"


class IHasNamespaceDeclare(IHasNamespace):
    ns: NamespaceDeclLike
    name: str
    uri: Any
    ns_prefix: str
    _ns_obj: Namespace

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        ns = kwargs.get("ns", None)
        assert ns is not None, f"'ns' not provided for {self.__class__.__name__}"
        self.ns = ns
        self.ns_prefix = self.ns.name

        name = kwargs.get("name", None)
        assert name is not None, f"'name' not provided for {self.__class__.__name__}"
        self.name = name

        self._ns_obj = Namespace(self.ns.uri)
        self.uri = self._ns_obj[self.name]

    @property
    def namespace(self) -> Namespace:
        return self._ns_obj


class NamedNamespaceObject(IHasNamespace):
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
