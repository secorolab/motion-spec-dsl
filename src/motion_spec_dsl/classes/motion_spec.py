# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to motion specification and root-model grammar rules."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from motion_spec_dsl.classes.base import Import, NamespaceDeclare
from motion_spec_dsl.classes.common import (
    IHasNamespaceDeclare,
    NamedNamespaceObject,
    NamespaceDeclLike,
)
from motion_spec_dsl.classes.constraints import (
    ConstraintAlias,
    ConstraintSpecification,
)
from motion_spec_dsl.classes.context import ContextQuantity, WorldQuantity

if TYPE_CHECKING:
    from motion_spec_dsl.classes.constraint_handler import ConstraintHandler


class Model:
    """Root of a parsed motion-spec model: its imports, namespaces and top-level specs."""

    def __init__(
        self,
        imports: list[Import] | None = None,
        namespaces: list[NamespaceDeclare] | None = None,
        specs: list[ExecutionContext | ContextSpec | GuardedMotion | ConstraintHandler]
        | None = None,
        **_,
    ):
        self.imports = imports or []
        self.namespaces = namespaces or []
        self.specs = specs or []


@dataclass
class ExecutionContext(IHasNamespaceDeclare):
    """Select the scene and platform on which this motion specification runs."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    scene: object
    platform: object
    timestep: float
    timestep_unit: str

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class ContextSpec(IHasNamespaceDeclare):
    """A named context block declaring world and context quantities."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class GuardedMotion(IHasNamespaceDeclare):
    """A guarded motion: its when/while/until constraint sections and context."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    description: str | None
    context: list[
        WorldContextDecl | PreContextDecl | SpecContextDecl | PostContextDecl | ContextDeclReference
    ]
    sections: list[WhenSection | WhileSection | UntilSection]

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)
        self.when = self._section("when")
        self.while_ = self._section("while")
        self.until = self._section("until")
        assert len(self.while_.constraints) > 0, (
            "GuardedMotion must have at least one 'while' constraint"
        )

    def _section(self, name: str) -> WhenSection | WhileSection | UntilSection:
        for section in self.sections:
            if section.name == name:
                return section
        raise ValueError(f"GuardedMotion '{self.name}' is missing required {name.upper()} section")


@dataclass
class QuantityContextDecl(NamedNamespaceObject):
    """A context declaration of quantities (world or context)."""

    parent: object
    name: str = ""
    declaration: list[ContextQuantity | WorldQuantity] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


class WorldContextDecl(QuantityContextDecl):
    pass


class PreContextDecl(QuantityContextDecl):
    pass


class SpecContextDecl(QuantityContextDecl):
    pass


class PostContextDecl(QuantityContextDecl):
    pass


@dataclass
class ContextDeclReference:
    parent: object
    ref: QuantityContextDecl


@dataclass
class ConstraintSection(NamedNamespaceObject):
    """A when/while/until section holding constraint items and its combination logic."""

    kind = ""

    parent: object
    constraints: list[ConstraintSpecification | ConstraintAlias] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.kind)


@dataclass
class WhenSection(ConstraintSection):
    kind = "when"
    logic: str | None = None


class WhileSection(ConstraintSection):
    kind = "while"


@dataclass
class UntilSection(ConstraintSection):
    kind = "until"
    logic: str | None = None
