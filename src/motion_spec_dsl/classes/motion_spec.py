# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to motion specification and root-model grammar rules."""

from __future__ import annotations

from motion_spec_dsl.classes.common import IHasNamespaceDeclare, NamedNamespaceObject
from motion_spec_dsl.classes.context import QuantityType, authored_enum
from motion_spec_dsl.classes.coordinates import const_value


class Model:
    """Root of a parsed motion-spec model: its imports, namespaces and top-level specs."""

    def __init__(self, imports, namespaces, specs) -> None:
        self.imports = imports
        self.namespaces = namespaces
        self.specs = specs


class ExecutionContext(IHasNamespaceDeclare):
    """The scene and platform a motion specification runs on."""

    def __init__(self, parent, ns, name, scene, platform, config, timestep, timestep_unit) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.scene = scene
        self.platform = platform
        self.config = config or None
        self.timestep = const_value(timestep)
        self.timestep_unit = timestep_unit


class ToleranceDefaults:
    """Model-wide satisfaction bands, resolved onto each constraint as it is emitted."""

    def __init__(self, parent, defaults) -> None:
        self.parent = parent
        self.defaults = defaults


class ToleranceDefault:
    """The band a constraint over `kind` is satisfied within unless it authors its own."""

    def __init__(self, parent, kind, band) -> None:
        self.parent = parent
        self.kind = authored_enum(QuantityType, str(kind))
        self.band = band


class ContextSpec(IHasNamespaceDeclare):
    """A named context block declaring world and context quantities."""

    def __init__(self, parent, ns, name, context) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.context = context


class GuardedMotion(IHasNamespaceDeclare):
    """A guarded motion: its context and its when, while and until sections."""

    def __init__(self, parent, ns, name, description, context, detects, sections) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.description = description or None
        self.context = context
        self.detects = detects
        self.sections = sections
        by_kind = {section.kind: section for section in sections}
        self.when = by_kind.get("when")
        self.while_ = by_kind.get("while")
        self.until = by_kind.get("until")


class RosActionDecl(NamedNamespaceObject):
    """A ROS action a motion sends goals on, and where a result holds a detection's pose."""

    def __init__(self, parent, name, channel_name, type_name, pose_field, pose_container) -> None:
        super().__init__(parent=parent, name=name)
        self.channel_name = channel_name
        self.type_name = type_name
        self.pose_field = pose_field
        self.pose_container = pose_container
        self.pose_path = f"{pose_container}.{pose_field}" if pose_field else ""


class SceneObjRef:
    """A scene object a detect act targets."""

    def __init__(self, parent, ref) -> None:
        self.parent = parent
        self.ref = ref


class DetectDecl(NamedNamespaceObject):
    """A detect act: the scene objects a motion locates on entry, and the action it asks."""

    def __init__(self, parent, name, targets, action) -> None:
        super().__init__(parent=parent, name=name)
        self.targets = targets
        self.action = action


class QuantityContextDecl(NamedNamespaceObject):
    """A `world`, `pre`, `spec` or `post` block of quantities; its kind is its name."""

    def __init__(self, parent, name, declaration) -> None:
        super().__init__(parent=parent, name=name)
        self.declaration = declaration


class ContextDeclReference:
    def __init__(self, parent, ref) -> None:
        self.parent = parent
        self.ref = ref


class ConstraintSection(NamedNamespaceObject):
    """A `when`, `while` or `until` section and how its constraints combine."""

    def __init__(self, parent, kind, logic, constraints) -> None:
        super().__init__(parent=parent, name=kind)
        self.kind = kind
        self.logic = logic or None
        self.constraints = constraints
