# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""The model's ROS interface: what it publishes, subscribes to, calls, and serves."""

from __future__ import annotations

from dataclasses import dataclass, field

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import NamedNamespaceObject, NamespaceDeclLike
from motion_spec_dsl.classes.constraint_handler import EventName, RosTopicDecl, authored_text
from motion_spec_dsl.classes.motion_spec import RosActionDecl


@dataclass
class RosResult:
    """The payload a goal is answered with, authored like a publish."""

    parent: object
    # Sugar form: the one value, whose field the message type resolves at generation.
    value: object | None = None
    fields: list = field(default_factory=list)

    @property
    def assignments(self) -> list[tuple[str, str]]:
        """The `(dot-path, value)` rows this outcome states; the sugar form leaves the path
        empty for the message type to resolve.
        """
        if self.value is not None:
            return [("", authored_text(self.value))]
        return [(".".join(item.path), authored_text(item.value)) for item in self.fields]


@dataclass
class RosActionServerDecl(NamedNamespaceObject):
    """A served ROS action: the channel goals arrive on, the event an accepted goal produces,
    and what the run answers with.
    """

    parent: object
    name: str
    channel_name: str
    type_name: str
    goal_event: EventName
    on_end: RosResult | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def result(self) -> list[tuple[str, str]]:
        """What the run answers with once it reaches the state the model calls its end.

        There is nothing to author for the other case: a run that stopped before its end state
        established none of this, so it leaves every field at the message's own default -- which
        is what the interface chose "unset" to mean.
        """
        return [] if self.on_end is None else self.on_end.assignments


@dataclass(eq=False)
class RosSubscriptionDecl(NamedNamespaceObject):
    """A standing subscription: the channel object poses arrive on, the scene objects the model
    reads off it, and where in one detection the pose is found.
    """

    parent: object
    name: str
    channel_name: str
    type_name: str
    pose_field: str
    pose_container: str
    targets: list = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def pose_path(self) -> str:
        """The dotted path from one detection to the pose it reports."""
        return f"{self.pose_container}.{self.pose_field}"


@dataclass
class Ros:
    """The model's ROS interface: what it publishes, what it subscribes to, what it calls, and
    what it serves, all minted in one namespace."""

    parent: object
    ns: NamespaceDeclLike
    topics: list[RosTopicDecl] = field(default_factory=list)
    subscriptions: list[RosSubscriptionDecl] = field(default_factory=list)
    action_clients: list[RosActionDecl] = field(default_factory=list)
    action_servers: list[RosActionServerDecl] = field(default_factory=list)

    @property
    def name(self) -> str:
        """The namespace prefix names this block, so `<ns.entry>` resolves by dotted path."""
        return self.ns.name

    @property
    def namespace(self) -> Namespace:
        return Namespace(self.ns.uri)
