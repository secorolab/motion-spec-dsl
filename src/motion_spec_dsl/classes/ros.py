# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""The model's ROS interface: what it publishes, subscribes to, calls, and serves."""

from __future__ import annotations

from dataclasses import dataclass, field

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import NamedNamespaceObject, NamespaceDeclLike
from motion_spec_dsl.classes.constraint_handler import EventName, RosTopicDecl
from motion_spec_dsl.classes.motion_spec import RosActionDecl


@dataclass
class WorldQuantityRef:
    """A reference to the world pose a subscription writes."""

    parent: object
    ref: object


@dataclass
class RosActionServerDecl(NamedNamespaceObject):
    """A served ROS action: the channel goals arrive on, and the event an accepted goal produces.

    What the goal is answered with is stated where the run reaches that point -- on the monitor
    that watches the constraint -- rather than here, which knows only that goals arrive.
    """

    parent: object
    name: str
    channel_name: str
    type_name: str
    goal_event: EventName

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


@dataclass(eq=False)
class RosSubscriptionDecl(NamedNamespaceObject):
    """A standing subscription: the channel poses arrive on, the world poses it writes, and where
    in one detection each pose is found.
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
class RosMeasurementAssign:
    """One field of a standing message, and the measurement component it reports."""

    parent: object
    path: list
    quantity: object
    selector: object

    @property
    def field_path(self) -> str:
        return ".".join(self.path)

    # A view is what the graph already calls "this component of that quantity", so a standing
    # field is read as one: same node names, same scalar, same blackboard channel.
    @property
    def subspace(self):
        return self.selector.subspace

    @property
    def axis(self):
        return self.selector.axis


@dataclass
class RosStandingEntry:
    """One quantity a standing message reports, and the scene entity it reports it of."""

    parent: object
    quantity: object
    subject: object | None = None


@dataclass
class RosStandingPub:
    """A declared topic, published for the whole run at a stated rate.

    A monitor's `publish:` states a verdict while its motion is active. This states readings,
    and belongs to the run rather than to any motion, so it keeps publishing between them. It
    names no entity of its own: the topic is the thing being published, so these are further
    facts about it.
    """

    parent: object
    rate: object
    topic: RosTopicDecl
    # One entry of the message per quantity: a message carrying an array of them reports every
    # one, and a message carrying a quantity whole reports the one it was given.
    entries: list[RosStandingEntry] = field(default_factory=list)
    fields: list = field(default_factory=list)

    @property
    def uri(self):
        """The topic's own node: what is published always is a fact about the topic."""
        return self.topic.uri

    @property
    def name(self) -> str:
        return self.topic.name


@dataclass
class _RosGroup:
    """One kind of ROS interface, named after the kind so an entry resolves by what it is."""

    parent: object

    @property
    def name(self) -> str:
        return self.GROUP

    @property
    def namespace(self) -> Namespace:
        """Its entries mint under the block, so the group is a path segment like any other."""
        return Namespace(self.parent.namespace + self.parent.name + "/")


@dataclass
class RosPublishers(_RosGroup):
    GROUP = "publishers"
    topics: list[RosTopicDecl] = field(default_factory=list)


@dataclass
class RosSubscribers(_RosGroup):
    GROUP = "subscribers"
    subscriptions: list[RosSubscriptionDecl] = field(default_factory=list)


@dataclass
class RosActionClients(_RosGroup):
    GROUP = "action-clients"
    action_clients: list = field(default_factory=list)


@dataclass
class RosActionServers(_RosGroup):
    GROUP = "action-servers"
    action_servers: list[RosActionServerDecl] = field(default_factory=list)


@dataclass
class RosAlways(_RosGroup):
    GROUP = "always"
    standing: list[RosStandingPub] = field(default_factory=list)


@dataclass
class Ros:
    """The model's ROS interface: what it publishes, what it subscribes to, what it calls, and
    what it serves, each in a scope named after the kind and all minting IRIs in one namespace."""

    parent: object
    ns: NamespaceDeclLike
    publishers: RosPublishers | None = None
    subscribers: RosSubscribers | None = None
    action_clients: RosActionClients | None = None
    action_servers: RosActionServers | None = None
    standing: RosAlways | None = None

    @property
    def name(self) -> str:
        """`ros` names the block, so an entry resolves as `<ros.publishers.entry>` -- the
        namespace is where its IRI is minted, not how it is referred to."""
        return "ros"

    @property
    def namespace(self) -> Namespace:
        return Namespace(self.ns.uri)

    @property
    def topics(self) -> list[RosTopicDecl]:
        return self.publishers.topics if self.publishers else []

    @property
    def subscriptions(self) -> list[RosSubscriptionDecl]:
        return self.subscribers.subscriptions if self.subscribers else []

    @property
    def clients(self) -> list:
        return self.action_clients.action_clients if self.action_clients else []

    @property
    def servers(self) -> list[RosActionServerDecl]:
        return self.action_servers.action_servers if self.action_servers else []

    @property
    def always(self) -> list[RosStandingPub]:
        return self.standing.standing if self.standing else []

    @property
    def namespace(self) -> Namespace:
        return Namespace(self.ns.uri)
