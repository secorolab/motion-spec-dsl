# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The model's ROS interface: what it publishes, subscribes to, calls, and serves."""

from __future__ import annotations

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import NamedNamespaceObject


class WorldQuantityRef:
    """A world pose a subscription writes."""

    def __init__(self, parent, ref) -> None:
        self.parent = parent
        self.ref = ref


class CameraTargetRef:
    """A camera whose images a subscription carries."""

    def __init__(self, parent, ref) -> None:
        self.parent = parent
        self.ref = ref


class RosActionServerDecl(NamedNamespaceObject):
    """A served ROS action: where goals arrive, and the event an accepted goal produces."""

    def __init__(self, parent, name, channel_name, type_name, goal_event) -> None:
        super().__init__(parent=parent, name=name)
        self.channel_name = channel_name
        self.type_name = type_name
        self.goal_event = goal_event


class RosSubscriptionDecl(NamedNamespaceObject):
    """A standing subscription: detections and the poses they write, or a camera's images."""

    def __init__(
        self,
        parent,
        name,
        channel_name,
        type_name,
        targets,
        pose_field,
        pose_container,
        observed,
        cameras,
    ) -> None:
        super().__init__(parent=parent, name=name)
        self.channel_name = channel_name
        self.type_name = type_name
        self.targets = targets
        self.pose_field = pose_field
        self.pose_container = pose_container
        self.observed = observed
        self.cameras = cameras
        # A camera's images carry no pose.
        self.pose_path = f"{pose_container}.{pose_field}" if pose_field and pose_container else None


class RosMeasurementAssign:
    """One field of a standing message and the quantity component it reports, read as a view."""

    def __init__(self, parent, path, quantity, selector) -> None:
        self.parent = parent
        self.path = path
        self.quantity = quantity
        self.selector = selector
        self.field_path = ".".join(path)
        self.subspace = selector.subspace
        self.axis = selector.axis


class RosStandingEntry:
    """One quantity a standing message reports, and the scene entity it reports it of."""

    def __init__(self, parent, subject, quantity) -> None:
        self.parent = parent
        self.subject = subject
        self.quantity = quantity


class RosStandingPub:
    """A declared topic published for the whole run at a rate; its node is the topic's."""

    def __init__(self, parent, rate, topic, entries, fields) -> None:
        self.parent = parent
        self.rate = rate
        self.topic = topic
        self.entries = entries
        self.fields = fields


class RosGroup(NamedNamespaceObject):
    """One kind of ROS interface, named after the kind so an entry resolves by what it is."""

    def __init__(self, parent, name, entries) -> None:
        super().__init__(parent=parent, name=name)
        self.entries = entries


class Ros:
    """The model's ROS interface; entries resolve as `<ros.<kind>.<entry>>` and mint under `ns`."""

    name = "ros"

    def __init__(self, parent, ns, groups) -> None:
        self.parent = parent
        self.ns = ns
        self.groups = groups
        self.namespace = Namespace(ns.uri)
        entries = {group.name: group.entries for group in groups}
        self.topics = entries.get("publishers", [])
        self.subscriptions = entries.get("subscribers", [])
        self.clients = entries.get("action-clients", [])
        self.servers = entries.get("action-servers", [])
        self.always = entries.get("always", [])
