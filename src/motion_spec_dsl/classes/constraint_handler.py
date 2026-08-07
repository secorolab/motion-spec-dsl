# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Constraint-handler, monitor, controller, and solver model classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import (
    IHasNamespaceDeclare,
    NamedNamespaceObject,
    NamespaceDeclLike,
)
from motion_spec_dsl.classes.constraints import ConstraintRef
from motion_spec_dsl.classes.context import (
    ContextRef,
    Measure,
    QuantityType,
    View,
    _authored_enum,
)
from motion_spec_dsl.classes.coordinates import Coordinates
from motion_spec_dsl.classes.motion_spec import (
    ContextDeclReference,
    GuardedMotion,
    SpecContextDecl,
    WorldContextDecl,
)


class ControllerType(StrEnum):
    PID = "PID"
    Impedance = "Impedance"
    FeedForward = "FeedForward"


class ControllerParamName(StrEnum):
    Kp = "Kp"
    Ki = "Ki"
    Kd = "Kd"
    Stiffness = "Stiffness"
    Damping = "Damping"
    Decay = "decay"


@dataclass
class GravityValue:
    """A solver gravity setting, authored either as literal coordinates or a spec reference."""

    coords: Coordinates | None = None
    unit: str = ""
    ref: ContextRef | None = None
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class UntilMonitorRef:
    """A monitor reference to a motion's whole `until` section."""

    motion: GuardedMotion
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return "until"

    def __str__(self) -> str:
        return f"{self.motion.name}.until"


@dataclass
class WhenMonitorRef:
    """A monitor reference to a motion's whole `when` section."""

    motion: GuardedMotion
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def motion_name(self) -> str:
        return self.motion.name

    @property
    def name(self) -> str:
        return "when"

    def __str__(self) -> str:
        return f"{self.motion.name}.when"


@dataclass
class SaturationSpec:
    parent: object
    maximum: ContextRef | None = None
    lower: ContextRef | None = None
    upper: ContextRef | None = None


@dataclass
class ConstraintHandler(IHasNamespaceDeclare):
    """Binds a motion to the controllers, monitors and solvers that realize it."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | SpecContextDecl | ContextDeclReference]
    motion: GuardedMotion
    solvers: list[SolverEntry | SolverAlias]
    monitors: list[MonitorEntry] = field(default_factory=list)
    controllers: list[ControllerEntry | ControllerAlias] = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, ns=self.ns, name=self.name)


@dataclass
class EventName:
    """A named FSM event referenced by a monitor."""

    parent: object
    # Qualified form: `ns.EventName`  →  event is the resolved fsm.Event object
    event: object = None
    ns: NamespaceDeclLike | None = None
    # Standalone form: bare name with no namespace prefix
    standalone: str = ""

    @property
    def name(self) -> str:
        """Bare event name string (works for both qualified and standalone forms)."""
        return self.event.name if self.event is not None else self.standalone

    @property
    def uri(self) -> str:
        if self.ns is not None and self.event is not None:
            # Qualified: use the locally-declared namespace URI (same as in the .fsm).
            return str(Namespace(self.ns.uri)[self.event.name])
        # Standalone (unqualified) event stays monitor-owned.
        return f"{self.parent.uri}.{self.standalone}"


@dataclass
class ROSTopic:
    """A ROS topic described with the terms from the ROS JSON-LD context."""

    channel_name: str
    type_name: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class MonitorEntry(NamedNamespaceObject):
    """A monitor watching a constraint and emitting an event when it triggers."""

    parent: object
    name: str
    constraint: ConstraintRef | UntilMonitorRef
    event: EventName | None = None
    fallback: GuardedMotion | None = None
    flag: str = ""
    ros_topic: ROSTopic | None = None
    # Optional `for <FLOAT> <Unit>` debounce clause: the monitored condition must
    # hold continuously for this long before the edge-triggered monitor fires.
    # Absent (None) == current byte-identical rising-edge behaviour.
    debounce: Measure | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def debounce_duration(self) -> float | None:
        if self.debounce is None:
            return None
        if self.debounce.unit not in ("s", "ms"):
            raise ValueError(
                f"Monitor '{self.name}' debounce unit '{self.debounce.unit}' must be 's' or 'ms'."
            )
        return float(self.debounce.value)

    @property
    def debounce_unit(self) -> str | None:
        """The unit the debounce was written in, which travels with its value."""
        return None if self.debounce is None else self.debounce.unit


@dataclass
class ControllerEntry(NamedNamespaceObject):
    """A controller (PID / impedance / feed-forward) driving a constraint."""

    parent: object
    name: str
    type: ControllerType
    params: ControllerParams
    solver: SolverRef | None = None
    command_type: QuantityType | None = None
    apply_at: object | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.type = _authored_enum(ControllerType, str(self.type))
        if self.command_type is not None and isinstance(self.command_type, str):
            self.command_type = _authored_enum(QuantityType, self.command_type)


@dataclass
class ControllerRef:
    """A reference to a controller by name."""

    handler: ConstraintHandler
    controller: ControllerEntry
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return self.controller.name

    def __str__(self) -> str:
        return f"{self.handler.name}.{self.controller.name}"


@dataclass(kw_only=True)
class ControllerAlias(ControllerEntry):
    """An alias referring to a ControllerEntry."""

    parent: object
    name: str
    ref: ControllerRef
    type: ControllerType = field(init=False)
    params: ControllerParams = field(init=False)
    solver: SolverRef | None = field(init=False, default=None)
    command_type: QuantityType | None = field(init=False, default=None)
    apply_at: object | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.controller.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.controller.uri
        self.type = self.ref.controller.type
        self.params = self.ref.controller.params
        self.solver = self.ref.controller.solver
        self.command_type = self.ref.controller.command_type
        self.apply_at = self.ref.controller.apply_at


@dataclass
class ControllerParams:
    """The resolved parameters of a controller (gains, constraint, profile, limits, ...)."""

    constraint: ConstraintRef
    profile: ContextRef | None = None
    measured_derivative: View | None = None
    output_saturation: SaturationSpec | None = None
    integral_saturation: SaturationSpec | None = None
    terms: list[object] = field(default_factory=list)
    kp: float | None = field(init=False, default=None)
    ki: float | None = field(init=False, default=None)
    kd: float | None = field(init=False, default=None)
    stiffness: float | None = field(init=False, default=None)
    damping: float | None = field(init=False, default=None)
    decay: float | None = field(init=False, default=None)
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        for term in self.terms:
            name = _authored_enum(ControllerParamName, str(term.name))
            if name == ControllerParamName.Kp:
                self.kp = term.value
            elif name == ControllerParamName.Ki:
                self.ki = term.value
            elif name == ControllerParamName.Kd:
                self.kd = term.value
            elif name == ControllerParamName.Stiffness:
                self.stiffness = term.value
            elif name == ControllerParamName.Damping:
                self.damping = term.value
            elif name == ControllerParamName.Decay:
                self.decay = term.value

@dataclass
class PIDControllerParams(ControllerParams):
    """Parameters admitted by the PID controller grammar alternative."""


@dataclass
class ImpedanceControllerParams(ControllerParams):
    """Parameters admitted by the impedance controller grammar alternative."""


@dataclass
class FeedForwardControllerParams(ControllerParams):
    """Parameters admitted by the feed-forward controller grammar alternative."""


@dataclass
class SolverLimits:
    parent: object
    entries: list[object] = field(default_factory=list)


@dataclass(eq=False)
class SerialChainSolver(NamedNamespaceObject):
    """A serial-chain dynamics solver (ACHD/RNE) over one ordered kinematic chain,
    assigned to an agent."""

    parent: object
    name: str
    agent: object
    # None on a monitor-only solver: no controller drives it, so no dynamics algorithm runs.
    algorithm: str | None = None
    limits: SolverLimits | None = None
    gravity_value: GravityValue | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.algorithm = {"achd": "ACHD", "rne": "RNE"}.get(self.algorithm, self.algorithm)


@dataclass(eq=False)
class MobilePlatformSolver(NamedNamespaceObject):
    """A mobile-platform solver over two independent axes: quantity {velocity, force} x
    operation {composition, distribution}. Composition aggregates drive/wheel quantities up
    to the platform; distribution allocates a platform quantity down to the drives.
    velocity-composition reconstructs the platform twist from measured wheel/drive
    velocities; velocity-distribution maps a desired platform twist to drive/wheel
    velocities; force-distribution (control allocation) maps a desired platform wrench to
    drive/wheel forces; force-composition aggregates measured drive/wheel (pivot) forces up
    to the platform wrench. All four are implemented by the hddc2b runtime."""

    parent: object
    name: str
    agent: object
    algorithm: str
    configuration: str
    quantity: ContextRef

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.algorithm = {
            "velocity-composition": "VelocityComposition",
            "velocity-distribution": "VelocityDistribution",
            "force-composition": "ForceComposition",
            "force-distribution": "ForceDistribution",
        }.get(self.algorithm, self.algorithm)


@dataclass(eq=False)
class CommandForwardingSolver(NamedNamespaceObject):
    """A pass-through solver forwarding a feed-forward controller's command directly to
    an agent's device; neither a dynamics solver nor a platform kinematics/control-allocation
    solver."""

    parent: object
    name: str
    agent: object
    algorithm: str = field(init=False, default="CommandForwarding")

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)


# The closed set of solver mechanisms a `solvers { }` group may authored: exactly one of a
# serial-chain dynamics solver, a mobile-platform velocity/force solver, or command forwarding.
SolverEntry = SerialChainSolver | MobilePlatformSolver | CommandForwardingSolver


@dataclass
class SolverRef:
    """A reference to a solver by name."""

    solver: SolverEntry
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return self.solver.name

    def __str__(self) -> str:
        return self.solver.name


@dataclass(eq=False)
class SolverAlias(NamedNamespaceObject):
    """An alias referring to a solver entry, whatever its mechanism."""

    parent: object
    name: str
    ref: SolverRef

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.solver.name
        super().__init__(parent=self.parent, name=self.name)
        self._uri = self.ref.solver.uri


def _resolved_controller(item: ControllerEntry | ControllerAlias) -> ControllerEntry:
    return item.ref.controller if isinstance(item, ControllerAlias) else item


def _resolved_solver(item: SolverEntry | SolverAlias) -> SolverEntry:
    return item.ref.solver if isinstance(item, SolverAlias) else item
