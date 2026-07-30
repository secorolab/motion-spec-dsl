# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Constraint-handler, monitor, controller, and solver model classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.motion import (
    ContextDeclReference,
    ContextRef,
    ConstraintRef,
    GuardedMotion,
    Measure,
    QuantityType,
    SpecContextDecl,
    View,
    WorldContextDecl,
    _authored_enum,
    _TIME_UNIT_SECONDS,
)
from motion_spec_dsl.classes.common import (
    IHasNamespaceDeclare,
    NamedNamespaceObject,
    NamespaceDeclLike,
)


class ControllerType(StrEnum):
    PID = "PID"
    Impedance = "Impedance"
    ABAG = "ABAG"
    FeedForward = "FeedForward"


class ControllerParamName(StrEnum):
    Kp = "Kp"
    Ki = "Ki"
    Kd = "Kd"
    Stiffness = "Stiffness"
    Damping = "Damping"
    Decay = "decay"


@dataclass
class GravityVector:
    """A concrete gravity vector expressed in the selected solver chain's root frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    unit: str = ""
    parent: object | None = field(default=None, repr=False, compare=False)


@dataclass
class GravityValue:
    """A solver gravity setting, authored either as a vector or a spec reference."""

    literal: GravityVector | None = None
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
class ProgressConstraint(NamedNamespaceObject):
    """A named advancement law: the rate and gating constraints that move a path
    parameter along one or more paths. This is the maintained path-tracking form.
    """

    parent: object
    name: str
    parameter: object  # ContextRef
    path: object | None = None  # ContextRef
    paths: list[object] = field(default_factory=list)  # ContextRef
    advancement: float = 0.0
    advancement_unit: str = "Hz"

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def path_refs(self) -> list[object]:
        """Return single- and multi-path syntax in one downstream representation."""
        return [self.path] if self.path is not None else self.paths


@dataclass
class ProgressObjective(NamedNamespaceObject):
    """A named handler policy asking a compatible solver to maximize a path parameter.

    No duration, easing, or fixed advancement rate: the solver owns the traversal.
    """

    parent: object
    name: str
    parameter: object  # ContextRef
    path: object | None = None  # ContextRef
    paths: list[object] = field(default_factory=list)  # ContextRef

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def path_refs(self) -> list[object]:
        """Return single- and multi-path syntax in one downstream representation."""
        return [self.path] if self.path is not None else self.paths


@dataclass
class ConstraintHandler(IHasNamespaceDeclare):
    """Binds a motion to the controllers, monitors and solvers that realize it."""

    parent: object
    ns: NamespaceDeclLike
    name: str
    context: list[WorldContextDecl | SpecContextDecl | ContextDeclReference]
    motion: GuardedMotion
    progress: list[ProgressConstraint | ProgressObjective]
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
        scale = _TIME_UNIT_SECONDS.get(self.debounce.unit)
        if scale is None:
            raise ValueError(
                f"Monitor '{self.name}' debounce unit '{self.debounce.unit}' must be 's' or 'ms'."
            )
        return self.debounce.value * scale


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
class ControllerParam:
    """A single controller parameter term."""

    name: ControllerParamName
    value: float
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        self.name = _authored_enum(ControllerParamName, str(self.name))


@dataclass
class ControllerParams:
    """The resolved parameters of a controller (gains, constraint, profile, limits, ...)."""

    constraint: ConstraintRef
    profile: ContextRef | None = None
    measured_derivative: View | None = None
    output_saturation: SaturationSpec | None = None
    integral_saturation: SaturationSpec | None = None
    terms: list[ControllerParam] = field(default_factory=list)
    kp: float | None = field(init=False, default=None)
    ki: float | None = field(init=False, default=None)
    kd: float | None = field(init=False, default=None)
    stiffness: float | None = field(init=False, default=None)
    damping: float | None = field(init=False, default=None)
    decay: float | None = field(init=False, default=None)
    duplicate_terms: list[str] = field(init=False, default_factory=list)
    parent: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        seen: set[str] = set()
        for term in self.terms:
            name = term.name
            if name in seen:
                self.duplicate_terms.append(name.value)
                continue
            seen.add(name)
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

    @property
    def pid_gains(self) -> tuple[float | None, float | None, float | None]:
        return (self.kp, self.ki, self.kd)

    @property
    def has_pid_gains(self) -> bool:
        return all(gain is not None for gain in self.pid_gains)

    @property
    def has_impedance_terms(self) -> bool:
        return self.stiffness is not None or self.damping is not None


@dataclass
class SolverLimitEntry:
    parent: object
    target: str
    saturation: SaturationSpec


@dataclass
class SolverLimits:
    parent: object
    entries: list[SolverLimitEntry] = field(default_factory=list)


@dataclass
class SolverEntry(NamedNamespaceObject):
    """A motion driver/solver (ACHD/RNE/...) assigned to an agent."""

    parent: object
    name: str
    agent: object
    algorithm: str
    limits: SolverLimits | None = None
    gravity_value: GravityValue | None = None

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)
        self.algorithm = {
            "command-forwarding": "CommandForwarding",
            "velocity-distribution": "VelocityDistribution",
            "force-distribution": "ForceDistribution",
        }.get(self.algorithm, self.algorithm)


@dataclass
class SolverRef:
    """A reference to a solver by name."""

    solver: SolverEntry
    handler: ConstraintHandler | None = field(default=None)
    parent: object | None = field(default=None, repr=False, compare=False)

    @property
    def name(self) -> str:
        return self.solver.name

    def __str__(self) -> str:
        if self.handler is not None:
            return f"{self.handler.name}.{self.solver.name}"
        return self.solver.name


@dataclass(kw_only=True)
class SolverAlias(SolverEntry):
    """An alias referring to a SolverEntry."""

    parent: object
    name: str
    ref: SolverRef
    agent: object = field(init=False)
    algorithm: str = field(init=False)
    limits: SolverLimits | None = field(init=False, default=None)
    gravity_value: GravityValue | None = field(init=False, default=None)

    def __post_init__(self):
        if not self.name:
            self.name = self.ref.solver.name
        NamedNamespaceObject.__init__(self, parent=self.parent, name=self.name)
        self._uri = self.ref.solver.uri
        self.agent = self.ref.solver.agent
        self.algorithm = self.ref.solver.algorithm
        self.limits = self.ref.solver.limits
        self.gravity_value = self.ref.solver.gravity_value


def _resolved_controller(item: ControllerEntry | ControllerAlias) -> ControllerEntry:
    return item.ref.controller if isinstance(item, ControllerAlias) else item


def _resolved_solver(item: SolverEntry | SolverAlias) -> SolverEntry:
    return item.ref.solver if isinstance(item, SolverAlias) else item
