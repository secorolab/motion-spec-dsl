# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Constraint-handler, monitor, controller, and solver model classes."""

from __future__ import annotations

from enum import StrEnum

from rdflib.namespace import Namespace

from motion_spec_dsl.classes.common import IHasNamespaceDeclare, NamedNamespaceObject
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    ConstraintAlias,
    ConstraintSpecification,
    EqualityConstraint,
    GoalStatusConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
    flatten_constraint_items,
)
from motion_spec_dsl.classes.context import (
    ContextQuantity,
    ContextQuantityAlias,
    QuantityType,
    SnapshotValue,
    WorldQuantity,
    WorldQuantityAlias,
    authored_enum,
)
from motion_spec_dsl.classes.coordinates import const_value
from motion_spec_dsl.classes.motion_spec import (
    ContextDeclReference,
    ContextSpec,
    GuardedMotion,
    Model,
)
from motion_spec_dsl.classes.path import AdmittanceSpec, ProfileSpec


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


class GravityValue:
    """A solver's gravity, as literal coordinates or a reference to a spec quantity."""

    def __init__(self, parent, coords, unit, ref) -> None:
        self.parent = parent
        self.coords = coords
        self.unit = unit
        self.ref = ref


class UntilMonitorRef:
    """A monitor target naming a motion's whole `until` section."""

    name = "until"

    def __init__(self, parent, motion) -> None:
        self.parent = parent
        self.motion = motion

    def __str__(self) -> str:
        return f"{self.motion.name}.until"


class WhenMonitorRef:
    """A monitor target naming a motion's whole `when` section."""

    name = "when"

    def __init__(self, parent, motion) -> None:
        self.parent = parent
        self.motion = motion

    def __str__(self) -> str:
        return f"{self.motion.name}.when"


class SaturationSpec:
    def __init__(self, parent, maximum, lower, upper) -> None:
        self.parent = parent
        self.maximum = maximum
        self.lower = lower
        self.upper = upper


class ConstraintHandler(IHasNamespaceDeclare):
    """Binds a motion to the controllers, monitors and solvers that realize it."""

    def __init__(
        self,
        parent,
        ns,
        name,
        context,
        motion,
        runs_in,
        monitors,
        controllers,
        solvers,
        perturbations,
    ) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.context = context
        self.motion = motion
        self.runs_in = runs_in
        self.monitors = monitors
        self.controllers = controllers
        self.solvers = solvers
        self.perturbations = perturbations


class PerturbationEntry(NamedNamespaceObject):
    """A wrench the simulator applies to a body while the handler's state is active.

    Its window opens once per activation when the `gate` holds (on entry without one) and
    closes after `duration` (on state exit without one).
    """

    def __init__(
        self,
        parent,
        name,
        body,
        force,
        force_direction,
        moment,
        moment_direction,
        gate_logic,
        gate,
        duration,
    ) -> None:
        super().__init__(parent=parent, name=name)
        self.body = body
        self.force = force
        self.force_direction = force_direction
        self.moment = moment
        self.moment_direction = moment_direction
        self.gate_logic = gate_logic or None
        self.gate = gate
        self.duration = duration


class StateName:
    """The FSM state a handler stays active in, for a motion meant to keep running."""

    def __init__(self, parent, ns, state) -> None:
        self.parent = parent
        self.ns = ns
        self.state = state
        self.name = state.name
        self.uri = str(Namespace(ns.uri)[state.name])


class EventName:
    """An FSM event, written `<ns.Event>`, or a bare name the monitor owns."""

    def __init__(self, parent, ns, event, standalone) -> None:
        self.parent = parent
        self.ns = ns
        self.event = event
        self.standalone = standalone
        self.name = event.name if event is not None else standalone

    @property
    def uri(self) -> str:
        if self.ns is not None and self.event is not None:
            return str(Namespace(self.ns.uri)[self.event.name])
        return f"{self.parent.uri}.{self.standalone}"


class RosTopicDecl(NamedNamespaceObject):
    """A ROS topic: the channel it is published on and the message it carries."""

    def __init__(self, parent, name, channel_name, type_name) -> None:
        super().__init__(parent=parent, name=name)
        self.channel_name = channel_name
        self.type_name = type_name


class MonitorAction:
    """One action a monitor performs while in a state: trigger, hold, flag, publish or result."""

    def __init__(
        self, parent, event, fallback, flag, events, topic, fields, rate, value, outcome, server
    ) -> None:
        self.parent = parent
        self.event = event
        self.fallback = fallback
        self.flag = flag
        # `publish: events {..}`: one message per event, carrying its IRI.
        self.events = events
        self.topic = topic
        self.fields = fields
        # Unstated, a verdict goes out every cycle the motion is active.
        self.rate = rate
        # The sugar form's one value, whose field the message type resolves.
        self.value = value
        self.outcome = outcome
        self.server = server
        if event is not None:
            self.kind = "trigger"
        elif fallback is not None:
            self.kind = "hold"
        elif server is not None:
            self.kind = "result"
        elif topic is not None:
            self.kind = "publish"
        else:
            self.kind = "flag"
        # (dot-path, authored text) rows; the sugar form leaves the path to the message type.
        written = [((), value)] if value is not None else [(item.path, item.value) for item in fields]
        self.assignments = [
            (".".join(path), field_value.constant or str(field_value.literal))
            for path, field_value in written
        ]


class MonitorStateBlock:
    """The actions a monitor performs while satisfied or violated."""

    def __init__(self, parent, state, sustain, actions) -> None:
        self.parent = parent
        self.state = state
        self.sustain = sustain
        self.actions = actions


class MonitorEntry(NamedNamespaceObject):
    """A monitor watching a constraint, acting per state; validation allows one action per kind."""

    def __init__(self, parent, name, constraint, states) -> None:
        super().__init__(parent=parent, name=name)
        self.constraint = constraint
        self.states = states
        self.actions_by_kind: dict[str, list] = {}
        for block in states:
            for action in block.actions:
                self.actions_by_kind.setdefault(action.kind, []).append((block, action))
        trigger = next(iter(self.actions_by_kind.get("trigger", [])), None)
        self.trigger = None if trigger is None else (trigger[0].state, trigger[1].event)
        self.event = None if trigger is None else trigger[1].event
        # The sustain the triggering state is entered under.
        self.debounce = None if trigger is None else trigger[0].sustain
        self.debounce_duration = None if self.debounce is None else float(self.debounce.value)
        self.debounce_unit = None if self.debounce is None else self.debounce.unit
        flag = next(iter(self.actions_by_kind.get("flag", [])), None)
        self.flag = "" if flag is None else flag[1].flag
        hold = next(iter(self.actions_by_kind.get("hold", [])), None)
        self.fallback = None if hold is None else hold[1].fallback
        # Reaching the answering state is what the model calls the goal finished.
        self.answer = next(iter(self.actions_by_kind.get("result", [])), None)
        self.published = self.actions_by_kind.get("publish", [])
        self.topic = self.published[0][1].topic if self.published else None
        announcing = [action for _block, action in self.published if action.events]
        self.occurrence_topic = announcing[0].topic if announcing else None
        self.announced_events = announcing[0].events if announcing else []


class ControllerEntry(NamedNamespaceObject):
    """A PID, impedance or feed-forward controller driving a constraint."""

    def __init__(self, parent, type, name, params, command_type, apply_at, solver) -> None:
        super().__init__(parent=parent, name=name)
        self.type = authored_enum(ControllerType, str(type))
        self.params = params
        self.command_type = authored_enum(QuantityType, command_type) if command_type else None
        self.apply_at = apply_at
        self.solver = solver


class ControllerRef:
    """A controller named through the handler that declares it."""

    def __init__(self, parent, handler, controller) -> None:
        self.parent = parent
        self.handler = handler
        self.controller = controller
        self.name = controller.name

    def __str__(self) -> str:
        return f"{self.handler.name}.{self.controller.name}"


class ControllerAlias(ControllerEntry):
    """A local name for a controller declared in another handler."""

    def __init__(self, parent, name, ref) -> None:
        controller = ref.controller
        NamedNamespaceObject.__init__(self, parent=parent, name=name or controller.name)
        self.ref = ref
        self._uri = controller.uri
        # The controller may still hold its raw grammar values: textX initializes it later.
        self.type = authored_enum(ControllerType, str(controller.type))
        self.params = controller.params
        command_type = controller.command_type
        self.command_type = authored_enum(QuantityType, str(command_type)) if command_type else None
        self.apply_at = controller.apply_at
        self.solver = controller.solver


def controller_gains(terms) -> dict[ControllerParamName, float]:
    """The gains a controller's authored terms state, by name."""
    return {
        authored_enum(ControllerParamName, str(term.name)): const_value(term.value)
        for term in terms
    }


class PIDControllerParams:
    """A PID's constraint, gains, optional profile, measured derivative and limits."""

    def __init__(
        self,
        parent,
        constraint,
        profile,
        measured_derivative,
        output_saturation,
        integral_saturation,
        terms,
        error_normalization,
    ) -> None:
        self.parent = parent
        self.constraint = constraint
        self.profile = profile
        self.measured_derivative = measured_derivative
        self.output_saturation = output_saturation
        self.integral_saturation = integral_saturation
        # The turn a circular quantity's error is read onto.
        self.error_normalization = error_normalization
        self.terms = terms
        gains = controller_gains(terms)
        self.kp = gains.get(ControllerParamName.Kp)
        self.ki = gains.get(ControllerParamName.Ki)
        self.kd = gains.get(ControllerParamName.Kd)
        self.decay = gains.get(ControllerParamName.Decay)
        self.stiffness = None
        self.damping = None


class ImpedanceControllerParams:
    """An impedance controller's constraint, stiffness, damping and optional integral gain."""

    def __init__(self, parent, constraint, output_saturation, terms) -> None:
        self.parent = parent
        self.constraint = constraint
        self.output_saturation = output_saturation
        self.terms = terms
        self.profile = None
        self.measured_derivative = None
        self.integral_saturation = None
        self.error_normalization = None
        gains = controller_gains(terms)
        self.ki = gains.get(ControllerParamName.Ki)
        self.stiffness = gains.get(ControllerParamName.Stiffness)
        self.damping = gains.get(ControllerParamName.Damping)
        self.kp = None
        self.kd = None
        self.decay = None


class FeedForwardControllerParams:
    """A feed-forward controller's constraint and optional output saturation."""

    def __init__(self, parent, constraint, output_saturation) -> None:
        self.parent = parent
        self.constraint = constraint
        self.output_saturation = output_saturation
        self.terms = []
        self.profile = None
        self.measured_derivative = None
        self.integral_saturation = None
        self.error_normalization = None
        self.kp = None
        self.ki = None
        self.kd = None
        self.decay = None
        self.stiffness = None
        self.damping = None


ControllerParams = PIDControllerParams | ImpedanceControllerParams | FeedForwardControllerParams


class SolverLimits:
    def __init__(self, parent, entries) -> None:
        self.parent = parent
        self.entries = entries


SERIAL_CHAIN_ALGORITHMS = {"achd": "ACHD", "rne": "RNE", "fpk": "FPK", "fvk": "FVK"}
KINEMATICS_ALGORITHMS = {"FPK", "FVK"}
MOBILE_PLATFORM_ALGORITHMS = {
    "velocity-composition": "VelocityComposition",
    "velocity-distribution": "VelocityDistribution",
    "force-composition": "ForceComposition",
    "force-distribution": "ForceDistribution",
}


class SerialChainSolver(NamedNamespaceObject):
    """A solver over one kinematic chain: dynamics (ACHD, RNE) or forward kinematics (FPK, FVK)."""

    def __init__(self, parent, name, agent, algorithm, limits, gravity_value) -> None:
        super().__init__(parent=parent, name=name)
        self.agent = agent
        self.algorithm = SERIAL_CHAIN_ALGORITHMS[algorithm]
        self.limits = limits
        self.gravity_value = gravity_value


class MobilePlatformSolver(NamedNamespaceObject):
    """A platform solver: velocity or force, composed up from the drives or distributed down."""

    def __init__(self, parent, name, agent, algorithm, configuration, quantity) -> None:
        super().__init__(parent=parent, name=name)
        self.agent = agent
        self.algorithm = MOBILE_PLATFORM_ALGORITHMS[algorithm]
        self.configuration = configuration
        self.quantity = quantity


class CommandForwardingSolver(NamedNamespaceObject):
    """Passes a feed-forward controller's command straight to an agent's device."""

    algorithm = "CommandForwarding"

    def __init__(self, parent, name, agent) -> None:
        super().__init__(parent=parent, name=name)
        self.agent = agent


SolverEntry = SerialChainSolver | MobilePlatformSolver | CommandForwardingSolver


class SolverRef:
    """A solver named through any aliases."""

    def __init__(self, parent, solver) -> None:
        self.parent = parent
        self.solver = solver
        self.name = solver.name

    def __str__(self) -> str:
        return self.solver.name


class SolverAlias(NamedNamespaceObject):
    """A local name for a solver declared in another handler."""

    def __init__(self, parent, name, ref) -> None:
        super().__init__(parent=parent, name=name or ref.solver.name)
        self.ref = ref
        self._uri = ref.solver.uri


def resolved_constraint_items(motion: GuardedMotion) -> list[ConstraintSpecification]:
    """The enabled constraints of a motion's sections, aliases resolved, goal statuses left out."""
    out = []
    for section in (motion.when, motion.while_, motion.until):
        for item in flatten_constraint_items(section.constraints):
            spec = item.constraint if isinstance(item, ConstraintAlias) else item
            if not isinstance(spec, GoalStatusConstraint) and not spec.disabled:
                out.append(spec)
    return out


def perturbation_conditions(handler: ConstraintHandler) -> list[ConstraintSpecification]:
    """The enabled constraints the handler's perturbation gates hold, aliases resolved."""
    out = []
    for perturbation in handler.perturbations:
        for item in flatten_constraint_items(perturbation.gate):
            spec = item.constraint if isinstance(item, ConstraintAlias) else item
            if not isinstance(spec, GoalStatusConstraint) and not spec.disabled:
                out.append(spec)
    return out


def motion_context_quantities(
    models, motion: GuardedMotion, handler: ConstraintHandler
) -> dict[str, ContextQuantity]:
    """Every context quantity in scope for a motion; its own declarations win."""
    shared = [
        ctx
        for model in models
        if isinstance(model, Model)
        for spec in model.specs
        if isinstance(spec, ContextSpec)
        for ctx in spec.context
    ]
    own = [ctx.ref if isinstance(ctx, ContextDeclReference) else ctx for ctx in motion.context]
    handled = [ctx.ref if isinstance(ctx, ContextDeclReference) else ctx for ctx in handler.context]
    refs = []
    for constraint in resolved_constraint_items(motion) + perturbation_conditions(handler):
        expr = constraint.expr
        if isinstance(expr, EqualityConstraint):
            refs.append(expr.reference)
        elif isinstance(expr, (GreaterThanConstraint, LessThanConstraint)):
            refs.append(expr.threshold)
        elif isinstance(expr, (BilateralConstraint, OutsideConstraint)):
            refs.extend((expr.lower, expr.upper))
    for solver in handler.solvers:
        solver = solver.ref.solver if isinstance(solver, SolverAlias) else solver
        if isinstance(solver, SerialChainSolver) and solver.gravity_value is not None:
            refs.append(solver.gravity_value.ref)
    profiles = []
    for controller in handler.controllers:
        controller = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
        profiles.append(controller.params.profile)
    # (quantity, whether it replaces an earlier one): shared ones are resolved, as were profiles.
    candidates = [
        (item.ref if isinstance(item, ContextQuantityAlias) else item, False)
        for ctx in shared
        if ctx.name != "world"
        for item in ctx.declaration
    ]
    candidates += [(item, True) for ctx in own if ctx.name != "world" for item in ctx.declaration]
    candidates += [(item, False) for ctx in handled if ctx.name == "spec" for item in ctx.declaration]
    candidates += [(ref.quantity, False) for ref in refs if ref is not None]
    candidates += [
        (ref.quantity.ref if isinstance(ref.quantity, ContextQuantityAlias) else ref.quantity, False)
        for ref in profiles
        if ref is not None
    ]
    quantities: dict[str, ContextQuantity] = {}
    for quantity, replaces in candidates:
        if not isinstance(quantity, ContextQuantity):
            continue
        if replaces:
            quantities[quantity.name] = quantity
        else:
            quantities.setdefault(quantity.name, quantity)
    return quantities


def motion_world_quantities(
    models,
    motion: GuardedMotion,
    handler: ConstraintHandler,
    context_quantities: dict[str, ContextQuantity],
) -> dict[str, WorldQuantity]:
    """Every world quantity in scope for a motion; its own declarations win."""
    shared = [
        ctx
        for model in models
        if isinstance(model, Model)
        for spec in model.specs
        if isinstance(spec, ContextSpec)
        for ctx in spec.context
    ]
    own = [ctx.ref if isinstance(ctx, ContextDeclReference) else ctx for ctx in motion.context]
    handled = [ctx.ref if isinstance(ctx, ContextDeclReference) else ctx for ctx in handler.context]
    views = [
        constraint.view
        for constraint in resolved_constraint_items(motion) + perturbation_conditions(handler)
    ]
    for quantity in context_quantities.values():
        if isinstance(quantity.value, SnapshotValue):
            views.append(quantity.value.source)
        elif isinstance(quantity.value, ProfileSpec):
            views.append(quantity.value.measured_velocity)
        elif isinstance(quantity.value, AdmittanceSpec):
            views.append(quantity.value.force)
    for controller in handler.controllers:
        controller = controller.ref.controller if isinstance(controller, ControllerAlias) else controller
        views.append(controller.params.measured_derivative)
    candidates = [(item, False) for ctx in shared if ctx.name == "world" for item in ctx.declaration]
    candidates += [(item, True) for ctx in own if ctx.name == "world" for item in ctx.declaration]
    candidates += [(item, False) for ctx in handled if ctx.name == "world" for item in ctx.declaration]
    for view in views:
        if view is None:
            continue
        candidates.append((view.quantity, False))
        if view.binary is not None:
            candidates += [(view.binary.left, False), (view.binary.right, False)]
    qtys: dict[str, WorldQuantity] = {}
    for item, replaces in candidates:
        if not isinstance(item, WorldQuantity):
            continue
        quantity = item.ref if isinstance(item, WorldQuantityAlias) else item
        if replaces:
            qtys[quantity.name] = quantity
        else:
            qtys.setdefault(quantity.name, quantity)
    return qtys
