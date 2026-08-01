# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""textX language construction and source-model reference resolution."""

from __future__ import annotations

from importlib.resources import files

from textx import metamodel_from_file
from textx.scoping import providers as scoping_providers

from motion_spec_dsl.classes.base import (
    Import,
    NamespaceDeclare,
)
from motion_spec_dsl.classes.constraint_handler import (
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerRef,
    FeedForwardControllerParams,
    ImpedanceControllerParams,
    PIDControllerParams,
    GravityValue,
    EventName,
    MonitorEntry,
    ROSTopic,
    SaturationSpec,
    SerialChainSolver,
    MobilePlatformSolver,
    CommandForwardingSolver,
    SolverAlias,
    SolverLimits,
    SolverRef,
    UntilMonitorRef,
    WhenMonitorRef,
    _resolved_controller,
    _resolved_solver,
)
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    OutsideConstraint,
    ConstraintAlias,
    ConstraintSpecification,
    ConstraintRef,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    ConstraintGroup,
    _resolved_spec,
)
from motion_spec_dsl.classes.context import (
    ContextRef,
    ElapsedTime,
    ProgressAlong,
    MovingAlong,
    OnPath,
    GeoPropPair,
    GeometricProps,
    ReferenceValue,
    Measure,
    SelectorTail,
    SnapshotValue,
    ContextQuantityAlias,
    ContextQuantity,
    ContextPath,
    VectorXYZ,
    View,
    WorldQuantityAlias,
    WorldQuantity,
)
from motion_spec_dsl.classes.coordinates import (
    Coordinates,
    CoordinateElement,
    DirectionCosineXYZ,
    EulerAngles,
    AccelerationTwistCoordinate,
    OrientationCoordinate,
    RelativeOrientation,
    PoseCoordinate,
    PositionCoordinate,
    Quaternion,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.motion_spec import (
    ContextDeclReference,
    ContextSpec,
    ExecutionContext,
    Model,
    GuardedMotion,
    PostContextDecl,
    PreContextDecl,
    SpecContextDecl,
    WorldContextDecl,
    WhenSection,
    WhileSection,
    UntilSection,
)
from motion_spec_dsl.classes.path import (
    Figure8Spec,
    LerpSpec,
    ProfileSpec,
    AdmittanceSpec,
    PathValue,
)
from motion_spec_dsl.classes.scoping import SceneRefProvider, finalize_imported_scenes
from motion_spec_dsl.classes.validation import motion_constraint_items, validate_model


GRAMMAR_PATH = str(files("motion_spec_dsl.grammars").joinpath("model.tx"))


LANGUAGE_CLASSES = [
    Model,
    NamespaceDeclare,
    Import,
    ExecutionContext,
    ContextSpec,
    Coordinates,
    CoordinateElement,
    PositionCoordinate,
    OrientationCoordinate,
    RelativeOrientation,
    EulerAngles,
    Quaternion,
    DirectionCosineXYZ,
    PoseCoordinate,
    VelocityTwistCoordinate,
    AccelerationTwistCoordinate,
    WrenchCoordinate,
    GuardedMotion,
    PathValue,
    LerpSpec,
    ProfileSpec,
    AdmittanceSpec,
    Figure8Spec,
    ConstraintHandler,
    WorldContextDecl,
    PreContextDecl,
    SpecContextDecl,
    PostContextDecl,
    ContextDeclReference,
    WorldQuantity,
    WorldQuantityAlias,
    GeometricProps,
    GeoPropPair,
    GravityValue,
    ContextQuantity,
    ContextQuantityAlias,
    ContextPath,
    Measure,
    VectorXYZ,
    ReferenceValue,
    SnapshotValue,
    ConstraintAlias,
    ConstraintGroup,
    ConstraintSpecification,
    ConstraintRef,
    UntilMonitorRef,
    WhenMonitorRef,
    ContextRef,
    SaturationSpec,
    View,
    ElapsedTime,
    ProgressAlong,
    MovingAlong,
    OnPath,
    SelectorTail,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    OutsideConstraint,
    MonitorEntry,
    ROSTopic,
    EventName,
    ControllerAlias,
    ControllerEntry,
    ControllerRef,
    FeedForwardControllerParams,
    ImpedanceControllerParams,
    PIDControllerParams,
    SerialChainSolver,
    MobilePlatformSolver,
    CommandForwardingSolver,
    SolverAlias,
    SolverLimits,
    SolverRef,
    WhenSection,
    WhileSection,
    UntilSection,
]


class MotionConstraintScopeProvider:
    """Resolve the constraint part of refs authored as motion.constraint."""

    def __call__(self, obj: ConstraintRef, attr, obj_ref):
        """Resolve `obj_ref` to a constraint declared in the ref's motion."""
        del attr
        motion = obj.motion
        if motion is None or not isinstance(motion, GuardedMotion):
            return None

        # An until group is monitored as one condition, so it is nameable like a constraint.
        for section in motion.sections:
            for item in section.constraints:
                if isinstance(item, ConstraintGroup) and item.name == obj_ref.obj_name:
                    return item

        for item in motion_constraint_items(motion):
            item_name = getattr(item, "name", None) or getattr(_resolved_spec(item), "name", None)
            if item_name == obj_ref.obj_name:
                return _resolved_spec(item)
        return None


class HandlerControllerScopeProvider:
    """Resolve controller refs against controllers declared in the target handler."""

    def __call__(self, obj: ControllerRef, attr, obj_ref):
        """Resolve `obj_ref` to a controller declared in the ref's handler."""
        del attr
        handler = obj.handler
        if handler is None or not isinstance(handler, ConstraintHandler):
            return None
        for controller in getattr(handler, "controllers", []):
            if controller.name == obj_ref.obj_name:
                return _resolved_controller(controller)
        return None


class CrossHandlerSolverScopeProvider:
    """Resolve solver refs: cross-handler when handler is set, else local handler via parent chain."""

    def __call__(self, obj: SolverRef, attr, obj_ref):
        """Resolve `obj_ref` to a solver in the target (or ancestor) handler."""
        del attr
        handler = obj.handler
        if not isinstance(handler, ConstraintHandler):
            current = getattr(obj, "parent", None)
            while current is not None and not isinstance(current, ConstraintHandler):
                current = getattr(current, "parent", None)
            handler = current
        if not isinstance(handler, ConstraintHandler):
            return None
        for solver in getattr(handler, "solvers", []):
            solver_name = getattr(solver, "name", None) or getattr(
                _resolved_solver(solver), "name", None
            )
            if solver_name == obj_ref.obj_name:
                return _resolved_solver(solver)
        return None


def motion_spec_metamodel():
    """Build the textx metamodel for the motion_spec DSL with its scope providers and
    model-validation processor.
    """
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers(
        {
            "*.*": SceneRefProvider(),
            # FSM events are flat (not FQN-nested) — resolve by plain name across
            # all models loaded via importURI (including any imported .fsm).
            "EventName.event": scoping_providers.PlainNameImportURI(),
            "ConstraintRef.constraint": MotionConstraintScopeProvider(),
            "ControllerRef.controller": HandlerControllerScopeProvider(),
            "SolverRef.solver": CrossHandlerSolverScopeProvider(),
        }
    )
    # Fill imported scene instance trees before anything walks scene objects.
    metamodel.register_model_processor(finalize_imported_scenes)
    metamodel.register_model_processor(validate_model)
    return metamodel
