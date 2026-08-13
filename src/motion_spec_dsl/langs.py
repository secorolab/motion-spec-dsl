# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""textX language construction and source-model reference resolution."""

from __future__ import annotations

from importlib.resources import files

from textx import metamodel_from_file

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
    StateName,
    MonitorAction,
    MonitorEntry,
    MonitorStateBlock,
    RosTopicDecl,
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
)
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    OutsideConstraint,
    ConstraintAlias,
    ConstraintSpecification,
    ConstraintRef,
    EqualityConstraint,
    GoalStatusConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    ConstraintGroup,
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
    DetectDecl,
    ExecutionContext,
    RosActionDecl,
    Model,
    GuardedMotion,
    PostContextDecl,
    PreContextDecl,
    SpecContextDecl,
    SceneObjRef,
    ToleranceDefault,
    ToleranceDefaults,
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
from motion_spec_dsl.classes.ros import (
    Ros,
    RosActionServerDecl,
    RosMeasurementAssign,
    RosResult,
    RosStandingPub,
    RosSubscriptionDecl,
)
from motion_spec_dsl.classes.scoping import SceneRefProvider, finalize_imported_scenes
from motion_spec_dsl.classes.validation import validate_model


GRAMMAR_PATH = str(files("motion_spec_dsl.grammars").joinpath("model.tx"))


LANGUAGE_CLASSES = [
    Model,
    NamespaceDeclare,
    Import,
    ExecutionContext,
    ContextSpec,
    ToleranceDefaults,
    ToleranceDefault,
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
    MonitorStateBlock,
    MonitorAction,
    Ros,
    RosTopicDecl,
    RosSubscriptionDecl,
    RosActionDecl,
    RosActionServerDecl,
    RosResult,
    RosStandingPub,
    RosMeasurementAssign,
    DetectDecl,
    SceneObjRef,
    GoalStatusConstraint,
    EventName,
    StateName,
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


def motion_spec_metamodel():
    """Build the textx metamodel for the motion_spec DSL with its scope providers and
    model-validation processor.
    """
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers(
        {
            "*.*": SceneRefProvider(),
            "ControllerRef.controller": HandlerControllerScopeProvider(),
        }
    )
    # Fill imported scene instance trees before anything walks scene objects.
    metamodel.register_model_processor(finalize_imported_scenes)
    metamodel.register_model_processor(validate_model)
    return metamodel
