# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""textX language construction and source-model reference resolution."""

from __future__ import annotations

from importlib.resources import files

from scene_dsl.langs import build_instance_trees, lower_frame_refs
from textx import metamodel_from_file

from motion_spec_dsl.classes.base import Import, NamespaceDeclare
from motion_spec_dsl.classes.constraint_handler import (
    CommandForwardingSolver,
    ConstraintHandler,
    ControllerAlias,
    ControllerEntry,
    ControllerRef,
    EventName,
    FeedForwardControllerParams,
    GravityValue,
    ImpedanceControllerParams,
    MobilePlatformSolver,
    MonitorAction,
    MonitorEntry,
    MonitorStateBlock,
    PerturbationEntry,
    PIDControllerParams,
    RosTopicDecl,
    SaturationSpec,
    SerialChainSolver,
    SolverAlias,
    SolverLimits,
    SolverRef,
    StateName,
    UntilMonitorRef,
    WhenMonitorRef,
)
from motion_spec_dsl.classes.constraints import (
    BilateralConstraint,
    ConstraintAlias,
    ConstraintGroup,
    ConstraintRef,
    ConstraintSpecification,
    EqualityConstraint,
    GoalStatusConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    OutsideConstraint,
)
from motion_spec_dsl.classes.context import (
    AngleBetweenView,
    ConfigValue,
    ContextPath,
    ContextQuantity,
    ContextQuantityAlias,
    ContextRef,
    DerivedScalarValue,
    DirectionBetween,
    DistanceBetweenView,
    DistanceFromView,
    ElapsedTime,
    GeometricProps,
    GeoPropPair,
    Measure,
    MovingAlong,
    NormView,
    ObserverSpec,
    OnPath,
    ProgressAlong,
    ProjectionOnView,
    QAddTail,
    QExpr,
    QFactor,
    QMulTail,
    QTerm,
    QuantityLeaf,
    ReferenceValue,
    SampledValue,
    SelectorTail,
    SnapshotValue,
    VectorXYZ,
    View,
    WorldQuantity,
    WorldQuantityAlias,
)
from motion_spec_dsl.classes.coordinates import (
    AccelerationTwistCoordinate,
    ConstAtom,
    ConstFactor,
    CoordinateElement,
    Coordinates,
    DirectionCosineXYZ,
    EulerAngles,
    OrientationCoordinate,
    PoseCoordinate,
    PositionCoordinate,
    Quaternion,
    RelativeOrientation,
    VelocityTwistCoordinate,
    WrenchCoordinate,
)
from motion_spec_dsl.classes.motion_spec import (
    ConstraintSection,
    ContextDeclReference,
    ContextSpec,
    DetectDecl,
    ExecutionContext,
    GuardedMotion,
    Model,
    QuantityContextDecl,
    RosActionDecl,
    SceneObjRef,
    ToleranceDefault,
    ToleranceDefaults,
)
from motion_spec_dsl.classes.path import (
    AdmittanceSpec,
    ArcSpec,
    CircleSpec,
    Figure8Spec,
    HelixSpec,
    LerpSpec,
    PathValue,
    ProfileSpec,
)
from motion_spec_dsl.classes.ros import (
    CameraTargetRef,
    Ros,
    RosActionServerDecl,
    RosGroup,
    RosMeasurementAssign,
    RosStandingEntry,
    RosStandingPub,
    RosSubscriptionDecl,
    WorldQuantityRef,
)
from motion_spec_dsl.classes.scoping import SceneRefProvider
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
    ConstFactor,
    ConstAtom,
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
    CircleSpec,
    ArcSpec,
    HelixSpec,
    Figure8Spec,
    ProfileSpec,
    AdmittanceSpec,
    ConstraintHandler,
    QuantityContextDecl,
    ContextDeclReference,
    WorldQuantity,
    WorldQuantityAlias,
    GeometricProps,
    GeoPropPair,
    ObserverSpec,
    GravityValue,
    ContextQuantity,
    ContextQuantityAlias,
    ContextPath,
    Measure,
    SampledValue,
    VectorXYZ,
    QExpr,
    QAddTail,
    QTerm,
    QMulTail,
    QFactor,
    QuantityLeaf,
    ReferenceValue,
    SnapshotValue,
    ConfigValue,
    DirectionBetween,
    DerivedScalarValue,
    DistanceBetweenView,
    AngleBetweenView,
    DistanceFromView,
    ProjectionOnView,
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
    NormView,
    SelectorTail,
    EqualityConstraint,
    GreaterThanConstraint,
    LessThanConstraint,
    BilateralConstraint,
    OutsideConstraint,
    MonitorEntry,
    MonitorStateBlock,
    MonitorAction,
    PerturbationEntry,
    Ros,
    RosGroup,
    RosTopicDecl,
    RosSubscriptionDecl,
    WorldQuantityRef,
    CameraTargetRef,
    RosActionDecl,
    RosActionServerDecl,
    RosStandingPub,
    RosStandingEntry,
    RosMeasurementAssign,
    DetectDecl,
    SceneObjRef,
    GoalStatusConstraint,
    EventName,
    StateName,
    ControllerAlias,
    ControllerEntry,
    ControllerRef,
    PIDControllerParams,
    ImpedanceControllerParams,
    FeedForwardControllerParams,
    SerialChainSolver,
    MobilePlatformSolver,
    CommandForwardingSolver,
    SolverAlias,
    SolverLimits,
    SolverRef,
    ConstraintSection,
]


class HandlerControllerScopeProvider:
    """Resolve a controller ref against the controllers of the handler it names."""

    def __call__(self, obj: ControllerRef, attr, obj_ref):
        del attr
        if not isinstance(obj.handler, ConstraintHandler):
            return None
        for controller in obj.handler.controllers:
            if controller.name == obj_ref.obj_name:
                return controller.ref.controller if isinstance(controller, ControllerAlias) else controller
        return None


def motion_spec_metamodel():
    """The textX metamodel for the motion-spec DSL, with its scope providers and validation."""
    metamodel = metamodel_from_file(GRAMMAR_PATH, autokwd=True, classes=LANGUAGE_CLASSES)
    metamodel.register_scope_providers(
        {
            "*.*": SceneRefProvider(),
            "ControllerRef.controller": HandlerControllerScopeProvider(),
        }
    )
    # Fill imported scene instance trees before anything walks scene objects.
    metamodel.register_model_processor(build_instance_trees)
    metamodel.register_model_processor(lower_frame_refs)
    metamodel.register_model_processor(validate_model)
    return metamodel
