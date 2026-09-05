# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to geometric-path and reference-generator grammar rules."""

from __future__ import annotations

from motion_spec_dsl.classes.coordinates import const_value
from dataclasses import dataclass


@dataclass
class LerpSpec:
    parent: object
    start: object  # ContextRef
    goal: object  # ContextRef


@dataclass
class CircleSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    center: object  # ContextRef (Position the curve orbits; radius = |start - center| in-plane)
    plane_normal: object  # ContextRef


@dataclass
class ArcSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    end: object  # ContextRef (Pose: the other endpoint and orientation target)
    amplitude: object  # ContextRef (LinearDistance: how far the arc bows from the chord; = chord/2 -> semicircle)
    plane_normal: object  # ContextRef


@dataclass
class HelixSpec:
    parent: object
    start: (
        object  # ContextRef (Pose on the curve: position -> start point, rotation -> orientation)
    )
    center: (
        object  # ContextRef (Position the helix winds around; radius = |start - center| in-plane)
    )
    axis: object  # ContextRef
    pitch: object  # ContextRef
    revolutions: object  # ContextRef


@dataclass
class Figure8Spec:
    parent: object
    anchor: object  # ContextRef (Pose: position -> center, rotation -> in-plane lobe axis)
    radius: object  # ContextRef
    plane_normal: object  # ContextRef
    form: str = "gerono"


@dataclass
class PathValue:
    parent: object
    lerp: LerpSpec | None = None
    circle: CircleSpec | None = None
    arc: ArcSpec | None = None
    helix: HelixSpec | None = None
    figure8: Figure8Spec | None = None


@dataclass
class ProfileSpec:
    """Authored limits and shape for an online velocity profile."""

    parent: object
    max_velocity: object
    max_acceleration: object
    measured_velocity: object | None = None
    max_jerk: object | None = None
    shape: str = "trapezoidal"


@dataclass
class AdmittanceSpec:
    parent: object
    force: object  # View onto a measured-Wrench force axis (force-in)
    mass: float
    damping: float
    stiffness: float
    max_velocity: float
    max_velocity_unit: object | None = None
    max_excursion: float = 0.0
    max_excursion_unit: object | None = None
    deadband: float = 0.0
    deadband_unit: object | None = None
    release_threshold: float | None = None
    release_threshold_unit: object | None = None

    def __post_init__(self) -> None:
        self.mass = const_value(self.mass)
        self.damping = const_value(self.damping)
        self.stiffness = const_value(self.stiffness)
        self.max_velocity = const_value(self.max_velocity)
        # Zero is "unbounded" for the excursion and "off" for the deadband; both are one-sided.
        self.max_excursion = abs(const_value(self.max_excursion)) if self.max_excursion else 0.0
        self.deadband = abs(const_value(self.deadband)) if self.deadband else 0.0
        self.release_threshold = (
            abs(const_value(self.release_threshold))
            if self.release_threshold is not None
            else self.deadband
        )
        if self.release_threshold > self.deadband:
            raise ValueError("admittance release-threshold must not exceed deadband")
