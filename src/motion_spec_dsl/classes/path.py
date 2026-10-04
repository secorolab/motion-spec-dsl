# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Classes bound to geometric-path and reference-generator grammar rules."""

from __future__ import annotations

from motion_spec_dsl.classes.coordinates import const_value


class LerpSpec:
    def __init__(self, parent, start, goal) -> None:
        self.parent = parent
        self.start = start
        self.goal = goal


class CircleSpec:
    """A circle through `start` around `center`; the radius is their in-plane distance."""

    def __init__(self, parent, start, center, plane_normal) -> None:
        self.parent = parent
        self.start = start
        self.center = center
        self.plane_normal = plane_normal


class ArcSpec:
    """An arc from `start` to `end` bowing `amplitude` off the chord; chord/2 is a semicircle."""

    def __init__(self, parent, start, end, amplitude, plane_normal) -> None:
        self.parent = parent
        self.start = start
        self.end = end
        self.amplitude = amplitude
        self.plane_normal = plane_normal


class HelixSpec:
    """A helix through `start` winding around `center` along `axis`."""

    def __init__(self, parent, start, center, axis, pitch, revolutions) -> None:
        self.parent = parent
        self.start = start
        self.center = center
        self.axis = axis
        self.pitch = pitch
        self.revolutions = revolutions


class Figure8Spec:
    """A figure-eight centred on `anchor`'s position, lobes along its in-plane rotation."""

    def __init__(self, parent, anchor, radius, plane_normal, form) -> None:
        self.parent = parent
        self.anchor = anchor
        self.radius = radius
        self.plane_normal = plane_normal
        self.form = form or "gerono"


class PathValue:
    def __init__(self, parent, lerp, circle, arc, helix, figure8) -> None:
        self.parent = parent
        self.lerp = lerp
        self.circle = circle
        self.arc = arc
        self.helix = helix
        self.figure8 = figure8


class ProfileSpec:
    """Authored limits and shape for an online velocity profile."""

    def __init__(
        self, parent, max_velocity, max_acceleration, measured_velocity, max_jerk, shape
    ) -> None:
        self.parent = parent
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.measured_velocity = measured_velocity
        self.max_jerk = max_jerk
        self.shape = shape or "trapezoidal"


class AdmittanceSpec:
    """A mass-damper-spring yield to a measured force axis, `force` a view onto it.

    `max-excursion` bounds how far the yield travels from where the motion started: a limit, not
    a spring, so letting go leaves the arm where it is. `deadband` is the force that counts as
    none, subtracted from the magnitude so the force stays continuous as a push crosses it.
    """

    def __init__(
        self,
        parent,
        force,
        mass,
        damping,
        stiffness,
        max_velocity,
        max_velocity_unit,
        max_excursion,
        max_excursion_unit,
        deadband,
        deadband_unit,
        release_threshold,
        release_threshold_unit,
    ) -> None:
        self.parent = parent
        self.force = force
        self.mass = const_value(mass)
        self.damping = const_value(damping)
        self.stiffness = const_value(stiffness)
        self.max_velocity = const_value(max_velocity)
        self.max_velocity_unit = max_velocity_unit
        # Zero is "unbounded" for the excursion and "off" for the deadband.
        self.max_excursion = abs(const_value(max_excursion)) if max_excursion else 0.0
        self.max_excursion_unit = max_excursion_unit
        self.deadband = abs(const_value(deadband)) if deadband else 0.0
        self.deadband_unit = deadband_unit
        self.release_threshold = (
            abs(const_value(release_threshold)) if release_threshold else self.deadband
        )
        self.release_threshold_unit = release_threshold_unit
