# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""`angle between`, `distance of`, `projection of` and `norm of`: the operator each lowers to and
the wrench a controller commands along it."""

from __future__ import annotations

import pytest
from rdflib.namespace import RDF

from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf.dataset import build_dataset
from motion_spec_dsl.rdf_parser.vocab import (
    GEOM_OP,
    GEOM_OP_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    RBDYN_OP,
    RBDYN_OP_EXT,
)
from support import BASE, BASE_TEXT, CTRL, HOLD, POSE_TABLE_TOP, SPEC, TABLE_PLANE, TWIST

PRIMITIVES = (
    ",\n        direction tool-up { as-seen-by: <gripper.g_base.g_pinch> } = (0, 0, -1)"
    ",\n        direction base-up { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 0, 1)"
    ",\n        direction wall-normal { as-seen-by: <gripper.g_base.g_pinch> } = (1, 0, 0)"
    ",\n        plane wall { of: <gripper.g_base.g_pinch>, normal: <shared.spec.wall-normal> }"
    ",\n        angle align-band = 0.05 rad"
    ",\n        direction rail-a-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (1, 0, 0)"
    ",\n        line rail-a { of: <gripper.g_base.g_pinch>, along: <shared.spec.rail-a-axis> }"
    ",\n        direction rail-b-axis { as-seen-by: <kinova.base_link.base_link_origin> } = (0, 1, 0)"
    ",\n        line rail-b { of: <table.table_top>, along: <shared.spec.rail-b-axis> }"
) + TABLE_PLANE
# The base model with `held: keeping CONSTRAINT` driven by CONTROLLER.
HELD = (
    BASE_TEXT.replace(TWIST, TWIST + POSE_TABLE_TOP, 1)
    .replace(SPEC, SPEC + PRIMITIVES, 1)
    .replace(HOLD, f"{HOLD},\n        held: keeping CONSTRAINT", 1)
    .replace(CTRL, f"{CTRL},\n        CONTROLLER", 1)
)
ALIGN = "angle between <shared.spec.tool-up> and <shared.spec.base-up>"
IMPEDANCE = (
    "impedance ctrl-held { constraint: <home.held>, stiffness: 40, damping: 8 }"
    " apply at <gripper.g_base>"
)
PID = "pid ctrl-held { constraint: <home.held>, Kp: 100, Ki: 0, Kd: 0, decay: 0 }"
ORIENTATION = "equal to <spec.home-pose>.orientation within <shared.spec.align-band>"


@pytest.mark.parametrize(
    ("relation", "pointwise"),
    [
        pytest.param("equal to 0 rad within <shared.spec.align-band>", True, id="zero_target"),
        pytest.param("equal to pi/6 rad within <shared.spec.align-band>", False, id="nonzero_target"),
        # `less than <cone>` is `between 0 and <cone>`: the gradient row would free a DOF.
        pytest.param("less than 0.1 rad", True, id="less_than"),
        pytest.param("greater than 0.3491 rad", False, id="greater_than"),
        pytest.param("between 0 rad and 0.1 rad", True, id="band_from_zero"),
        pytest.param("between 0.2 rad and 0.4 rad", False, id="band_above_zero"),
    ],
)
def test_a_point_target_drives_the_rotation_vector_and_a_cone_the_gradient(relation, pointwise) -> None:
    source = HELD.replace("CONSTRAINT", f"{ALIGN} {relation}", 1).replace("CONTROLLER", PID, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    rotation_vectors = list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))
    gradients = list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))
    assert (len(rotation_vectors), len(gradients)) == ((1, 0) if pointwise else (0, 1))
    assert len(list(graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections))) == 1


def test_a_bound_and_a_cone_at_one_value_do_not_share_a_chain() -> None:
    """`less than 0.5` (2-DOF) and `equal to 0.5` (1-DOF) fold to the same scalar tokens; only
    the row shape separates them, or the second silently gets the first's chain."""
    source = HELD.replace(
        "CONSTRAINT",
        f"{ALIGN} less than 0.5 rad,\n"
        f"        point: keeping {ALIGN} equal to 0.5 rad within <shared.spec.align-band>",
        1,
    ).replace(
        "CONTROLLER",
        f"{PID},\n        pid ctrl-point {{ constraint: <home.point>, Kp: 1, Ki: 0, Kd: 0, decay: 0 }}",
        1,
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.RotationVectorFromDirections))) == 1
    assert len(list(graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections))) == 1


def test_plane_angle_gradient_keeps_the_authored_operand_order() -> None:
    """The scalar's from-directions is unordered, but the gradient's in1/in2 carry the order:
    swapped, it points away from the target and the controller diverges."""
    source = HELD.replace(
        "CONSTRAINT",
        "angle between <shared.spec.wall> and <shared.spec.table> equal to pi/6 rad"
        " within <shared.spec.align-band>",
        1,
    ).replace("CONTROLLER", PID, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [angle] = graph.subjects(RDF.type, GEOM_OP.PlanarAngleFromDirections)
    [gradient] = graph.subjects(RDF.type, GEOM_OP_EXT.AngleGradientFromDirections)
    first, second = graph.value(gradient, GEOM_OP.in1), graph.value(gradient, GEOM_OP.in2)
    assert {first, second} == set(graph.objects(angle, GEOM_OP["from-directions"]))
    assert "table-normal" in str(second) and "table-normal" not in str(first)


def test_line_line_projection_keeps_the_authored_operand_order() -> None:
    """Swapping the operands would silently compute s2 instead of s1."""
    source = HELD.replace(
        "CONSTRAINT",
        "projection of <shared.spec.rail-a> on <shared.spec.rail-b> equal to 0.02 m"
        " within <shared.spec.satisfied-band>",
        1,
    ).replace("CONTROLLER", PID, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [op] = graph.subjects(RDF.type, GEOM_OP_EXT.LineOnLineProjection)
    assert str(graph.value(op, GEOM_OP.in1)).endswith("rail-a-axis")
    assert str(graph.value(op, GEOM_OP.in2)).endswith("rail-b-axis")
    [diff] = [
        node
        for node in graph.subjects(RDF.type, GEOM_OP_EXT.PoseDiffEvaluator)
        if graph.value(node, GEOM_OP.out) == graph.value(op, GEOM_OP.pose)
    ]
    assert str(graph.value(diff, GEOM_OP.in1)).endswith("pose-ee-base")
    assert str(graph.value(diff, GEOM_OP.in2)).endswith("pose-table-top")


@pytest.mark.parametrize(
    ("view", "relation", "moments", "folds", "forces"),
    [
        pytest.param("orientation", ORIENTATION, 3, 2, 0, id="whole_orientation"),
        pytest.param("orientation.z", ORIENTATION, 1, 0, 0, id="one_angular_axis"),
        pytest.param(
            "position.z",
            "equal to <spec.home-pose>.position within <shared.spec.satisfied-band>",
            0,
            0,
            1,
            id="one_linear_axis",
        ),
    ],
)
def test_an_impedance_commands_the_wrench_its_subspace_takes(view, relation, moments, folds, forces) -> None:
    """An orientation error once lifted the arm as a linear force; it must turn it instead."""
    source = HELD.replace("CONSTRAINT", f"<shared.world.pose-ee-base>.{view} {relation}", 1).replace(
        "CONTROLLER", IMPEDANCE, 1
    )
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    assert len(list(graph.subjects(RDF.type, RBDYN_OP_EXT.WrenchFromDirectionAndMoment))) == moments
    assert len(list(graph.subjects(RDF.type, RBDYN_OP.AddWrench))) == folds
    assert len(list(graph.subjects(RDF.type, RBDYN_OP.WrenchFromPositionDirectionAndMagnitude))) == forces


@pytest.mark.parametrize(
    ("constraint", "controller", "operator", "wrench"),
    [
        pytest.param(
            "distance of <shared.world.pose-ee-base> from <shared.spec.table> greater than 0.05 m",
            f"{PID} as force apply at <gripper.g_base>",
            GEOM_OP_EXT.PointPlaneToLinearDistance,
            RBDYN_OP.WrenchFromPositionDirectionAndMagnitude,
            id="force_along_point_plane_distance",
        ),
        pytest.param(
            "angle between <shared.spec.tool-up> and <shared.spec.table> equal to 0 rad"
            " within <shared.spec.align-band>",
            IMPEDANCE,
            GEOM_OP_EXT.DirectionPlaneToAngularDistance,
            RBDYN_OP_EXT.WrenchFromDirectionAndMoment,
            id="moment_along_incident_angle",
        ),
    ],
)
def test_a_geometric_expression_commands_along_its_own_gradient(
    constraint, controller, operator, wrench
) -> None:
    """The expression has no named axis, so its wrench reuses the operator's runtime gradient."""
    source = HELD.replace("CONSTRAINT", constraint, 1).replace("CONTROLLER", controller, 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [op] = graph.subjects(RDF.type, operator)
    [command] = graph.subjects(RDF.type, wrench)
    assert graph.value(command, RBDYN_OP.direction) == graph.value(op, GEOM_OP_EXT.gradient)


def test_a_norm_across_a_direction_keeps_the_vectors_kind() -> None:
    source = HELD.replace(
        "CONSTRAINT",
        "norm of <shared.world.twist-ee-base>.linvel across <shared.spec.base-up> greater than 0.05 m/s",
        1,
    ).replace(",\n        CONTROLLER", "", 1)
    model = motion_spec_metamodel().model_from_str(source, file_name=str(BASE))
    graph = build_dataset(model)[0].default_graph

    [op] = graph.subjects(RDF.type, GEOM_OP_EXT.VectorNorm)
    assert str(graph.value(op, GEOM_OP.direction)).endswith("base-up")
    norm = graph.value(op, GEOM_OP_EXT.norm)
    assert graph.value(norm, QUDT_SCHEMA.hasQuantityKind) == QUDT_QKIND.LinearVelocity
    assert "M-PER-SEC" in str(graph.value(norm, QUDT_SCHEMA.unit))
