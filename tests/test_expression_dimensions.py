# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Dimension inference over quantity-expression trees: `classes.dimensions.infer` decides what
`+`/`-`/`*`/`/` produce, and validation raises before the RDF emitter ever sees a bad tree.
"""

from __future__ import annotations

import pytest
from textx.exceptions import TextXSemanticError

SPEC_ANCHOR = "linear-velocity zero-linvel = 0.0 m/s"


def _mutate(parse_mutated, declaration: str):
    return parse_mutated(SPEC_ANCHOR, f"{SPEC_ANCHOR},\n        {declaration}")


def test_same_kind_addition_is_accepted(parse_mutated) -> None:
    _mutate(
        parse_mutated,
        "length sum-ok = <shared.spec.satisfied-band> + <shared.spec.satisfied-band>",
    )


def test_mismatched_kind_addition_is_rejected(parse_mutated) -> None:
    with pytest.raises(TextXSemanticError, match="different kinds of quantity"):
        _mutate(
            parse_mutated,
            "length sum-bad = <shared.spec.satisfied-band> + <shared.world.twist-ee-base>.linvel.z",
        )


def test_a_product_that_maps_to_a_known_kind_is_accepted(parse_mutated) -> None:
    _mutate(
        parse_mutated,
        "mass k = 1.2 kg,\n"
        "        linear-acceleration a = 9.81 m/s^2,\n"
        "        force computed = <spec.k> * <spec.a>",
    )


def test_a_product_with_no_known_kind_is_rejected(parse_mutated) -> None:
    with pytest.raises(TextXSemanticError, match="names no known"):
        _mutate(
            parse_mutated,
            "mass k = 1.2 kg,\n        dimensionless bad = <spec.k> * <spec.k>",
        )


def test_geometry_kind_addition_of_the_same_kind_is_accepted(parse_mutated) -> None:
    _mutate(
        parse_mutated,
        "position sum-pos = <shared.world.pose-ee-base>.position "
        "+ <shared.world.pose-ee-base>.position",
    )


def test_geometry_kind_multiplication_is_rejected(parse_mutated) -> None:
    with pytest.raises(TextXSemanticError, match="geometry kind"):
        _mutate(
            parse_mutated,
            "position bad-mul = <shared.world.pose-ee-base>.position * 2.0 1",
        )


def test_a_composite_leaf_with_no_selected_axis_is_rejected_in_multiplication(
    parse_mutated,
) -> None:
    with pytest.raises(TextXSemanticError, match="select a scalar subspace axis"):
        _mutate(
            parse_mutated,
            "linear-velocity bad-axis = <shared.world.twist-ee-base>.linvel * 2.0 1",
        )


def test_a_declared_type_must_equal_the_expressions_inferred_type(parse_mutated) -> None:
    with pytest.raises(TextXSemanticError, match="declared"):
        _mutate(parse_mutated, "distance mismatch = <shared.spec.satisfied-band>")


def test_literal_division_by_zero_is_rejected(parse_mutated) -> None:
    with pytest.raises(TextXSemanticError, match="division by zero"):
        _mutate(
            parse_mutated,
            "mass k = 1.2 kg,\n        mass bad-div = <spec.k> / 0.0 kg",
        )
