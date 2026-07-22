# SPDX-License-Identifier: MPL-2.0
"""Shared fixtures: the minimal valid model and a mutate-then-parse harness."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"
BASE = Path(__file__).parent / "fixtures" / "base.robmot"


@pytest.fixture(autouse=True)
def _metamodels_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("METAMODELS_PATH", os.environ.get("METAMODELS_PATH", str(METAMODELS)))


@pytest.fixture(scope="session")
def base_source() -> str:
    return BASE.read_text()


@pytest.fixture
def parse_source():
    """Parse DSL source as if it sat next to the base fixture, so imports resolve."""
    from motion_spec_dsl.registration import motion_spec_metamodel

    def _parse(source: str):
        return motion_spec_metamodel().model_from_str(source, file_name=str(BASE))

    return _parse


@pytest.fixture
def parse_mutated(base_source: str, parse_source):
    """Parse the base model with `old` replaced by `new`; asserts the mutation applied."""

    def _mutate(old: str, new: str):
        assert old in base_source, f"mutation anchor not found in base.robmot: {old!r}"
        return parse_source(base_source.replace(old, new, 1))

    return _mutate
