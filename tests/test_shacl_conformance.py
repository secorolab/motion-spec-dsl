# SPDX-License-Identifier: MPL-2.0
"""Regression: the example models must pass SHACL validation end-to-end.

Runs JSON-LD generation (`textx generate`) followed by `motion_spec.check` for each
example and asserts the manifest conforms. This guards against silent SHACL drift
(e.g. coordinate-view typing regressions).

Skipped automatically when the workspace layout, tooling, or network access required
to resolve the upstream comp-rob2b constraint graphs is unavailable.
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
from pathlib import Path

import pytest

WORKSPACE = Path(__file__).resolve().parents[3]
MODELS_DIR = WORKSPACE / "src" / "motion-spec-dsl" / "models"
METAMODELS_DIR = WORKSPACE / "src" / "metamodels"

EXAMPLES = [
    "pick_place_single",
    "pick_place_single_rnea",
    "pick_place_single_wait",
    "admittance_arc_single",
]


def _comp_rob2b_reachable() -> bool:
    try:
        socket.create_connection(("comp-rob2b.github.io", 443), timeout=5).close()
        return True
    except OSError:
        return False


@pytest.mark.parametrize("model", EXAMPLES)
def test_example_model_conforms_to_shacl(tmp_path, model):
    source = MODELS_DIR / f"{model}.robmot"
    if not source.exists():
        pytest.skip(f"example model not present: {source}")
    if not METAMODELS_DIR.exists():
        pytest.skip(f"metamodels not present: {METAMODELS_DIR}")
    if shutil.which("textx") is None:
        pytest.skip("textx CLI not available")
    if not _comp_rob2b_reachable():
        pytest.skip("upstream comp-rob2b constraint graphs unreachable (offline)")

    env = {**os.environ, "METAMODELS_PATH": str(METAMODELS_DIR)}
    gen_dir = tmp_path / model

    gen = subprocess.run(
        ["textx", "generate", str(source), "--target", "jsonld", "-o", str(gen_dir)],
        cwd=MODELS_DIR, env=env, capture_output=True, text=True,
    )
    assert gen.returncode == 0, f"generation failed:\n{gen.stdout}\n{gen.stderr}"

    manifest = gen_dir / f"{model}-app.json"
    assert manifest.exists(), f"manifest not generated: {manifest}"

    check = subprocess.run(
        [sys.executable, "-m", "motion_spec.check", str(manifest)],
        cwd=MODELS_DIR, env=env, capture_output=True, text=True,
    )
    assert check.returncode == 0 and "Conforms: True" in check.stdout, (
        f"{model} did not conform:\n{check.stdout}\n{check.stderr}"
    )
