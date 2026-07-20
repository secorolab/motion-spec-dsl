# motion-spec-dsl

[![build](https://github.com/secorolab/motion-spec-dsl/actions/workflows/build.yml/badge.svg)](https://github.com/secorolab/motion-spec-dsl/actions/workflows/build.yml)
[![test](https://github.com/secorolab/motion-spec-dsl/actions/workflows/test.yml/badge.svg)](https://github.com/secorolab/motion-spec-dsl/actions/workflows/test.yml)

A textX DSL for guarded robot motions. It validates `.robmot` models and emits the
motion graph, application manifest, imported scene/FSM graphs, and DSL provenance as
separate JSON-LD artifacts.

## Workspace setup

Use the workspace virtual environment and local metamodel checkout:

```bash
cd /path/to/workspace
source .venv/bin/activate
export METAMODELS_PATH="$PWD/src/metamodels"
pip install -e src/motion-spec-dsl --no-deps
```

The package depends on the workspace siblings `motion_spec`, `scene-dsl`, `coord_dsl`,
and `rdf-utils`.

## Generate

```bash
textx generate src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot \
  --target jsonld -o /tmp/motion-spec-model
```

The application manifest imports the motion, scene, FSM, and provenance graphs and
lists the SHACL files required by the emitted metamodel terms. Metamodel IRIs remain
portable; the downstream loader resolves them through `METAMODELS_PATH`.

## Verify

```bash
cd src/motion-spec-dsl
pytest
ruff check src tests
```

The representative model is [models/pick_place_single/pick_place_single.robmot](models/pick_place_single/pick_place_single.robmot).
