# motion-spec-dsl

[![build](https://github.com/secorolab/motion-spec-dsl/actions/workflows/build.yml/badge.svg)](https://github.com/secorolab/motion-spec-dsl/actions/workflows/build.yml)
[![test](https://github.com/secorolab/motion-spec-dsl/actions/workflows/test.yml/badge.svg)](https://github.com/secorolab/motion-spec-dsl/actions/workflows/test.yml)
[![docs](https://github.com/secorolab/motion-spec-dsl/actions/workflows/docs.yml/badge.svg)](https://secoro.uni-bremen.de/motion-spec-dsl/)

`motion-spec-dsl` is a [textX](https://textx.github.io/textX/) language for writing
guarded robot motion specifications in `.robmot` files and generating RDF motion-spec
graphs for the rest of the motion-spec toolchain.

The repository contains:

- a textX grammar for `.robmot`
- semantic validation for motions, handlers, controllers, monitors, and solvers
- RDF/JSON-LD graph generation
- example models and pytest fixtures

The Python package is organized as `motion_spec_dsl/{rdf, validation}` plus the shared
`controller_semantics` and `domain` modules. `rdf` is a package: `builder.py` holds the
graph builder, with its constant tables and pure helpers split into `_specs.py`/`_helpers.py`.

Language syntax is documented in the [tutorial](https://secoro.uni-bremen.de/motion-spec-dsl/tutorial.html)
(source in [docs/](docs/); build locally with `sphinx-build -b html docs docs/_build/html`).

## Setup

Use Python 3.10 or newer.

```bash
cd src/motion-spec-dsl
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

The package depends on the workspace siblings `motion_spec` and `coord_dsl` and a patched
`rdflib`. In the monorepo setup these are expected to be available from the surrounding
workspace or installed in the same environment (see `.github/workflows/test.yml` for how CI
resolves them from git).

Graph generation also needs `METAMODELS_PATH` so the generated application manifest can
map ontology IRIs to local metamodel files:

```bash
export METAMODELS_PATH=/path/to/secorolab/metamodels
```

Run the test suite with:

```bash
pytest
```

## Pipeline

The normal pipeline has two authored inputs and two generated outputs.

1. Write a `.robmot` motion specification.
   The file declares namespaces, optional imports, environment/runtime assets, motions,
   and constraint handlers.

2. Parse and validate the motion spec.
   textX loads the grammar from `src/motion_spec_dsl/metamodels/motion_spec.tx`, resolves
   imports and cross references, and runs semantic validators. Invalid combinations such
   as missing controllers, missing monitors, unsupported solver/controller mappings, or
   ambiguous multi-solver routing fail before graph generation.

3. Generate the RDF graph and application manifest.

   ```bash
   textx generate models/ex.robmot --target jsonld -o build/
   ```

   This writes:

   - `build/ex.json`: the generated motion-spec graph serialized as JSON-LD
   - `build/ex-app.json`: an application manifest listing the generated graph,
     required SHACL/ontology constraint files, and local IRI mappings

4. Feed the generated graph and manifest into downstream code generation or runtime
   tooling.
   The DSL package stops at graph generation. Codegen consumes the generated JSON-LD and
   manifest as its inputs.

## Useful Commands

Generate next to the source file:

```bash
textx generate models/ex.robmot --target jsonld
```

Generate into an output directory:

```bash
textx generate models/ex.robmot --target jsonld -o build/
```

Run validation through the parser without keeping generated files:

```bash
python - <<'PY'
from motion_spec_dsl.registration import motion_spec_metamodel

motion_spec_metamodel().model_from_file("models/ex.robmot")
print("ok")
PY
```

## Examples

Example `.robmot` files live in `models/` and `tests/fixtures/valid/`. The
`models/` directory is a source-tree example set, not installed package data.

Start with:

- `models/ex.robmot` for a multi-motion example with environment, monitors,
  reusable controllers, and solvers
- `models/pick_place_single.robmot` for the pick-place integration fixture with
  an imported FSM
- `tests/fixtures/valid/01_core_semantics/01_standalone_manipulator.robmot` for a
  minimal velocity constraint
- `tests/fixtures/valid/05_trajectories/01_circle.robmot` for a trajectory-following
  motion
