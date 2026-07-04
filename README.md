# motion-spec-dsl

`motion-spec-dsl` is a [textX](https://textx.github.io/textX/) language for writing
guarded robot motion specifications in `.robmot` files and generating RDF motion-spec
graphs for the rest of the motion-spec toolchain.

The repository contains:

- a textX grammar for `.robmot`
- semantic validation for motions, handlers, controllers, monitors, and solvers
- RDF/JSON-LD graph generation
- example models and pytest fixtures

Language syntax is documented in [tutorial.md](tutorial.md).

## Setup

Use Python 3.10 or newer.

```bash
cd src/motion-spec-dsl
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

The package depends on `motion_spec`. In the monorepo setup this is expected to be
available from the surrounding workspace or installed in the same environment.

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

Example `.robmot` files live in `models/` and `tests/fixtures/valid/`.

Start with:

- `models/ex.robmot` for a multi-motion example with environment, monitors,
  reusable controllers, and solvers
- `tests/fixtures/valid/01_core_semantics/01_standalone_manipulator.robmot` for a
  minimal velocity constraint
- `tests/fixtures/valid/05_trajectories/01_circle.robmot` for a trajectory-following
  motion
