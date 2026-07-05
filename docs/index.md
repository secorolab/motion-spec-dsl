# motion-spec-dsl

```{toctree}
:hidden:
:maxdepth: 2

tutorial
```

`motion-spec-dsl` is a [textX](https://textx.github.io/textX/) language for writing
guarded robot motion specifications in `.robmot` files and generating RDF motion-spec
graphs for the rest of the motion-spec toolchain.

- **[Tutorial](tutorial.md)** — the `.robmot` language, section by section.

## Setup

Use Python 3.10 or newer.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

Graph generation needs `METAMODELS_PATH` so the generated application manifest can map
ontology IRIs to local metamodel files:

```bash
export METAMODELS_PATH=/path/to/secorolab/metamodels
```

## Pipeline

You author a `.robmot` file; the DSL validates it and emits an RDF graph plus an
application manifest, which the downstream `motion-spec` toolchain lowers to C++.

```bash
textx generate models/ex.robmot --target jsonld -o build/
```

This writes:

- `build/ex.jsonld` — the motion-spec graph as JSON-LD
- `build/ex-app.jsonld` — an application manifest listing the graph, its SHACL/ontology
  constraints, and local IRI mappings

The DSL stops at graph generation; code generation consumes the JSON-LD and manifest.

## Validate without keeping output

```bash
python - <<'PY'
from motion_spec_dsl.registration import motion_spec_metamodel

motion_spec_metamodel().model_from_file("models/ex.robmot")
print("ok")
PY
```
