# motion-spec-dsl

A [textX](https://textx.github.io/textX/) DSL for authoring guarded robot motion specifications and generating RDF/JSON-LD motion-spec graphs.

The DSL source files use the `.robmot` extension.

## Installation

```bash
pip install -e .
```

## Usage

Generate JSON-LD next to the input file:

```bash
textx generate model.robmot --target jsonld
```

Choose an output directory:

```bash
textx generate model.robmot --target jsonld -o build/
```

The generator currently writes two files:

- `<stem>.json`: the generated RDF graph serialized as JSON-LD
- `<stem>-app.json`: an application manifest listing the generated graph and ontology constraint files

The generator builds one RDF graph per input model. The old numbered multi-file JSON-LD output is not supported.

## Package Layout

```text
motion_spec_dsl/
  domain.py             textX classes for parsed DSL objects
  namespaces.py         namespace-aware object base helpers
  semantics.py          shared semantic classification records
  validation/           semantic validation phases
  rdf.py                RDF graph construction
  registration.py       textX language and generator entrypoints
  metamodels/           textX grammar
```

## Minimal Example

```robmot
ns app = "https://secorolab.github.io/models/test/"

ROBOT (ns=app) kinova {
    type: Manipulator,
    model: KinovaGen3,
    urdf: "../robots/kg3.urdf",
    chain: {
        root: link-base,
        end: link-ee
    }
}

MOTION_SPEC (ns=app) m_move {
    CONTEXT {
        world: World {
            twist-ee-base: VelocityTwist { of: link-ee, wrt: link-base }
        },
        spec: Spec {
            vel-z-ref: LinearVelocity = 0.1 m/s
        }
    }

    WHEN {}

    WHILE {
        keep-vel-z: <world.twist-ee-base>.linvel.z equal to <spec.vel-z-ref>
    }

    UNTIL {}
}

CONSTRAINT_HANDLER (ns=app) handler_move {
    CONTEXT {
        world: World {
            gravity: Gravity
        },
        spec: Spec {
            gravity-vec: Vector { x = 0.0, y = 0.0, z = -9.81 m/s2 }
        }
    }

    MOTION: <m_move>

    CONTROLLERS {
        ctrl-vel-z: PID { constraint: <m_move.keep-vel-z>, Kp = 1.0, Ki = 0.0, Kd = 0.1 }
    }

    SOLVERS {
        arm_solver: Solver {
            robot: <kinova>,
            algorithm: ACHD,
            root: <kinova.chain.root>,
            end: <kinova.chain.end>,
            gravity: <world.gravity> equal to <spec.gravity-vec>
        }
    }
}
```

## Language Overview

### Imports And Namespaces

```robmot
import "common.robmot"
ns app = "https://secorolab.github.io/models/demo/"
```

Imports are resolved relative to the current file and participate in textX cross-reference resolution.

### Contexts

`World` declares physical quantities and structural entities. `Pre`, `Spec`, and `Post` declare context quantities used as reference values, thresholds, and solver values.

World quantity types: `VelocityTwist`, `Wrench`, `Pose`, `JointPosition`, `KinematicChain`, `Frame`, `Link`, `Gravity`.

Context quantity types: `AngularVelocity`, `LinearVelocity`, `Force`, `Torque`, `Distance`, `LinearDistance`, `Angle`, `AngularDistance`, `Vector`.

`LinearDistance` is accepted as an alias for `Distance`.

### Constraints

Motion sections are `WHEN`, `WHILE`, and `UNTIL`.

```robmot
WHILE {
    keep-vel-z: <world.twist-ee-base>.linvel.z equal to <spec.velocity-ref>,
    keep-force-z: <world.wrench-ee>.force.z greater than <spec.force-threshold>
}
```

Supported expressions: `equal to`, `greater than`, `less than`, and `between <lower> and <upper>`.

Supported view subspaces:

- `VelocityTwist`: `.linvel.<axis>`, `.angvel.<axis>`
- `Wrench`: `.force.<axis>`, `.torque.<axis>`
- `Pose`: `.position.<axis>`, `.orientation.<axis>`
- `JointPosition`: no subspace or axis

Axes are `x`, `y`, and `z`.

### Explicit Distance

Distance between two positions is explicit:

```robmot
keep-distance: distance between <world.pose-platform-shoulder> and <world.pose-platform-ee> between <spec.distance-lower> and <spec.distance-upper>
```

Both endpoints must be `Pose` quantities with explicit `of` and `wrt` properties and the same `wrt` frame. The generator resolves the corresponding relative pose and emits a `geom-op:PoseToLinearDistance` operation.

Use `.position.<axis>` for one coordinate of a pose position. Do not use bare `.position` for distance.

### Constraint Handlers

A `CONSTRAINT_HANDLER` binds a motion to controllers, monitors, and solvers.

Controller options:

- `as <QuantityType>` selects the command type when it cannot be inferred
- `for Posture` marks posture control for `JointPosition` constraints
- `apply at <world.link>` selects a link target for commands that need one
- `via <handler.solver>` selects a solver when a handler assembles multiple solvers

Solver algorithms: `ACHD`, `RNE`, `VelocityDistribution`, `ForceDistribution`.

## Generator Design

The RDF generator is handler-rooted. It walks each authored `ConstraintHandler`, follows the referenced `MotionSpec`, collects only the world quantities and context quantities needed for that handler, and emits RDF triples directly.

```text
Algorithm 1 Motion-Spec Graph Construction

Input: Parsed DSL model M
Output: RDF dataset D and JSON-LD context C

1:  H <- all ConstraintHandler declarations in M and imported models
2:  S <- constraint specs reused by more than one referenced motion
3:  D <- empty rdflib Dataset
4:  bind ontology namespaces
5:
6:  for each handler h in H do
7:      m <- motion referenced by h
8:      bind h and m namespaces
9:      collect world quantities from m and h
10:     collect context quantities from m, constraints, and solver gravity values
11:     collect resolved WHEN / WHILE / UNTIL constraints from m
12:
13:     emit authored graph nodes:
14:         structural entities
15:         world quantities
16:         context quantities
17:         constraints
18:         guarded motion
19:
20:     emit derived graph nodes:
21:         scalar coordinate views
22:         map operations, including explicit distance operations
23:         controllers, monitors, error signals, and evaluators
24:         solver drivers, interfaces, and solver nodes
25:  end for
26:
27:  serialize D using C
```
