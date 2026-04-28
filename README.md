# motion-spec-dsl

A [textX](https://textx.github.io/textX/) DSL for specifying guarded motions, transforming `.robmot` files into JSON-LD models for robotic control code generation.

## Installation

```bash
pip install -e .
```

## DSL Syntax

The DSL allows you to specify robotic motions with constraints and control parameters. A typical `.rob_mot` file contains:

Files can import other `.rob_mot` files with textX import scoping:

```
import "common.rob_mot"
ns app = "https://secorolab.github.io/models/generated/"
```

Imported files are resolved relative to the current file and participate in cross-reference resolution and JSON-LD generation.

### Motion Specification Block

```
ns app = "https://secorolab.github.io/models/generated/"

MOTION_SPEC (ns=app) motion_name {
    MOVE: "Description of the motion"

    CONTEXT:
        c1: World {
            twist-ee-base: VelocityTwist { of: link-ee, wrt: link-base },
            wrench-ee-ee:  Wrench
        }
        c2: Spec {
            vel-down: LinearVelocity = -0.05 m/s,
            vel-zero: LinearVelocity = 0.0 m/s
        }
        c3: Post {
            force-contact: Force = 5.0 N
        }

    WHEN:

    WHILE:
        wi1: keeping twist-ee-base.linear.z equal to Spec[vel-down]
        wi2: keeping twist-ee-base.linear.x equal to Spec[vel-zero]
        wi3: keeping twist-ee-base.linear.y equal to Spec[vel-zero]

    UNTIL:
        u1: wrench-ee-ee.force.z is larger than Post[force-contact]
}
```

### Constraint Handler Block

```
CONSTRAINT_HANDLER (ns=app) ctrl_name {
    CONTEXT:
        c1: World {
            chain-arm: KinematicChain,
            frame-base: Frame,
            gravity: UniformGravitationalField { x = 0.0, y = 0.0, z = -9.81 m/s2 }
        }

    MOTION: motion_name

    CONTROLLERS:
        ctrl-vel-z: PID { constraint: wi1, Kp = 5.0, Ki = 1.0, Kd = 1.0 }
        ctrl-vel-x: PID { constraint: wi2, Kp = 5.0, Ki = 1.0, Kd = 1.0 }
        ctrl-force: PID { constraint: wi3, Kp = 5.0, Ki = 1.0, Kd = 1.0 } outputs Force apply at World[link-ee] feed to cartesian Force

    MONITORS:
        monitor u1 and trigger event evt-contact when active

    PRIORITIES:
        prio-move: level = 1 { drivers: [ spec-acc-ee-move ] }

    SOLVER:
        algorithm: ACHD,
        chain: World[chain-arm],
        root: World[frame-base],
        gravity: World[gravity]
}
```

### Key Elements

- **Namespace declaration**: Define IRI prefix with `ns app = "..."`
- **Imports**: Reuse specs from another `.rob_mot` file with `import "file.rob_mot"`
- **Context blocks**: Declare physical quantities, reference values, and constraints
- **Constraint conditions**: `WHILE` (equality constraints), `UNTIL` (termination conditions)
- **Controllers**: PID controllers with optional `outputs`, `apply at`, and `feed to` routing
- **Monitors**: Trigger events or flags when a constraint becomes active
- **Priorities**: Prioritization levels for constraint resolution
- **Solver configuration**: Arm solver configuration plus base velocity composition and force distribution solvers

## Usage

Generate 7 numbered JSON-LD files (00-misc.json through 06-solver-specification.json):

```bash
textx generate input.rob_mot --target jsonld
```

Generate a single merged JSON-LD file:

```bash
textx generate input.rob_mot --target jsonld --single
```

Examples:

```bash
# Generate separate files in models/ex/ directory
textx generate models/ex.rob_mot --target jsonld

# Generate single file in custom output directory  
textx generate models/ex.rob_mot --target jsonld --single -o output/
```

## Generator Algorithm

The JSON-LD generator is handler-rooted and walks the parsed model directly.
It keeps only small per-handler collections while emitting RDF triples, instead
of building a second normalized analysis model.

The current algorithm is:

```text
Algorithm 1 Motion-Spec Graph Construction

Input: Parsed DSL model M
Output: JSON-LD graph J

1:  H <- all ConstraintHandler declarations in M
2:  S <- constraint specs reused by more than one referenced motion
3:  G <- empty rdflib Dataset
4:  bind ontology namespaces
5:
6:  for each handler h in H do
7:      m <- motion referenced by h
8:      bind h and m namespaces
9:      collect world quantities from m and h
10:     collect value variables from m, constraints, and solver gravity values
11:     collect resolved WHEN / WHILE / UNTIL constraints from m
12:
13:     emit authored entities:
14:         structural entities
15:         world quantities
16:         value variables
17:         constraints
18:         motion node
19:
20:     emit derived entities:
21:         scalar views
22:         map operations
23:         controllers, monitors, error signals, and evaluators
24:         solver drivers, interfaces, and solver nodes
25:  end for
26:
27:  J <- serialize G as JSON-LD with the same namespace bindings
28:  return J
```

The generator is rooted at `ConstraintHandler` declarations and uses the RDF
graph as the contract; intermediate Python objects are private implementation
details.
