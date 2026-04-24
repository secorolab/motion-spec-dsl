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
        algorithm: Vereshchagin,
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

The JSON-LD generator is handler-rooted and builds the RDF graph in ordered
passes. It keeps only a small amount of indexed state:

- authored handlers
- motions referenced by those handlers
- per-motion context (`world` quantities, value variables, constraints)
- derived constraint metadata needed for `while` evaluators and solver nodes

The current algorithm is:

```text
Algorithm 1 Motion-Spec Graph Construction

Input: Parsed DSL model M
Output: JSON-LD graph J

1:  H <- all ConstraintHandler declarations in M
2:  X <- empty motion scope index
3:  for each handler h in H do
4:      if h references motion m and m is not in X then
5:          collect m.world quantities
6:          collect m.pre/spec/post value variables
7:          collect m.when, m.while, m.until constraints
8:          record m and its local scope in X
9:      end if
10: end for
11:
12: C <- empty derived constraint list
13: for each motion scope x in X do
14:     for each authored constraint c in x do
15:         resolve the referenced world quantity and viewed property
16:         derive scalar quantity ids and constraint kind metadata
17:         if c is a while-equality then
18:             derive shared/local while error signal ids
19:             derive acceleration-energy ids when applicable
20:         end if
21:         append derived record for c to C
22:     end for
23: end for
24:
25: G <- empty rdflib Dataset
26: bind common graph namespaces
27:
28: materialize authored entities into G:
29:     structural entities
30:     world quantities
31:     value variables
32:     constraints
33:     motions
34:     constraint handlers, controllers, and monitors
35:
36: materialize derived entities into G:
37:     scalar views
38:     while error signals
39:     acceleration energies
40:     constraint evaluators
41:     solver entities and motion drivers
42:     map operations
43:     transform operations
44:
45: J <- serialize G as JSON-LD with the same namespace bindings
46: return J
```

This algorithm is intentionally not a generic graph search. It starts from
`ConstraintHandler` declarations because handlers determine which motions,
controllers, monitors, and solver structures are relevant to the emitted RDF.
Derived nodes are then produced from those authored objects in a fixed order.

The generator still refuses to infer missing semantic structure from authored
names. Unsupported derived cases, such as pose-distance transforms without
explicit model-backed fields, fail explicitly rather than guessing.
