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

The JSON-LD generator is handler-rooted and compiles the parsed model into an
RDF dataset through cached analysis records plus ordered materialization
passes.

The main cached records are:

- authored handlers
- motions referenced by those handlers
- one `MotionScope` per referenced motion
- global indexes for world quantities and value variables
- controlled, monitored, and shared constraint usage sets
- one normalized `ConstraintData` record per assembled constraint

The current algorithm is:

```text
Algorithm 1 Motion-Spec Graph Construction

Input: Parsed DSL model M
Output: JSON-LD graph J

1:  H <- all ConstraintHandler declarations in M
2:  Mref <- motions referenced by H, deduplicated by entity identity
3:  build MotionScope for each motion in Mref:
4:      collect local world quantities
5:      collect local value variables
6:      collect resolved WHEN / WHILE / UNTIL constraints
7:
8:  build global indexes from the scopes and handlers:
9:      world quantities
10:     value variables
11:     implicit structural entities
12:     controlled constraint usage
13:     monitored constraint usage
14:     shared constraint reuse across motions
15:
16: C <- empty derived constraint list
17: for each motion scope x do
18:     for each constraint c in x do
19:         resolve the viewed quantity and scalar property
20:         classify c as equality / greater-than / less-than / bilateral
21:         derive referenced threshold or reference variables
22:         derive error-signal ids when c is controlled or monitored
23:         mark whether c is reused across motions
24:         append normalized ConstraintData record to C
25:     end for
26: end for
27:
28: G <- empty rdflib Dataset
29: bind namespaces owned by handlers and referenced motions
30:
31: materialize authored entities into G:
32:     structural entities
33:     world quantities
34:     value variables
35:     constraints
36:     motions
37:     handlers, controllers, and monitors
38:
39: materialize derived entities into G:
40:     scalar views
41:     error signals and evaluators
42:     solver interfaces, motion drivers, and solver nodes
43:     map operations
44:     transform operations
45:
46: J <- serialize G as JSON-LD with the same namespace bindings
47: return J
```

The generator is rooted at `ConstraintHandler` declarations and emits authored
entities before derived solver and mapping structures.
