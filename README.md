# motion-spec-dsl

A [textX](https://textx.github.io/textX/) DSL for specifying guarded motions, transforming `.rob_mot` files into JSON-LD models for robotic control code generation.

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


```
