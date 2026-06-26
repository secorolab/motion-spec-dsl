# Motion Spec DSL Tutorial

This tutorial explains the `.robmot` language. It focuses on what you author in a
motion spec file; setup and generation commands are in [README.md](README.md).

## File Shape

A `.robmot` file usually has four layers:

```robmot
ns app = "https://secorolab.github.io/models/demo/"

ENVIRONMENT (ns=app) world {
    runtime: MuJoCo,
    ASSETS {
        kinova-mjcf: RobotAsset { model: KinovaGen3, xml: "../robots/kg3.xml" }
    },
    ASSEMBLY {
        Robot kinova using <kinova-mjcf> {
            chain: { root: link-base, end: link-ee }
        }
    }
}

MOTION_SPEC (ns=app) m_move {
    CONTEXT {
        w: World {
            twist-ee-base: VelocityTwist { of: link-ee, wrt: link-base }
        },
        s: Spec {
            vel-z-ref: LinearVelocity = 0.1 m/s
        }
    }

    WHEN {}
    WHILE {
        keep-z: keeping <w.twist-ee-base>.linvel.z equal to <s.vel-z-ref>
    }
    UNTIL {}
}

CONSTRAINT_HANDLER (ns=app) handler_move {
    CONTEXT {
        w: World {
            gravity: Gravity
        },
        s: Spec {
            gravity-vec: FreeVector { x = 0.0, y = 0.0, z = -9.81 m/s2 }
        }
    }

    MOTION: <m_move>
    CONTROL_MODE: JointTorque

    CONTROLLERS {
        ctrl-keep-z: PID { constraint: <m_move.keep-z>, Kp = 1.0, Ki = 0.0, Kd = 0.1 }
    }

    SOLVERS {
        arm-solver: Solver {
            robot: <world.kinova>,
            algorithm: ACHD,
            root: <world.kinova.chain.root>,
            end: <world.kinova.chain.end>,
            gravity: <w.gravity> equal to <s.gravity-vec>
        }
    }
}
```

The important idea is separation:

- `ENVIRONMENT` describes runtime assets and assembled robots or objects.
- `MOTION_SPEC` describes guarded motion constraints.
- `CONSTRAINT_HANDLER` says how a motion is monitored, controlled, and solved.

## Namespaces And Imports

Every top-level declaration is created inside a namespace:

```robmot
ns app = "https://secorolab.github.io/models/demo/"
```

Use imports to split specifications across files:

```robmot
import "common.robmot"
```

Imports are resolved relative to the current file and participate in textX cross-reference
resolution. References use angle brackets:

```robmot
<m_move.keep-z>
<handler_move.arm-solver>
<world.kinova.chain.root>
```

## Environments

An environment declares the target runtime, assets, and assembled instances.

```robmot
ENVIRONMENT (ns=app) world {
    runtime: MuJoCo,
    ASSETS {
        kinova-mjcf: RobotAsset { model: KinovaGen3, xml: "../robots/kg3.xml" },
        table: SceneObject
    },
    ASSEMBLY {
        Robot kinova using <kinova-mjcf> {
            chain: { root: link-base, end: link-ee }
        },
        Object table-1 using <table> {
            shape: Box,
            size: { x: 0.5 m, y: 0.4 m, z: 0.1 m },
            position: { x: 0.4 m, y: 0.0 m, z: 0.2 m },
            color: { r: 0.2, g: 0.2, b: 0.2, a: 1.0 }
        }
    }
}
```

Supported runtimes are `MuJoCo` and `RealRobot`. Assets can be `RobotAsset`,
`AttachmentAsset`, or `SceneObject`. Assemblies can instantiate `Robot`, `Attachment`,
or `Object`.

Robot assemblies commonly define a `chain` so solvers can refer to
`<world.kinova.chain.root>` and `<world.kinova.chain.end>`.

## Contexts

Contexts declare the quantities that constraints compare.

`World` contains measured or structural quantities:

```robmot
w: World {
    twist-ee-base: VelocityTwist { of: link-ee, wrt: link-base },
    wrench-ee: Wrench { ref-point: point-ee-origin, as-seen-by: frame-ee },
    pose-ee-base: Pose { of: link-ee, wrt: link-base, as-seen-by: link-base },
    q-j2: JointPosition { of: joint-2 },
    link-ee: Link,
    gravity: Gravity
}
```

`Pre`, `Spec`, and `Post` contain reference values, thresholds, snapshots, and trajectory
inputs:

```robmot
s: Spec {
    vel-zero: LinearVelocity = 0.0 m/s,
    force-ref: Force = 10.0 N,
    gravity-vec: FreeVector { x = 0.0, y = 0.0, z = -9.81 m/s2 },
    start-pose: Pose = Snapshot of <w.pose-ee-base>
}
```

World quantity types include `VelocityTwist`, `Wrench`, `Pose`, `JointPosition`,
`KinematicChain`, `Frame`, `SceneObject`, `Link`, and `Gravity`.

Context quantity types include `AngularVelocity`, `LinearVelocity`, `Force`, `Torque`,
`Distance`, `LinearDistance`, `Angle`, `AngularDistance`, `Direction`, `FreeVector`,
`Pose`, `Position`, `Orientation`, `Trajectory`, and `TrajectoryProgress`.

## Constraints

Each motion has three guard sections:

- `WHEN`: entry conditions
- `WHILE`: constraints to maintain during motion execution
- `UNTIL`: exit conditions

```robmot
MOTION_SPEC (ns=app) m_contact {
    CONTEXT {
        w: World {
            wrench-ee: Wrench { ref-point: point-ee-origin, as-seen-by: frame-ee }
        },
        s: Spec {
            force-ref: Force = 10.0 N
        },
        p: Post {
            overload: Force = 25.0 N
        }
    }

    WHEN {}

    WHILE {
        regulate-force: keeping <w.wrench-ee>.force.z equal to <s.force-ref>
    }

    UNTIL {
        stop-overload: <w.wrench-ee>.force.z is larger than <p.overload>
    }
}
```

Supported comparisons are:

```robmot
equal to <s.reference>
greater than <s.threshold>
less than <s.threshold>
between <s.lower> and <s.upper>
```

The DSL also accepts readable aliases such as `is larger than`, `is smaller than`,
`away from`, and `up to`.

Supported views include:

```robmot
<w.twist>.linvel.x
<w.twist>.angvel.z
<w.wrench>.force.z
<w.wrench>.torque.z
<w.pose>.position.x
<w.pose>.orientation.yaw
<w.joint-position>
```

Axes are `x`, `y`, `z`, `roll`, `pitch`, and `yaw` where they make sense for the selected
quantity.

## Distance Constraints

Use the explicit distance form when constraining distance between two poses:

```robmot
keep-distance: distance between <w.pose-a> and <w.pose-b> between <s.lower> and <s.upper>
```

Both endpoints must be `Pose` world quantities with compatible geometric properties.
Use `<w.pose>.position.x` for a single coordinate; use `distance between` for a scalar
distance between two poses.

## Inline Values

Small one-off values can be declared inline in a constraint instead of being named in
the context:

```robmot
keep-y: keeping <w.twist-ee-base>.linvel.y equal to Spec[vel-y-zero: LinearVelocity = 0.0 m/s]
```

Use named context quantities when values are shared, referenced from multiple places, or
important enough to inspect in generated graphs.

## Reuse

You can reuse context declarations, quantities, constraints, controllers, and solvers.

```robmot
CONTEXT (ns=app) common_context {
    w: World {
        pose-ee-base: Pose { of: link-ee, wrt: link-base, as-seen-by: link-base }
    }
}

MOTION_SPEC (ns=app) m_a {
    CONTEXT {
        <common_context.w>
    }
    WHEN {}
    WHILE {}
    UNTIL {}
}
```

Inside another motion or handler, reference earlier declarations:

```robmot
<m_approach.keep-j2>
<handler_approach.ctrl-keep-j2>
<handler_approach.arm-solver>
```

Aliases let you reuse something under a local name:

```robmot
ctrl-j2: <handler_approach.ctrl-keep-j2>
```

## Monitors

`WHEN` and `UNTIL` constraints need monitors in the handler. A monitor can trigger an
event or set a flag while its target is active.

```robmot
MONITORS {
    mon-contact: monitor <m_contact.stop-overload> and trigger event evt-overload when active,
    mon-until: monitor <m_contact.until> and set flag flg-contact-done while active
}
```

`<motion.until>` targets the whole `UNTIL` section. `UNTIL any` means the section is
active when any exit constraint is active; `UNTIL all` requires all exit constraints.

```robmot
UNTIL any {
    contact: <w.wrench-ee>.force.z is larger than <s.contact-force>,
    overload: <w.wrench-ee>.force.z is larger than <s.overload-force>
}
```

## Controllers

Controllers bind `WHILE` constraints to command semantics.

```robmot
CONTROLLERS {
    ctrl-vel-z: PID { constraint: <m_move.keep-z>, Kp = 5.0, Ki = 1.0, Kd = 1.0 },
    ctrl-force-z: PID { constraint: <m_contact.regulate-force>, Kp = 1.0 } as Force apply at <w.link-ee>,
    ctrl-joint: PID { constraint: <m_move.keep-j2>, Kp = 10.0, Kd = 0.5 } as Torque
}
```

`PID` gains are optional, so P, PI, PD, I, D, and full PID controllers can be expressed
by including only the terms you need.

Command type is inferred when possible. Use `as Force` for force commands and
`apply at <link>` when the command needs a link target. `JointPosition` constraints must
use `as Torque`.

The grammar also recognizes `Impedance`, `ABAG`, and `FeedForward`, but validation and RDF
support are intentionally narrower than the syntax. If a controller family or mapping is
not supported by the current backend, validation fails before generation.

## Solvers

A handler must declare at least one solver.

```robmot
SOLVERS {
    arm-solver: Solver {
        robot: <world.kinova>,
        algorithm: ACHD,
        root: <world.kinova.chain.root>,
        end: <world.kinova.chain.end>,
        gravity: <w.gravity> equal to <s.gravity-vec>
    }
}
```

Supported solver names in the language are `ACHD`, `RNE`, `CommandForwarding`,
`VelocityDistribution`, and `ForceDistribution`. Validation limits which solver algorithms
can be used with `CONTROL_MODE: JointTorque` and the selected controllers.

If a handler has multiple solvers, controllers that could route to more than one solver
must say which one they use:

```robmot
ctrl-x: PID { constraint: <m_move.keep-x>, Kp = 1.0 } via <arm-solver>
```

Cross-handler solver references include the handler name:

```robmot
<handler_approach.arm-solver>
```

## Trajectories

Trajectory quantities are declared in `Spec` and then used through pose subspaces.

```robmot
s: Spec {
    alpha: TrajectoryProgress,
    plane-normal: Direction { as-seen-by: link-base } { x = 0.0, y = 0.0, z = 1.0 },
    radius-vec: FreeVector { as-seen-by: link-base } { x = 0.10, y = 0.0, z = 0.0 m },
    start-pose: Pose = Snapshot of <w.pose-ee-base>,
    center-pos: Position = Snapshot of <w.pose-ee-base>.position + <s.radius-vec>,
    traj: Trajectory = Circle {
        start: <s.start-pose>,
        center: <s.center-pos>,
        plane-normal: <s.plane-normal>,
        alpha: <s.alpha>
    }
}
```

Follow a trajectory by comparing the measured pose subspace with the trajectory subspace:

```robmot
WHILE {
    follow-pos: keeping <w.pose-ee-base>.position equal to <s.traj>.position
}
```

Available trajectory forms are `Lerp`, `Circle`, `Arc`, `Helix`, and `Figure8`.

## Validation Rules To Expect

The parser catches syntax and reference errors. Semantic validation then catches modeling
errors, including:

- duplicate constraint names inside a motion
- `WHILE` constraints without controllers
- `WHEN` or `UNTIL` constraints without monitors
- handlers whose controllers or monitors reference constraints from another primary motion
- force commands missing `apply at <link>`
- joint position constraints missing `as Torque`
- controllers that need explicit `via <solver>` in multi-solver handlers
- unsupported controller or solver mappings

When generation fails, fix validation errors in the `.robmot` source first. Generated
JSON-LD should be treated as output, not the place to patch model intent.
