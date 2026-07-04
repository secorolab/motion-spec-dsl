# Motion Spec DSL Tutorial

This tutorial explains the `.robmot` language. It focuses on what you author in a
motion spec file; setup and generation commands are on the [home page](index.md).

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

A `MOTION_SPEC` may open with an optional `MOVE: "…"` human-readable description before its
`CONTEXT`.

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

An optional `timestep` sets the simulation step:

```robmot
ENVIRONMENT (ns=app) world {
    runtime: MuJoCo,
    timestep: 1.0 ms,
    ...
}
```

### Assemblies

Every assembly instance may carry a `position` and `orientation` (empty `{}` means
identity), plus type-specific fields.

`Object` instances describe scene geometry — either from an asset `xml`, or as a primitive
`shape` (`Box`, `Sphere`, `Cylinder`) with `size`, `mass`, `color`, and `friction`:

```robmot
Object table using <table-mjcf> {
    position: { z: 0.72 m },
    orientation: {}
},
Object box using <box-asset> {
    shape: Box,
    size: { x: 0.4 m, y: 0.3 m, z: 0.1 m },
    mass: 1.5,
    color: { r: 0.2, g: 0.2, b: 0.2, a: 1.0 },
    friction: { slide: 1.0, torsion: 0.005, roll: 0.0001 }
}
```

`Attachment` instances mount an `AttachmentAsset` (a gripper, an FT sensor) onto a body,
site, or frame of another assembly with `attach-to`. Grippers can add a `prefix`,
`tcp-site`, and `actuator`; a robot can expose an `ft-sensor` bound to a frame site:

```robmot
Robot robot using <kinova-mjcf> {
    attach-to: <table>.site(table_top),
    ft-sensor: wrist_ft frame-site wrist_ft_site,
    chain: { root: base_link, end: g_pinch }
},
Attachment ft using <ft-mjcf> {
    attach-to: <robot>.site(pinch_site)
},
Attachment gripper using <gripper-mjcf> {
    attach-to: <ft>.site(wrist_ft_site),
    prefix: g_,
    tcp-site: g_pinch,
    actuator: g_fingers_actuator
}
```

`attach-to` targets are `<assembly>.body(name)`, `.site(name)`, or `.frame(name)`.

### Traces

`TRACE` enables an end-effector path trace in the viewer:

```robmot
TRACE {
    enabled: true,
    length: 4096,
    color: { r: 0.1, g: 0.8, b: 1.0, a: 1.0 },
    ee: [g_pinch]
}
```

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

World quantity types include `VelocityTwist`, `Wrench`, `Pose`, and `JointPosition`.
World entity/field declarations include `KinematicChain`, `Frame`, `SceneObject`,
`Link`, and `Gravity`.

Context quantity types include `AngularVelocity`, `LinearVelocity`, `LinearAcceleration`,
`AngularAcceleration`, `Force`, `Torque`, `Distance`, `Angle`, `Direction`, `FreeVector`,
`Pose`, `Position`, `Orientation`, `VelocityTwist`, `AccelerationTwist`, `Wrench`,
`Dimensionless`, `Duration`, and `TrajectoryProgress`.
`LinearDistance` and `AngularDistance` are accepted as aliases for `Distance` and
`Angle`. Reference generator declarations include `Trajectory`, `VelocityProfile`,
and `Admittance` (see their own sections below).

## Snapshots

A `Snapshot` captures a live quantity at runtime and holds it as a reference — useful for
"hold the pose you were in when this motion started". Snapshot any world quantity or one of
its subspaces:

```robmot
s: Spec {
    home-pose: Pose = Snapshot of <w.pose-ee-base>,
    home-x:    LinearDistance = Snapshot of <w.pose-ee-base>.position.x
}
```

Add an offset with `+ <ref>`, and choose when the capture happens with `on`:

```robmot
target-x:   LinearDistance = Snapshot of <w.pose-ee-base>.position.x + <s.forward-distance>,
start-pose: Pose = Snapshot of <w.pose-ee-base> on entry
```

`on task` (the default) samples once and holds it for the whole run; `on entry` re-samples
each time the owning motion's state is (re-)entered.

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
outside <s.lower> and <s.upper>
```

`between` is satisfied inside the band; `outside` is satisfied out of it (an out-of-band
guard, e.g. a force exceeding a symmetric threshold in either direction). The DSL also
accepts readable aliases such as `is larger than`, `is smaller than`, `away from`, and
`up to`.

Supported views include:

```robmot
<w.twist>.linvel.x
<w.twist>.angvel.z
<w.wrench>.force.z
<w.wrench>.torque.z
<w.pose>.position.x
<w.pose>.orientation.yaw
<w.joint-position>
Norm of <w.wrench>.force
```

Axes are `x`, `y`, `z`, `roll`, `pitch`, and `yaw` where they make sense for the selected
quantity. `Norm of <view>` reduces a vector view to its scalar magnitude — for example
`released: Norm of <w.ext-force>.force less than <s.release-threshold>`.

An `elapsed` view measures time since the motion's state was entered, so `UNTIL` can hold
for a duration:

```robmot
UNTIL {
    dwell: elapsed greater than <s.wait-time>,   // s.wait-time: Duration
    inline: elapsed greater than 5000.0 ms
}
```

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

A monitor target is a single constraint (`<motion.constraint>`), or a whole section via
`<motion.until>` / `<motion.when>`. `<motion.until>` targets the whole `UNTIL` section:
`UNTIL any` means the section is active when any exit constraint is active; `UNTIL all`
requires all exit constraints.

```robmot
UNTIL any {
    contact: <w.wrench-ee>.force.z is larger than <s.contact-force>,
    overload: <w.wrench-ee>.force.z is larger than <s.overload-force>
}
```

An event monitor can debounce with `for <duration>` (the target must stay active that long
before the event fires) and name a `fallback` hold motion for a `WHEN` precondition — the
FSM waits in the fallback motion until the precondition is met, then advances:

```robmot
MONITORS {
    mon-settled: monitor <m_push.overload> and trigger event evt-overload when active for 50.0 ms,
    mon-ready:   monitor <m_grasp.when> and trigger event evt-start when active fallback <m_hold>
}
```

## Controllers

Controllers bind `WHILE` constraints to command semantics.

```robmot
CONTROLLERS {
    ctrl-vel-z: PID { constraint: <m_move.keep-z>, Kp = 5.0, Ki = 1.0, Kd = 1.0 },
    ctrl-force-z: PID { constraint: <m_contact.regulate-force>, Kp = 1.0, Ki = 0.0, Kd = 0.0 } as Force apply at <w.link-ee>,
    ctrl-joint: PID { constraint: <m_move.keep-j2>, Kp = 10.0, Ki = 0.0, Kd = 0.5 } as Torque
}
```

`PID` controllers must spell out `Kp`, `Ki`, and `Kd`; set unused terms explicitly to
`0.0`.

Command type is inferred when possible. Use `as Force` for force commands and
`apply at <link>` when the command needs a link target. `JointPosition` constraints must
use `as Torque`.

### PID options

Beyond the gains, a PID accepts (all optional, before the gains):

- `profile: <ref>` — track a [velocity profile](#velocity-profiles) instead of a raw setpoint.
- `measured_derivative: <view>` — feed a measured velocity into the derivative term instead
  of differentiating the error (avoids differentiating a noisy/stepped reference).
- `output-saturation` / `integral-saturation` — see [Clamps and Saturations](#clamps-and-saturations).
- `decay = <rate>` — a leaky (decaying) integral term for anti-windup.

```robmot
ctrl-follow: PID {
    constraint: <arc.follow-pos>,
    measured_derivative: <w.twist-ee-base>.linvel,
    Kp = 200, Ki = 0, Kd = 40, decay = 0
}
```

### Impedance

An `Impedance` controller maps a constraint error to a force through a virtual
spring-damper. It takes `Stiffness` and `Damping` (and an optional integral gain `Ki`),
and usually needs `apply at <link>`:

```robmot
ctrl-support: Impedance { constraint: <grasp-hold.support-elbow-z>, Stiffness = 800, Damping = 80 }
    apply at <w.half-arm-2-link>
```

### Feed-forward

A `FeedForward` controller forwards an equality-constraint reference straight through to a
solver, with no gains. It is the way to drive a `CommandForwarding` solver — for example a
gripper open/close command:

```robmot
ctrl-close-gripper: FeedForward { constraint: <grasp-hold.close-gripper> }
```

`ABAG` is accepted by the grammar but is not backed by the current code generator; use
`PID`, `Impedance`, or `FeedForward`. Unsupported controller families or mappings fail
validation before generation.

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

`ACHD`/`RNE` solvers accept an optional `regularization` (DLS/Tikhonov damping λ) after
`algorithm`, and clamp their outputs with a `limits` block — see
[Clamps and Saturations](#clamps-and-saturations):

```robmot
arm-solver: Solver {
    robot: <world.kinova>,
    algorithm: ACHD,
    regularization: 0.05,
    root: <world.kinova.chain.root>,
    end: <world.kinova.chain.end>,
    gravity: <w.gravity> equal to <s.gravity-vec>
}
```

`CommandForwarding` is a pass-through solver (e.g. for a gripper actuator) and needs only
`robot` and `algorithm`:

```robmot
gripper-solver: Solver { robot: <world.robot>, algorithm: CommandForwarding }
```

If a handler has multiple solvers, controllers that could route to more than one solver
must say which one they use:

```robmot
ctrl-x: PID { constraint: <m_move.keep-x>, Kp = 1.0, Ki = 0.0, Kd = 0.0 } via <arm-solver>
```

Cross-handler solver references include the handler name:

```robmot
<handler_approach.arm-solver>
```

## Clamps and Saturations

Runtime limits are authored explicitly with a `Saturation`. Nothing is clamped unless
you say so. A saturation is either symmetric around zero or an explicit range:

```robmot
Saturation { max: <s.limit> }              // clamp to [-limit, +limit]
Saturation { lower: <s.lo>, upper: <s.hi> } // clamp to [lo, hi]
```

The bounds are context references, so the limit values live in a `Spec` like any other
quantity and show up in the generated graph.

Solvers limit their outputs with a `limits` block. Targets are `torque`,
`linear-acceleration`, and `angular-acceleration`, each matched to its own quantity type:

```robmot
s: Spec {
    tau-max:  Torque = 40.0 Nm,
    lin-max:  LinearAcceleration = 6.0 m/s2,
    ang-max:  AngularAcceleration = 8.0 rad/s2
}

arm-solver: Solver {
    robot: <world.kinova>,
    algorithm: ACHD,
    limits {
        torque: Saturation { max: <s.tau-max> },
        linear-acceleration: Saturation { max: <s.lin-max> },
        angular-acceleration: Saturation { max: <s.ang-max> }
    },
    root: <world.kinova.chain.root>,
    end: <world.kinova.chain.end>,
    gravity: <w.gravity> equal to <s.gravity-vec>
}
```

`torque` clamps the per-joint command torques; the acceleration targets clamp the matching
acceleration-energy constraints fed to the solver.

Controllers clamp their own signals. `output-saturation` bounds the control signal (its
type must match the controller's command type) and `integral-saturation` bounds a PID's
integral state (anti-windup). Both come **before** the `Kp`/`Ki`/`Kd` terms:

```robmot
ctrl-j2: PID {
    constraint: <m_move.keep-j2>,
    output-saturation: Saturation { max: <s.out-max> },
    integral-saturation: Saturation { max: <s.i-max> },
    Kp = 10.0, Ki = 1.0, Kd = 0.5
} as Torque
```

## Trajectories

Trajectory reference generators are declared in `Spec` and then used through pose
subspaces.

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

Every form is driven by an `alpha` progress reference (a `TrajectoryProgress`, advanced by
the runtime from 0 to 1). The available forms and their parameters are:

- `Lerp { start, goal, alpha }` — straight interpolation; optional `profile` easing
  (`Linear`, `EaseIn`, `EaseOut`, `EaseInOut`).
- `Circle { start, center, plane-normal, alpha }`
- `Arc { start, end, amplitude, plane-normal, alpha }`
- `Helix { start, center, axis, pitch, revolutions, alpha }`
- `Figure8 { anchor, radius, plane-normal, alpha }` — optional `form` (`Gerono` or
  `Bernoulli`).

```robmot
traj: Trajectory = Lerp {
    start: <s.start-pose>,
    goal:  <s.goal-pose>,
    alpha: <s.alpha>,
    profile: EaseInOut
}
```

Positions and free vectors carry `as-seen-by` frames, e.g.
`plane-normal: Direction { as-seen-by: link-base } { x = 0.0, y = 0.0, z = 1.0 }`.

## Velocity Profiles

A `VelocityProfile` shapes how fast a setpoint is approached (trapezoidal or S-curve),
bounded by authored limits. Declare it in `Spec` and hand it to a controller's `profile`:

```robmot
s: Spec {
    vmax: LinearVelocity = 0.10 m/s,
    amax: LinearAcceleration = 0.30 m/s2,
    jmax: LinearJerk = 2.0 m/s3,
    vp: VelocityProfile = Profile {
        max_velocity: <s.vmax>,
        max_acceleration: <s.amax>,
        max_jerk: <s.jmax>,
        shape: SCurve
    }
}
```

`max_jerk` and `shape` (`Trapezoidal` or `SCurve`) are optional; an optional
`measured_velocity: <view>` closes the loop on a measured speed. Use it from a controller:

```robmot
ctrl-gap: PID { constraint: <m.gap>, profile: <s.vp>, Kp = 30.0, Ki = 0.0, Kd = 4.0 } as Force apply at <w.link-ee>
```

## Admittance

An `Admittance` reference generator turns a measured external-force axis into a velocity
setpoint through a virtual mass-damper(-spring) — the basis of compliant "push me around"
behaviour. Declare one per axis in `Spec` and constrain the measured velocity to it:

```robmot
s: Spec {
    admit-vx: Admittance { force: <w.ext-force>.force.x, mass = 2.0, damping = 30.0, stiffness = 0.0, max-velocity = 0.30 m/s }
}
```

```robmot
WHILE {
    comply-x: keeping <w.twist-ee-base>.linvel.x equal to <s.admit-vx>
}
```

`stiffness` is optional (omit for a pure mass-damper); `max-velocity` caps the resulting
setpoint.

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
- saturation bounds whose quantity type does not match the target (e.g. a `torque` limit
  that is not a `Torque`), duplicate solver limit targets, or `integral-saturation` on a
  non-PID controller

When generation fails, fix validation errors in the `.robmot` source first. Generated
JSON-LD should be treated as output, not the place to patch model intent.
