# kinova_gen3_robotiq_communication_test

This model generates a communication-only smoke test for a Kinova Gen3,
Robotiq FT 300-S, and Robotiq 2F-85. It has no motion solver, URDF, KDL runtime,
ROS dependency, or background control loop.

The generated program performs these bounded steps:

1. Connect to the Kinova, read and print all seven joint positions.
2. Connect to the FT sensor, print one reading, zero it through `robif2b`, and
   print the next reading.
3. Activate the gripper driver and print its position and status without
   sending a position command.
4. Shut down every successfully configured device in reverse order and exit.

The Kinova low-level servoing mode is never started. The gripper's normal
`configure` operation deactivates and reactivates it, so a small initialization
movement or click is possible even though no open/close command is sent.

## Generate, build, and run

From `ws/src`, the `run` command performs all three steps. This communication-only
test does not create a trajectory log; the final printed path is the generated
build directory:

```sh
source ../setup-grc.zsh
MOTION_SPEC_KINOVA_IP=192.168.1.12 \
MOTION_SPEC_ROBOTIQ_FT_PORT=/dev/ttyUSB0 \
MOTION_SPEC_ROBOTIQ_GRIPPER_PORT=/dev/ttyUSB1 \
motion-spec run motion-spec-dsl/models/kinova_gen3_robotiq_communication_test/kinova_gen3_robotiq_communication_test.robmot
```

By default all three devices are checked. To check a subset, set
`MOTION_SPEC_COMMUNICATION_DEVICES` to a comma-separated list (without spaces)
of `arm`, `ft`, and `gripper`. The alias `kinova` can be used for `arm`, and
`all` selects every device. Ports for unselected devices are not opened.

For example, from `ws/src`:

```sh
# Kinova only
MOTION_SPEC_COMMUNICATION_DEVICES=arm motion-spec run \
  motion-spec-dsl/models/kinova_gen3_robotiq_communication_test/kinova_gen3_robotiq_communication_test.robmot

# Kinova and gripper
MOTION_SPEC_COMMUNICATION_DEVICES=arm,gripper motion-spec run \
  motion-spec-dsl/models/kinova_gen3_robotiq_communication_test/kinova_gen3_robotiq_communication_test.robmot

# FT sensor only
MOTION_SPEC_COMMUNICATION_DEVICES=ft motion-spec run \
  motion-spec-dsl/models/kinova_gen3_robotiq_communication_test/kinova_gen3_robotiq_communication_test.robmot
```

The active `robif2b` package must be built with `ENABLE_KORTEX`,
`ENABLE_ROBOTIQ_FT`, and `ENABLE_ROBOTIQ_GRIPPER`. This workspace has already
been rebuilt with those options.

## Run an already generated test

Keep the robot stationary, unload the FT sensor, clear the workspace, and keep
the emergency stop accessible. Then run:

```sh
MOTION_SPEC_KINOVA_IP=192.168.1.12 \
MOTION_SPEC_ROBOTIQ_FT_PORT=/dev/ttyUSB0 \
MOTION_SPEC_ROBOTIQ_GRIPPER_PORT=/dev/ttyUSB1 \
  /tmp/kinova_gen3_robotiq_communication_test/build/main
```

`MOTION_SPEC_ROBOTIQ_FT_PORT` accepts either `ttyUSB0` or `/dev/ttyUSB0`.
Successful output ends with:

```text
[1/4] Kinova: connecting to 192.168.1.12
      joint positions [rad]: ... seven values ...
[2/4] FT sensor: connecting to /dev/ttyUSB0
      before zero: F=[...] N, T=[...] Nm
      after zero:  F=[...] N, T=[...] Nm
[3/4] Gripper: connecting to /dev/ttyUSB1
      position=..., status=2, moving=false
[4/4] Shutting down configured devices
PASS: requested communication checks succeeded
```

Any failed stage prints `FAIL: ...`, shuts down the devices that were already
configured, and exits with status 1.

The zeroed FT values should be close to zero but will not be exactly zero due
to sensor noise. Gripper `status=2` means activation completed.
