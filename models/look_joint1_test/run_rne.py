# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Run a bounded joint-1 trajectory through recursive Newton-Euler inverse dynamics."""

from __future__ import annotations

import argparse
import math

import PyKDL as kdl
import mj_kdl_wrapper as mjk

HOME = [0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708]
TARGET = [2.0 * math.pi / 3.0, *HOME[1:]]
KP = [100.0, 40.0, 40.0, 40.0, 20.0, 20.0, 15.0]
KD = [20.0, 12.0, 12.0, 12.0, 8.0, 8.0, 6.0]
MOVE_DURATION = 20.0
SETTLE_DURATION = 20.0


def joint_array(values: list[float]) -> kdl.JntArray:
    result = kdl.JntArray(len(values))
    for index, value in enumerate(values):
        result[index] = value
    return result


def build_env() -> tuple[mjk.Env, mjk.Robot]:
    gripper = mjk.AttachmentSpec()
    gripper.mjcf_path = mjk.menagerie.asset_path(
        "robotiq_2f85/2f85.xml", env_var="MJ_KDL_GRIPPER"
    )
    gripper.attach_to = mjk.AttachTarget(mjk.AttachKind.Site, "pinch_site")
    gripper.prefix = "g_"

    scene = mjk.SceneSpec()
    scene.timestep = 0.001
    scene.add_floor = True
    scene.add_skybox = True
    robot_spec = mjk.RobotSpec()
    robot_spec.path = mjk.menagerie.model_path("kinova_gen3", env_var="MJ_KDL_MODEL")
    robot_spec.pos = [0.0, 0.0, 0.72]
    robot_spec.attachments = [gripper]
    scene.robots = [robot_spec]
    env = mjk.Env.build(scene)

    tool = mjk.ToolFrameSpec()
    tool.tool_body = "g_base"
    tool.tcp_site = "g_pinch"
    return env, env.create_robot("base_link", "bracelet_link", tool=tool)


def reference(time_s: float) -> tuple[list[float], list[float], list[float]]:
    u = min(1.0, max(0.0, time_s / MOVE_DURATION))
    position = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    velocity = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / MOVE_DURATION
    acceleration = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / MOVE_DURATION**2
    delta = TARGET[0] - HOME[0]
    q_ref = HOME[:]
    qd_ref = [0.0] * 7
    qdd_ref = [0.0] * 7
    q_ref[0] += delta * position
    qd_ref[0] = delta * velocity
    qdd_ref[0] = delta * acceleration
    return q_ref, qd_ref, qdd_ref


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    env, robot = build_env()
    viewer = None
    try:
        chain = robot.kdl_chain()
        rne = kdl.ChainIdSolver_RNE(chain, kdl.Vector(0.0, 0.0, -9.81))
        external = [kdl.Wrench.Zero() for _ in range(chain.getNrOfSegments())]
        robot.ctrl_mode = mjk.CtrlMode.TORQUE
        env.on_reset = lambda _context: robot.set_joint_pos(HOME, call_forward=False)
        env.reset()
        viewer = mjk.SimulateViewer.open(robot, "look_joint1_test") if args.gui else None

        max_torque = [0.0] * 7
        max_hold_error = [0.0] * 7
        start = env.time()
        while env.time() - start < MOVE_DURATION + SETTLE_DURATION:
            robot.update()
            elapsed = env.time() - start
            q_ref, qd_ref, _qdd_ref = reference(elapsed)
            q = joint_array(robot.jnt_pos_msr)
            qd = joint_array(robot.jnt_vel_msr)
            qdd = kdl.JntArray(7)
            tau = kdl.JntArray(7)
            for index in range(7):
                qdd[index] = 0.0
            if rne.CartToJnt(q, qd, qdd, external, tau) < 0:
                raise RuntimeError("RNE inverse dynamics failed")
            command = []
            for index in range(7):
                value = (
                    tau[index]
                    + KP[index] * (q_ref[index] - q[index])
                    + KD[index] * (qd_ref[index] - qd[index])
                )
                if index == 0:
                    value = max(-105.0, min(105.0, value))
                command.append(value)
                max_torque[index] = max(max_torque[index], abs(value))
                max_hold_error[index] = max(max_hold_error[index], abs(q[index] - q_ref[index]))
            robot.jnt_trq_cmd = command
            if viewer is not None:
                if not viewer.is_running() or not viewer.step():
                    break
            elif not robot.step():
                break

        robot.update()
        print("final q:", [round(value, 6) for value in robot.jnt_pos_msr])
        print("final qd:", [round(value, 6) for value in robot.jnt_vel_msr])
        print("target q:", [round(value, 6) for value in TARGET])
        print("max torque:", [round(value, 3) for value in max_torque])
        print("max tracking error:", [round(value, 6) for value in max_hold_error])
    finally:
        if viewer is not None:
            viewer.close()
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
