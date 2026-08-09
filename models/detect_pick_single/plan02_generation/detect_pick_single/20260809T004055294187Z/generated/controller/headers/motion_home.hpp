/// Motion: home
/// Hold the TCP at the startup home pose
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_home_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_home_solver_state arm_solver_home;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_x;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_y;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_z;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_z;
};

inline void reset_motion_home(motion_home_state &state) {
    state = motion_home_state{};
}

inline void init_motion_home(motion_home_state &state, const robot_io &robot) {
    if (!state.arm_solver_home.initialized) {
        state.arm_solver_home.num_joints = robot.arm_solver_home.chain->getNrOfJoints();
        state.arm_solver_home.num_segments = robot.arm_solver_home.chain->getNrOfSegments();
        state.arm_solver_home.q = KDL::JntArray(state.arm_solver_home.num_joints);
        state.arm_solver_home.qd = KDL::JntArray(state.arm_solver_home.num_joints);
        state.arm_solver_home.qdd = KDL::JntArray(state.arm_solver_home.num_joints);
        state.arm_solver_home.tau_ff = KDL::JntArray(state.arm_solver_home.num_joints);
        state.arm_solver_home.tau_ctrl = KDL::JntArray(state.arm_solver_home.num_joints);
        state.arm_solver_home.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_home.num_spatial_directions = 6;
        state.arm_solver_home.spatial_directions = KDL::Jacobian(state.arm_solver_home.num_spatial_directions);
        state.arm_solver_home.acceleration_energy = KDL::JntArray(state.arm_solver_home.num_spatial_directions);
        state.arm_solver_home.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_home.chain, state.arm_solver_home.root_acc, state.arm_solver_home.num_spatial_directions);
        state.arm_solver_home.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_home.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_home.initialized = true;
    }
}

inline void update_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_home(state, robot);

    mj_kdl::update(robot.arm_solver_home.robot);
    for (int i = 0; i < state.arm_solver_home.num_joints; ++i) {
        state.arm_solver_home.q(i) = robot.arm_solver_home.robot->jnt_pos_msr[i];
        state.arm_solver_home.qd(i) = robot.arm_solver_home.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_home(state.arm_solver_home.q, state.arm_solver_home.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_home.chain);
        fk.JntToCart(
            state.arm_solver_home.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_home.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_home.chain);
        fk.JntToCart(
            state.arm_solver_home.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_home.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_home.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_home,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_home.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_home.robot->model,
                robot.arm_solver_home.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_home.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_home.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.home_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_home() {
    return true;
}

inline void monitor_when_motion_home() {
}

inline void monitor_until_motion_home() {
}

inline void monitor_motion_home() {
}

inline void control_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_hold_position
    shared.pose_diff_ctrl_hold_position = KDL::diff(shared.pose_ee_base, shared.home_pose);
    shared.ctrl_hold_position_err_lin_x = shared.pose_diff_ctrl_hold_position.vel[0];
    shared.ctrl_hold_position_err_lin_y = shared.pose_diff_ctrl_hold_position.vel[1];
    shared.ctrl_hold_position_err_lin_z = shared.pose_diff_ctrl_hold_position.vel[2];
    // eval_pose_diff_ctrl_hold_orientation
    shared.pose_diff_ctrl_hold_orientation = KDL::diff(shared.pose_ee_base, shared.home_pose);
    shared.ctrl_hold_orientation_err_ang_x = shared.pose_diff_ctrl_hold_orientation.rot[0];
    shared.ctrl_hold_orientation_err_ang_y = shared.pose_diff_ctrl_hold_orientation.rot[1];
    shared.ctrl_hold_orientation_err_ang_z = shared.pose_diff_ctrl_hold_orientation.rot[2];
    // ctrl_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_hold_orientation_ang_z.control(shared.pose_diff_ctrl_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_hold_orientation_ang_z_kp, shared.ctrl_hold_orientation_ang_z_ki, shared.ctrl_hold_orientation_ang_z_kd, shared.ctrl_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_hold_orientation_ang_z = _control_signal;
        shared.ctrl_hold_orientation_ang_z_error_integral = state.ctrl_hold_orientation_ang_z.error_integral();
        shared.ctrl_hold_orientation_ang_z_previous_error = state.ctrl_hold_orientation_ang_z.previous_error();
        shared.ctrl_hold_orientation_ang_z_first_sample = state.ctrl_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_hold_orientation_ang_y.control(shared.pose_diff_ctrl_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_hold_orientation_ang_y_kp, shared.ctrl_hold_orientation_ang_y_ki, shared.ctrl_hold_orientation_ang_y_kd, shared.ctrl_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_hold_orientation_ang_y = _control_signal;
        shared.ctrl_hold_orientation_ang_y_error_integral = state.ctrl_hold_orientation_ang_y.error_integral();
        shared.ctrl_hold_orientation_ang_y_previous_error = state.ctrl_hold_orientation_ang_y.previous_error();
        shared.ctrl_hold_orientation_ang_y_first_sample = state.ctrl_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_hold_orientation_ang_x.control(shared.pose_diff_ctrl_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_hold_orientation_ang_x_kp, shared.ctrl_hold_orientation_ang_x_ki, shared.ctrl_hold_orientation_ang_x_kd, shared.ctrl_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_hold_orientation_ang_x = _control_signal;
        shared.ctrl_hold_orientation_ang_x_error_integral = state.ctrl_hold_orientation_ang_x.error_integral();
        shared.ctrl_hold_orientation_ang_x_previous_error = state.ctrl_hold_orientation_ang_x.previous_error();
        shared.ctrl_hold_orientation_ang_x_first_sample = state.ctrl_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_hold_position_lin_z
    {
        const double _control_signal = state.ctrl_hold_position_lin_z.control(shared.pose_diff_ctrl_hold_position.vel[2], shared.dt_measured_s, {shared.ctrl_hold_position_lin_z_kp, shared.ctrl_hold_position_lin_z_ki, shared.ctrl_hold_position_lin_z_kd, shared.ctrl_hold_position_lin_z_decay_rate});
        shared.eacc_ctrl_hold_position_lin_z = _control_signal;
        shared.ctrl_hold_position_lin_z_error_integral = state.ctrl_hold_position_lin_z.error_integral();
        shared.ctrl_hold_position_lin_z_previous_error = state.ctrl_hold_position_lin_z.previous_error();
        shared.ctrl_hold_position_lin_z_first_sample = state.ctrl_hold_position_lin_z.is_first_sample();
    }
    // ctrl_hold_position_lin_y
    {
        const double _control_signal = state.ctrl_hold_position_lin_y.control(shared.pose_diff_ctrl_hold_position.vel[1], shared.dt_measured_s, {shared.ctrl_hold_position_lin_y_kp, shared.ctrl_hold_position_lin_y_ki, shared.ctrl_hold_position_lin_y_kd, shared.ctrl_hold_position_lin_y_decay_rate});
        shared.eacc_ctrl_hold_position_lin_y = _control_signal;
        shared.ctrl_hold_position_lin_y_error_integral = state.ctrl_hold_position_lin_y.error_integral();
        shared.ctrl_hold_position_lin_y_previous_error = state.ctrl_hold_position_lin_y.previous_error();
        shared.ctrl_hold_position_lin_y_first_sample = state.ctrl_hold_position_lin_y.is_first_sample();
    }
    // ctrl_hold_position_lin_x
    {
        const double _control_signal = state.ctrl_hold_position_lin_x.control(shared.pose_diff_ctrl_hold_position.vel[0], shared.dt_measured_s, {shared.ctrl_hold_position_lin_x_kp, shared.ctrl_hold_position_lin_x_ki, shared.ctrl_hold_position_lin_x_kd, shared.ctrl_hold_position_lin_x_decay_rate});
        shared.eacc_ctrl_hold_position_lin_x = _control_signal;
        shared.ctrl_hold_position_lin_x_error_integral = state.ctrl_hold_position_lin_x.error_integral();
        shared.ctrl_hold_position_lin_x_previous_error = state.ctrl_hold_position_lin_x.previous_error();
        shared.ctrl_hold_position_lin_x_first_sample = state.ctrl_hold_position_lin_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_home.spatial_directions);

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_home.acceleration_energy(0) = shared.eacc_ctrl_hold_position_lin_x;

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_home.acceleration_energy(1) = shared.eacc_ctrl_hold_position_lin_y;

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_home.acceleration_energy(2) = shared.eacc_ctrl_hold_position_lin_z;

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_home.acceleration_energy(3) = shared.eacc_ctrl_hold_orientation_ang_x;

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_home.acceleration_energy(4) = shared.eacc_ctrl_hold_orientation_ang_y;

    state.arm_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_home.acceleration_energy(5) = shared.eacc_ctrl_hold_orientation_ang_z;

    KDL::SetToZero(state.arm_solver_home.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_home(state.arm_solver_home.num_segments);
    for (int i = 0; i < state.arm_solver_home.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_home[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_home(state.arm_solver_home.num_joints);
    state.arm_solver_home.achd_acc->CartToJnt(
        state.arm_solver_home.q,
        state.arm_solver_home.qd,
        state.arm_solver_home.qdd,
        state.arm_solver_home.spatial_directions,
        state.arm_solver_home.acceleration_energy,
        f_ext_zero_arm_solver_home,
        state.arm_solver_home.tau_ff,
        tau_ctrl_acc_arm_solver_home);
    state.arm_solver_home.rnea->CartToJnt(
        state.arm_solver_home.q,
        state.arm_solver_home.qd,
        state.arm_solver_home.qdd,
        f_ext_zero_arm_solver_home,
        state.arm_solver_home.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_home.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_home.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_home.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_home.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_home.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_home.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_home.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_home.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_home.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_home.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_home.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_home.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_home.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_home.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_home.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_home.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_home.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_home.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_home.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_home.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_home.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_home.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_home.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_home.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_home.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_home.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_home.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_home.tau_ctrl(6);

}

inline void apply_motion_home(
    motion_home_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_home.num_joints; ++i) {
        robot.arm_solver_home.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_home.tau_ctrl(i), i);
    }

}
