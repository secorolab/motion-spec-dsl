/// Motion: home
/// Hold both TCPs at their startup home poses
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_home_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_home_solver_state arm1_solver_home;
    arm2_solver_home_solver_state arm2_solver_home;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_h1_position_lin_x;
    motion_spec::runtime::PIDControl ctrl_h1_position_lin_y;
    motion_spec::runtime::PIDControl ctrl_h1_position_lin_z;
    motion_spec::runtime::PIDControl ctrl_h1_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_h1_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_h1_orientation_ang_z;
    motion_spec::runtime::PIDControl ctrl_h2_position_lin_x;
    motion_spec::runtime::PIDControl ctrl_h2_position_lin_y;
    motion_spec::runtime::PIDControl ctrl_h2_position_lin_z;
    motion_spec::runtime::PIDControl ctrl_h2_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_h2_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_h2_orientation_ang_z;
};

inline void reset_motion_home(motion_home_state &state) {
    state = motion_home_state{};
}

inline void init_motion_home(motion_home_state &state, const robot_io &robot) {
    if (!state.arm1_solver_home.initialized) {
        state.arm1_solver_home.num_joints = robot.arm1_solver_home.chain->getNrOfJoints();
        state.arm1_solver_home.num_segments = robot.arm1_solver_home.chain->getNrOfSegments();
        state.arm1_solver_home.q = KDL::JntArray(state.arm1_solver_home.num_joints);
        state.arm1_solver_home.qd = KDL::JntArray(state.arm1_solver_home.num_joints);
        state.arm1_solver_home.qdd = KDL::JntArray(state.arm1_solver_home.num_joints);
        state.arm1_solver_home.tau_ff = KDL::JntArray(state.arm1_solver_home.num_joints);
        state.arm1_solver_home.tau_ctrl = KDL::JntArray(state.arm1_solver_home.num_joints);
        state.arm1_solver_home.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_home.num_spatial_directions = 6;
        state.arm1_solver_home.spatial_directions = KDL::Jacobian(state.arm1_solver_home.num_spatial_directions);
        state.arm1_solver_home.acceleration_energy = KDL::JntArray(state.arm1_solver_home.num_spatial_directions);
        state.arm1_solver_home.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_home.chain, state.arm1_solver_home.root_acc, state.arm1_solver_home.num_spatial_directions);
        state.arm1_solver_home.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_home.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_home.initialized = true;
    }
    if (!state.arm2_solver_home.initialized) {
        state.arm2_solver_home.num_joints = robot.arm2_solver_home.chain->getNrOfJoints();
        state.arm2_solver_home.num_segments = robot.arm2_solver_home.chain->getNrOfSegments();
        state.arm2_solver_home.q = KDL::JntArray(state.arm2_solver_home.num_joints);
        state.arm2_solver_home.qd = KDL::JntArray(state.arm2_solver_home.num_joints);
        state.arm2_solver_home.qdd = KDL::JntArray(state.arm2_solver_home.num_joints);
        state.arm2_solver_home.tau_ff = KDL::JntArray(state.arm2_solver_home.num_joints);
        state.arm2_solver_home.tau_ctrl = KDL::JntArray(state.arm2_solver_home.num_joints);
        state.arm2_solver_home.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_home.num_spatial_directions = 6;
        state.arm2_solver_home.spatial_directions = KDL::Jacobian(state.arm2_solver_home.num_spatial_directions);
        state.arm2_solver_home.acceleration_energy = KDL::JntArray(state.arm2_solver_home.num_spatial_directions);
        state.arm2_solver_home.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_home.chain, state.arm2_solver_home.root_acc, state.arm2_solver_home.num_spatial_directions);
        state.arm2_solver_home.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_home.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_home.initialized = true;
    }
}

inline void update_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_home(state, robot);

    mj_kdl::update(robot.arm1_solver_home.robot);
    for (int i = 0; i < state.arm1_solver_home.num_joints; ++i) {
        state.arm1_solver_home.q(i) = robot.arm1_solver_home.robot->jnt_pos_msr[i];
        state.arm1_solver_home.qd(i) = robot.arm1_solver_home.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_home(state.arm1_solver_home.q, state.arm1_solver_home.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_home.robot->model,
                robot.arm1_solver_home.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_home;
        mj_kdl::get_body_frame(
                robot.arm1_solver_home.robot->model,
                robot.arm1_solver_home.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_home);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_home.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_home.chain);
        fk.JntToCart(
            state.arm1_solver_home.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_home.chain);
        fk.JntToCart(
            state.arm1_solver_home.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_home.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_home,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_home.robot->model,
                robot.arm1_solver_home.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_home.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_home.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_home.robot);
    for (int i = 0; i < state.arm2_solver_home.num_joints; ++i) {
        state.arm2_solver_home.q(i) = robot.arm2_solver_home.robot->jnt_pos_msr[i];
        state.arm2_solver_home.qd(i) = robot.arm2_solver_home.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_home(state.arm2_solver_home.q, state.arm2_solver_home.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_home.robot->model,
                robot.arm2_solver_home.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_home;
        mj_kdl::get_body_frame(
                robot.arm2_solver_home.robot->model,
                robot.arm2_solver_home.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_home);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_home.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_home.chain);
        fk.JntToCart(
            state.arm2_solver_home.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_home.chain);
        fk.JntToCart(
            state.arm2_solver_home.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_home.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_home,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_home.robot->model,
                robot.arm2_solver_home.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_home.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_home.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.home1_pose = shared.pose_ee1_base;
        shared.home2_pose = shared.pose_ee2_base;
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
    // eval_pose_diff_ctrl_h1_position
    shared.pose_diff_ctrl_h1_position = KDL::diff(shared.pose_ee1_base, shared.home1_pose);
    shared.ctrl_h1_position_err_lin_x = shared.pose_diff_ctrl_h1_position.vel[0];
    shared.ctrl_h1_position_err_lin_y = shared.pose_diff_ctrl_h1_position.vel[1];
    shared.ctrl_h1_position_err_lin_z = shared.pose_diff_ctrl_h1_position.vel[2];
    // eval_pose_diff_ctrl_h1_orientation
    shared.pose_diff_ctrl_h1_orientation = KDL::diff(shared.pose_ee1_base, shared.home1_pose);
    shared.ctrl_h1_orientation_err_ang_x = shared.pose_diff_ctrl_h1_orientation.rot[0];
    shared.ctrl_h1_orientation_err_ang_y = shared.pose_diff_ctrl_h1_orientation.rot[1];
    shared.ctrl_h1_orientation_err_ang_z = shared.pose_diff_ctrl_h1_orientation.rot[2];
    // eval_pose_diff_ctrl_h2_position
    shared.pose_diff_ctrl_h2_position = KDL::diff(shared.pose_ee2_base, shared.home2_pose);
    shared.ctrl_h2_position_err_lin_x = shared.pose_diff_ctrl_h2_position.vel[0];
    shared.ctrl_h2_position_err_lin_y = shared.pose_diff_ctrl_h2_position.vel[1];
    shared.ctrl_h2_position_err_lin_z = shared.pose_diff_ctrl_h2_position.vel[2];
    // eval_pose_diff_ctrl_h2_orientation
    shared.pose_diff_ctrl_h2_orientation = KDL::diff(shared.pose_ee2_base, shared.home2_pose);
    shared.ctrl_h2_orientation_err_ang_x = shared.pose_diff_ctrl_h2_orientation.rot[0];
    shared.ctrl_h2_orientation_err_ang_y = shared.pose_diff_ctrl_h2_orientation.rot[1];
    shared.ctrl_h2_orientation_err_ang_z = shared.pose_diff_ctrl_h2_orientation.rot[2];
    // ctrl_h2_orientation_ang_z
    {
        const double _control_signal = state.ctrl_h2_orientation_ang_z.control(shared.pose_diff_ctrl_h2_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_h2_orientation_ang_z_kp, shared.ctrl_h2_orientation_ang_z_ki, shared.ctrl_h2_orientation_ang_z_kd, shared.ctrl_h2_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_h2_orientation_ang_z = _control_signal;
        shared.ctrl_h2_orientation_ang_z_error_integral = state.ctrl_h2_orientation_ang_z.error_integral();
        shared.ctrl_h2_orientation_ang_z_previous_error = state.ctrl_h2_orientation_ang_z.previous_error();
        shared.ctrl_h2_orientation_ang_z_first_sample = state.ctrl_h2_orientation_ang_z.is_first_sample();
    }
    // ctrl_h2_orientation_ang_y
    {
        const double _control_signal = state.ctrl_h2_orientation_ang_y.control(shared.pose_diff_ctrl_h2_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_h2_orientation_ang_y_kp, shared.ctrl_h2_orientation_ang_y_ki, shared.ctrl_h2_orientation_ang_y_kd, shared.ctrl_h2_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_h2_orientation_ang_y = _control_signal;
        shared.ctrl_h2_orientation_ang_y_error_integral = state.ctrl_h2_orientation_ang_y.error_integral();
        shared.ctrl_h2_orientation_ang_y_previous_error = state.ctrl_h2_orientation_ang_y.previous_error();
        shared.ctrl_h2_orientation_ang_y_first_sample = state.ctrl_h2_orientation_ang_y.is_first_sample();
    }
    // ctrl_h2_orientation_ang_x
    {
        const double _control_signal = state.ctrl_h2_orientation_ang_x.control(shared.pose_diff_ctrl_h2_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_h2_orientation_ang_x_kp, shared.ctrl_h2_orientation_ang_x_ki, shared.ctrl_h2_orientation_ang_x_kd, shared.ctrl_h2_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_h2_orientation_ang_x = _control_signal;
        shared.ctrl_h2_orientation_ang_x_error_integral = state.ctrl_h2_orientation_ang_x.error_integral();
        shared.ctrl_h2_orientation_ang_x_previous_error = state.ctrl_h2_orientation_ang_x.previous_error();
        shared.ctrl_h2_orientation_ang_x_first_sample = state.ctrl_h2_orientation_ang_x.is_first_sample();
    }
    // ctrl_h2_position_lin_z
    {
        const double _control_signal = state.ctrl_h2_position_lin_z.control(shared.pose_diff_ctrl_h2_position.vel[2], shared.dt_measured_s, {shared.ctrl_h2_position_lin_z_kp, shared.ctrl_h2_position_lin_z_ki, shared.ctrl_h2_position_lin_z_kd, shared.ctrl_h2_position_lin_z_decay_rate});
        shared.eacc_ctrl_h2_position_lin_z = _control_signal;
        shared.ctrl_h2_position_lin_z_error_integral = state.ctrl_h2_position_lin_z.error_integral();
        shared.ctrl_h2_position_lin_z_previous_error = state.ctrl_h2_position_lin_z.previous_error();
        shared.ctrl_h2_position_lin_z_first_sample = state.ctrl_h2_position_lin_z.is_first_sample();
    }
    // ctrl_h2_position_lin_y
    {
        const double _control_signal = state.ctrl_h2_position_lin_y.control(shared.pose_diff_ctrl_h2_position.vel[1], shared.dt_measured_s, {shared.ctrl_h2_position_lin_y_kp, shared.ctrl_h2_position_lin_y_ki, shared.ctrl_h2_position_lin_y_kd, shared.ctrl_h2_position_lin_y_decay_rate});
        shared.eacc_ctrl_h2_position_lin_y = _control_signal;
        shared.ctrl_h2_position_lin_y_error_integral = state.ctrl_h2_position_lin_y.error_integral();
        shared.ctrl_h2_position_lin_y_previous_error = state.ctrl_h2_position_lin_y.previous_error();
        shared.ctrl_h2_position_lin_y_first_sample = state.ctrl_h2_position_lin_y.is_first_sample();
    }
    // ctrl_h2_position_lin_x
    {
        const double _control_signal = state.ctrl_h2_position_lin_x.control(shared.pose_diff_ctrl_h2_position.vel[0], shared.dt_measured_s, {shared.ctrl_h2_position_lin_x_kp, shared.ctrl_h2_position_lin_x_ki, shared.ctrl_h2_position_lin_x_kd, shared.ctrl_h2_position_lin_x_decay_rate});
        shared.eacc_ctrl_h2_position_lin_x = _control_signal;
        shared.ctrl_h2_position_lin_x_error_integral = state.ctrl_h2_position_lin_x.error_integral();
        shared.ctrl_h2_position_lin_x_previous_error = state.ctrl_h2_position_lin_x.previous_error();
        shared.ctrl_h2_position_lin_x_first_sample = state.ctrl_h2_position_lin_x.is_first_sample();
    }
    // ctrl_h1_orientation_ang_z
    {
        const double _control_signal = state.ctrl_h1_orientation_ang_z.control(shared.pose_diff_ctrl_h1_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_h1_orientation_ang_z_kp, shared.ctrl_h1_orientation_ang_z_ki, shared.ctrl_h1_orientation_ang_z_kd, shared.ctrl_h1_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_h1_orientation_ang_z = _control_signal;
        shared.ctrl_h1_orientation_ang_z_error_integral = state.ctrl_h1_orientation_ang_z.error_integral();
        shared.ctrl_h1_orientation_ang_z_previous_error = state.ctrl_h1_orientation_ang_z.previous_error();
        shared.ctrl_h1_orientation_ang_z_first_sample = state.ctrl_h1_orientation_ang_z.is_first_sample();
    }
    // ctrl_h1_orientation_ang_y
    {
        const double _control_signal = state.ctrl_h1_orientation_ang_y.control(shared.pose_diff_ctrl_h1_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_h1_orientation_ang_y_kp, shared.ctrl_h1_orientation_ang_y_ki, shared.ctrl_h1_orientation_ang_y_kd, shared.ctrl_h1_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_h1_orientation_ang_y = _control_signal;
        shared.ctrl_h1_orientation_ang_y_error_integral = state.ctrl_h1_orientation_ang_y.error_integral();
        shared.ctrl_h1_orientation_ang_y_previous_error = state.ctrl_h1_orientation_ang_y.previous_error();
        shared.ctrl_h1_orientation_ang_y_first_sample = state.ctrl_h1_orientation_ang_y.is_first_sample();
    }
    // ctrl_h1_orientation_ang_x
    {
        const double _control_signal = state.ctrl_h1_orientation_ang_x.control(shared.pose_diff_ctrl_h1_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_h1_orientation_ang_x_kp, shared.ctrl_h1_orientation_ang_x_ki, shared.ctrl_h1_orientation_ang_x_kd, shared.ctrl_h1_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_h1_orientation_ang_x = _control_signal;
        shared.ctrl_h1_orientation_ang_x_error_integral = state.ctrl_h1_orientation_ang_x.error_integral();
        shared.ctrl_h1_orientation_ang_x_previous_error = state.ctrl_h1_orientation_ang_x.previous_error();
        shared.ctrl_h1_orientation_ang_x_first_sample = state.ctrl_h1_orientation_ang_x.is_first_sample();
    }
    // ctrl_h1_position_lin_z
    {
        const double _control_signal = state.ctrl_h1_position_lin_z.control(shared.pose_diff_ctrl_h1_position.vel[2], shared.dt_measured_s, {shared.ctrl_h1_position_lin_z_kp, shared.ctrl_h1_position_lin_z_ki, shared.ctrl_h1_position_lin_z_kd, shared.ctrl_h1_position_lin_z_decay_rate});
        shared.eacc_ctrl_h1_position_lin_z = _control_signal;
        shared.ctrl_h1_position_lin_z_error_integral = state.ctrl_h1_position_lin_z.error_integral();
        shared.ctrl_h1_position_lin_z_previous_error = state.ctrl_h1_position_lin_z.previous_error();
        shared.ctrl_h1_position_lin_z_first_sample = state.ctrl_h1_position_lin_z.is_first_sample();
    }
    // ctrl_h1_position_lin_y
    {
        const double _control_signal = state.ctrl_h1_position_lin_y.control(shared.pose_diff_ctrl_h1_position.vel[1], shared.dt_measured_s, {shared.ctrl_h1_position_lin_y_kp, shared.ctrl_h1_position_lin_y_ki, shared.ctrl_h1_position_lin_y_kd, shared.ctrl_h1_position_lin_y_decay_rate});
        shared.eacc_ctrl_h1_position_lin_y = _control_signal;
        shared.ctrl_h1_position_lin_y_error_integral = state.ctrl_h1_position_lin_y.error_integral();
        shared.ctrl_h1_position_lin_y_previous_error = state.ctrl_h1_position_lin_y.previous_error();
        shared.ctrl_h1_position_lin_y_first_sample = state.ctrl_h1_position_lin_y.is_first_sample();
    }
    // ctrl_h1_position_lin_x
    {
        const double _control_signal = state.ctrl_h1_position_lin_x.control(shared.pose_diff_ctrl_h1_position.vel[0], shared.dt_measured_s, {shared.ctrl_h1_position_lin_x_kp, shared.ctrl_h1_position_lin_x_ki, shared.ctrl_h1_position_lin_x_kd, shared.ctrl_h1_position_lin_x_decay_rate});
        shared.eacc_ctrl_h1_position_lin_x = _control_signal;
        shared.ctrl_h1_position_lin_x_error_integral = state.ctrl_h1_position_lin_x.error_integral();
        shared.ctrl_h1_position_lin_x_previous_error = state.ctrl_h1_position_lin_x.previous_error();
        shared.ctrl_h1_position_lin_x_first_sample = state.ctrl_h1_position_lin_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_home.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_home_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_0(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_0.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_0 =
            alpha_frame_arm1_solver_home_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_home_0[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_home_0[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_home_0[2];
    }

    state.arm1_solver_home.acceleration_energy(0) = shared.eacc_ctrl_h1_position_lin_x;

    {
        KDL::Frame alpha_frame_arm1_solver_home_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_1(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_1.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_1 =
            alpha_frame_arm1_solver_home_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_home_1[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_home_1[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_home_1[2];
    }

    state.arm1_solver_home.acceleration_energy(1) = shared.eacc_ctrl_h1_position_lin_y;

    {
        KDL::Frame alpha_frame_arm1_solver_home_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_2(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_2.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_2 =
            alpha_frame_arm1_solver_home_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_home_2[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_home_2[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_home_2[2];
    }

    state.arm1_solver_home.acceleration_energy(2) = shared.eacc_ctrl_h1_position_lin_z;

    {
        KDL::Frame alpha_frame_arm1_solver_home_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_3(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_3.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_3 =
            alpha_frame_arm1_solver_home_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_home_3[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_home_3[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_home_3[2];
    }

    state.arm1_solver_home.acceleration_energy(3) = shared.eacc_ctrl_h1_orientation_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_home_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_4(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_4.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_4 =
            alpha_frame_arm1_solver_home_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_home_4[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_home_4[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_home_4[2];
    }

    state.arm1_solver_home.acceleration_energy(4) = shared.eacc_ctrl_h1_orientation_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_home_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_home_5(*robot.arm1_solver_home.chain);
        alpha_fk_arm1_solver_home_5.JntToCart(
            state.arm1_solver_home.q,
            alpha_frame_arm1_solver_home_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_home.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_home_5 =
            alpha_frame_arm1_solver_home_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_home_5[0];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_home_5[1];
        state.arm1_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_home_5[2];
    }

    state.arm1_solver_home.acceleration_energy(5) = shared.eacc_ctrl_h1_orientation_ang_z;

    KDL::SetToZero(state.arm1_solver_home.tau_ff);

    KDL::Wrenches f_ext_zero_arm1_solver_home(state.arm1_solver_home.num_segments);
    for (int i = 0; i < state.arm1_solver_home.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_home[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_home(state.arm1_solver_home.num_joints);
    state.arm1_solver_home.achd_acc->CartToJnt(
        state.arm1_solver_home.q,
        state.arm1_solver_home.qd,
        state.arm1_solver_home.qdd,
        state.arm1_solver_home.spatial_directions,
        state.arm1_solver_home.acceleration_energy,
        f_ext_zero_arm1_solver_home,
        state.arm1_solver_home.tau_ff,
        tau_ctrl_acc_arm1_solver_home);
    state.arm1_solver_home.rnea->CartToJnt(
        state.arm1_solver_home.q,
        state.arm1_solver_home.qd,
        state.arm1_solver_home.qdd,
        f_ext_zero_arm1_solver_home,
        state.arm1_solver_home.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_home.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_home.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_home.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_home.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_home.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_home.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_home.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_home.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_home.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_home.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_home.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_home.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_home.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_home.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_home.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_home.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_home.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_home.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_home.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_home.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_home.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_home.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_home.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_home.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_home.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_home.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_home.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_home.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_home.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_home_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_0(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_0.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_0 =
            alpha_frame_arm2_solver_home_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_home_0[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_home_0[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_home_0[2];
    }

    state.arm2_solver_home.acceleration_energy(0) = shared.eacc_ctrl_h2_position_lin_x;

    {
        KDL::Frame alpha_frame_arm2_solver_home_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_1(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_1.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_1 =
            alpha_frame_arm2_solver_home_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_home_1[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_home_1[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_home_1[2];
    }

    state.arm2_solver_home.acceleration_energy(1) = shared.eacc_ctrl_h2_position_lin_y;

    {
        KDL::Frame alpha_frame_arm2_solver_home_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_2(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_2.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_2 =
            alpha_frame_arm2_solver_home_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_home_2[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_home_2[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_home_2[2];
    }

    state.arm2_solver_home.acceleration_energy(2) = shared.eacc_ctrl_h2_position_lin_z;

    {
        KDL::Frame alpha_frame_arm2_solver_home_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_3(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_3.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_3 =
            alpha_frame_arm2_solver_home_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_home_3[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_home_3[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_home_3[2];
    }

    state.arm2_solver_home.acceleration_energy(3) = shared.eacc_ctrl_h2_orientation_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_home_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_4(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_4.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_4 =
            alpha_frame_arm2_solver_home_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_home_4[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_home_4[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_home_4[2];
    }

    state.arm2_solver_home.acceleration_energy(4) = shared.eacc_ctrl_h2_orientation_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_home_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_home_5(*robot.arm2_solver_home.chain);
        alpha_fk_arm2_solver_home_5.JntToCart(
            state.arm2_solver_home.q,
            alpha_frame_arm2_solver_home_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_home.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_home_5 =
            alpha_frame_arm2_solver_home_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_home_5[0];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_home_5[1];
        state.arm2_solver_home.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_home_5[2];
    }

    state.arm2_solver_home.acceleration_energy(5) = shared.eacc_ctrl_h2_orientation_ang_z;

    KDL::SetToZero(state.arm2_solver_home.tau_ff);

    KDL::Wrenches f_ext_zero_arm2_solver_home(state.arm2_solver_home.num_segments);
    for (int i = 0; i < state.arm2_solver_home.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_home[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_home(state.arm2_solver_home.num_joints);
    state.arm2_solver_home.achd_acc->CartToJnt(
        state.arm2_solver_home.q,
        state.arm2_solver_home.qd,
        state.arm2_solver_home.qdd,
        state.arm2_solver_home.spatial_directions,
        state.arm2_solver_home.acceleration_energy,
        f_ext_zero_arm2_solver_home,
        state.arm2_solver_home.tau_ff,
        tau_ctrl_acc_arm2_solver_home);
    state.arm2_solver_home.rnea->CartToJnt(
        state.arm2_solver_home.q,
        state.arm2_solver_home.qd,
        state.arm2_solver_home.qdd,
        f_ext_zero_arm2_solver_home,
        state.arm2_solver_home.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_home.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_home.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_home.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_home.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_home.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_home.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_home.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_home.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_home.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_home.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_home.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_home.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_home.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_home.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_home.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_home.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_home.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_home.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_home.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_home.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_home.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_home.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_home.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_home.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_home.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_home.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_home.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_home.tau_ctrl(6);

}

inline void apply_motion_home(
    motion_home_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_home.num_joints; ++i) {
        robot.arm1_solver_home.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_home.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_home.num_joints; ++i) {
        robot.arm2_solver_home.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_home.tau_ctrl(i), i);
    }

}
