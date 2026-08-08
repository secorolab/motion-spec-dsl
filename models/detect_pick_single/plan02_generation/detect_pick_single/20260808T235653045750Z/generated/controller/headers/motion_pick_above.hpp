/// Motion: pick-above
/// Move TCP to the pre-grasp position above the cube along a path
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_pick_above_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_pick_above_solver_state arm_solver_pick_above;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pa_follow_tan;
    motion_spec::runtime::PIDControl ctrl_pa_follow_lat_lin_normal_a;
    motion_spec::runtime::PIDControl ctrl_pa_follow_lat_lin_normal_b;
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_z;
    bool mon_pa_advancing_pa_advancing = false;
};

inline void reset_motion_pick_above(motion_pick_above_state &state) {
    state = motion_pick_above_state{};
}

inline void init_motion_pick_above(motion_pick_above_state &state, const robot_io &robot) {
    if (!state.arm_solver_pick_above.initialized) {
        state.arm_solver_pick_above.num_joints = robot.arm_solver_pick_above.chain->getNrOfJoints();
        state.arm_solver_pick_above.num_segments = robot.arm_solver_pick_above.chain->getNrOfSegments();
        state.arm_solver_pick_above.q = KDL::JntArray(state.arm_solver_pick_above.num_joints);
        state.arm_solver_pick_above.qd = KDL::JntArray(state.arm_solver_pick_above.num_joints);
        state.arm_solver_pick_above.qdd = KDL::JntArray(state.arm_solver_pick_above.num_joints);
        state.arm_solver_pick_above.tau_ff = KDL::JntArray(state.arm_solver_pick_above.num_joints);
        state.arm_solver_pick_above.tau_ctrl = KDL::JntArray(state.arm_solver_pick_above.num_joints);
        state.arm_solver_pick_above.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_pick_above.num_spatial_directions = 6;
        state.arm_solver_pick_above.spatial_directions = KDL::Jacobian(state.arm_solver_pick_above.num_spatial_directions);
        state.arm_solver_pick_above.acceleration_energy = KDL::JntArray(state.arm_solver_pick_above.num_spatial_directions);
        state.arm_solver_pick_above.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_pick_above.chain, state.arm_solver_pick_above.root_acc, state.arm_solver_pick_above.num_spatial_directions);
        state.arm_solver_pick_above.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_pick_above.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_pick_above.initialized = true;
    }
}

inline void update_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_pick_above(state, robot);

    mj_kdl::update(robot.arm_solver_pick_above.robot);
    for (int i = 0; i < state.arm_solver_pick_above.num_joints; ++i) {
        state.arm_solver_pick_above.q(i) = robot.arm_solver_pick_above.robot->jnt_pos_msr[i];
        state.arm_solver_pick_above.qd(i) = robot.arm_solver_pick_above.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_pick_above(state.arm_solver_pick_above.q, state.arm_solver_pick_above.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_pick_above.chain);
        fk.JntToCart(
            state.arm_solver_pick_above.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick_above.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_pick_above.chain);
        fk.JntToCart(
            state.arm_solver_pick_above.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick_above.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_pick_above.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_pick_above,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick_above.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_pick_above.robot->model,
                robot.arm_solver_pick_above.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_pick_above.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_pick_above.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.start_cube_x = shared.pose_cube_base.p[0];
        shared.start_cube_y = shared.pose_cube_base.p[1];
        shared.start_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
    shared.goal_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start_cube_x, shared.start_cube_y, 0.26));
}

inline bool can_start_motion_pick_above() {
    return true;
}

inline void monitor_when_motion_pick_above(
    shared_data &shared
) {
    shared.goal_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start_cube_x, shared.start_cube_y, 0.26));
}

inline void monitor_until_motion_pick_above() {
}

inline void monitor_motion_pick_above() {
}

inline void control_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // projection_approach_path
    {
        const KDL::Frame _goal = KDL::Frame(motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))), KDL::Vector(shared.start_cube_x, shared.start_cube_y, 0.26));
        shared.goal_pose = _goal;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start_pose, KDL::diff(shared.start_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.approach_path_s = motion_spec::runtime::path_project(
            _path_eval, shared.pose_ee_base.p, shared.approach_path_s);
    }
    // frame_approach_path
    {
        const KDL::Frame _goal = shared.goal_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start_pose, KDL::diff(shared.start_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        motion_spec::runtime::path_frame(
            _path_eval, shared.approach_path_s,
            shared.approach_path_tangent, shared.approach_path_normal_a, shared.approach_path_normal_b);
    }
    // along_approach_path
    shared.approach_path_along_speed = KDL::dot(shared.twist_ee_base.vel, shared.approach_path_tangent);
    // evaluator_approach_path
    {
        const KDL::Frame _goal = shared.goal_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start_pose, KDL::diff(shared.start_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.reference = _path_eval(shared.approach_path_s);
    }
    // eval_pose_diff_ctrl_pa_follow_lat
    shared.pose_diff_ctrl_pa_follow_lat = KDL::diff(shared.pose_ee_base, shared.reference);
    shared.ctrl_pa_follow_lat_err_lin_normal_a = KDL::dot(shared.pose_diff_ctrl_pa_follow_lat.vel, shared.approach_path_normal_a);
    shared.ctrl_pa_follow_lat_err_lin_normal_b = KDL::dot(shared.pose_diff_ctrl_pa_follow_lat.vel, shared.approach_path_normal_b);
    // eval_pose_diff_ctrl_pa_follow_ori
    shared.pose_diff_ctrl_pa_follow_ori = KDL::diff(shared.pose_ee_base, shared.reference);
    shared.ctrl_pa_follow_ori_err_ang_x = shared.pose_diff_ctrl_pa_follow_ori.rot[0];
    shared.ctrl_pa_follow_ori_err_ang_y = shared.pose_diff_ctrl_pa_follow_ori.rot[1];
    shared.ctrl_pa_follow_ori_err_ang_z = shared.pose_diff_ctrl_pa_follow_ori.rot[2];
    // eval_pick_above_while_follow_tan
    shared.approach_path_along_speed_err_pick_above = motion_spec::runtime::evaluate_equality_constraint(shared.approach_speed, shared.approach_path_along_speed);
    // eval_pick_above_while_advance
    shared.eval_pick_above_while_advance_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.approach_path_along_speed, shared.min_approach_speed);
    // ctrl_pa_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pa_follow_ori_ang_z.control(shared.pose_diff_ctrl_pa_follow_ori.rot[2], -(shared.twist_ee_base.rot[2]), shared.dt_measured_s, {shared.ctrl_pa_follow_ori_ang_z_kp, shared.ctrl_pa_follow_ori_ang_z_ki, shared.ctrl_pa_follow_ori_ang_z_kd, shared.ctrl_pa_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pa_follow_ori_ang_z = _control_signal;
        shared.ctrl_pa_follow_ori_ang_z_error_integral = state.ctrl_pa_follow_ori_ang_z.error_integral();
        shared.ctrl_pa_follow_ori_ang_z_previous_error = state.ctrl_pa_follow_ori_ang_z.previous_error();
        shared.ctrl_pa_follow_ori_ang_z_first_sample = state.ctrl_pa_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pa_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pa_follow_ori_ang_y.control(shared.pose_diff_ctrl_pa_follow_ori.rot[1], -(shared.twist_ee_base.rot[1]), shared.dt_measured_s, {shared.ctrl_pa_follow_ori_ang_y_kp, shared.ctrl_pa_follow_ori_ang_y_ki, shared.ctrl_pa_follow_ori_ang_y_kd, shared.ctrl_pa_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pa_follow_ori_ang_y = _control_signal;
        shared.ctrl_pa_follow_ori_ang_y_error_integral = state.ctrl_pa_follow_ori_ang_y.error_integral();
        shared.ctrl_pa_follow_ori_ang_y_previous_error = state.ctrl_pa_follow_ori_ang_y.previous_error();
        shared.ctrl_pa_follow_ori_ang_y_first_sample = state.ctrl_pa_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pa_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pa_follow_ori_ang_x.control(shared.pose_diff_ctrl_pa_follow_ori.rot[0], -(shared.twist_ee_base.rot[0]), shared.dt_measured_s, {shared.ctrl_pa_follow_ori_ang_x_kp, shared.ctrl_pa_follow_ori_ang_x_ki, shared.ctrl_pa_follow_ori_ang_x_kd, shared.ctrl_pa_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pa_follow_ori_ang_x = _control_signal;
        shared.ctrl_pa_follow_ori_ang_x_error_integral = state.ctrl_pa_follow_ori_ang_x.error_integral();
        shared.ctrl_pa_follow_ori_ang_x_previous_error = state.ctrl_pa_follow_ori_ang_x.previous_error();
        shared.ctrl_pa_follow_ori_ang_x_first_sample = state.ctrl_pa_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pa_follow_lat_lin_normal_b
    {
        const double _control_signal = state.ctrl_pa_follow_lat_lin_normal_b.control(KDL::dot(shared.pose_diff_ctrl_pa_follow_lat.vel, shared.approach_path_normal_b), -(KDL::dot(shared.twist_ee_base.vel, shared.approach_path_normal_b)), shared.dt_measured_s, {shared.ctrl_pa_follow_lat_lin_normal_b_kp, shared.ctrl_pa_follow_lat_lin_normal_b_ki, shared.ctrl_pa_follow_lat_lin_normal_b_kd, shared.ctrl_pa_follow_lat_lin_normal_b_decay_rate});
        shared.eacc_ctrl_pa_follow_lat_lin_normal_b = _control_signal;
        shared.ctrl_pa_follow_lat_lin_normal_b_error_integral = state.ctrl_pa_follow_lat_lin_normal_b.error_integral();
        shared.ctrl_pa_follow_lat_lin_normal_b_previous_error = state.ctrl_pa_follow_lat_lin_normal_b.previous_error();
        shared.ctrl_pa_follow_lat_lin_normal_b_first_sample = state.ctrl_pa_follow_lat_lin_normal_b.is_first_sample();
    }
    // ctrl_pa_follow_lat_lin_normal_a
    {
        const double _control_signal = state.ctrl_pa_follow_lat_lin_normal_a.control(KDL::dot(shared.pose_diff_ctrl_pa_follow_lat.vel, shared.approach_path_normal_a), -(KDL::dot(shared.twist_ee_base.vel, shared.approach_path_normal_a)), shared.dt_measured_s, {shared.ctrl_pa_follow_lat_lin_normal_a_kp, shared.ctrl_pa_follow_lat_lin_normal_a_ki, shared.ctrl_pa_follow_lat_lin_normal_a_kd, shared.ctrl_pa_follow_lat_lin_normal_a_decay_rate});
        shared.eacc_ctrl_pa_follow_lat_lin_normal_a = _control_signal;
        shared.ctrl_pa_follow_lat_lin_normal_a_error_integral = state.ctrl_pa_follow_lat_lin_normal_a.error_integral();
        shared.ctrl_pa_follow_lat_lin_normal_a_previous_error = state.ctrl_pa_follow_lat_lin_normal_a.previous_error();
        shared.ctrl_pa_follow_lat_lin_normal_a_first_sample = state.ctrl_pa_follow_lat_lin_normal_a.is_first_sample();
    }
    // ctrl_pa_follow_tan
    {
        const double _control_signal = state.ctrl_pa_follow_tan.control(shared.approach_path_along_speed_err_pick_above, shared.dt_measured_s, {shared.ctrl_pa_follow_tan_kp, shared.ctrl_pa_follow_tan_ki, shared.ctrl_pa_follow_tan_kd, shared.ctrl_pa_follow_tan_decay_rate});
        shared.eacc_approach_path_along_speed_pick_above = _control_signal;
        shared.ctrl_pa_follow_tan_error_integral = state.ctrl_pa_follow_tan.error_integral();
        shared.ctrl_pa_follow_tan_previous_error = state.ctrl_pa_follow_tan.previous_error();
        shared.ctrl_pa_follow_tan_first_sample = state.ctrl_pa_follow_tan.is_first_sample();
    }

    motion_spec::runtime::set_flag(state.mon_pa_advancing_pa_advancing, motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_while_advance_err, 0.0));

    KDL::SetToZero(state.arm_solver_pick_above.spatial_directions);

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = shared.approach_path_tangent.x();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = shared.approach_path_tangent.y();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = shared.approach_path_tangent.z();

    state.arm_solver_pick_above.acceleration_energy(0) = shared.eacc_approach_path_along_speed_pick_above;

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = shared.approach_path_normal_a.x();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = shared.approach_path_normal_a.y();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = shared.approach_path_normal_a.z();

    state.arm_solver_pick_above.acceleration_energy(1) = shared.eacc_ctrl_pa_follow_lat_lin_normal_a;

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = shared.approach_path_normal_b.x();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = shared.approach_path_normal_b.y();
    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = shared.approach_path_normal_b.z();

    state.arm_solver_pick_above.acceleration_energy(2) = shared.eacc_ctrl_pa_follow_lat_lin_normal_b;

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_pick_above.acceleration_energy(3) = shared.eacc_ctrl_pa_follow_ori_ang_x;

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_pick_above.acceleration_energy(4) = shared.eacc_ctrl_pa_follow_ori_ang_y;

    state.arm_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_pick_above.acceleration_energy(5) = shared.eacc_ctrl_pa_follow_ori_ang_z;

    KDL::SetToZero(state.arm_solver_pick_above.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_pick_above(state.arm_solver_pick_above.num_segments);
    for (int i = 0; i < state.arm_solver_pick_above.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_pick_above[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_pick_above(state.arm_solver_pick_above.num_joints);
    state.arm_solver_pick_above.achd_acc->CartToJnt(
        state.arm_solver_pick_above.q,
        state.arm_solver_pick_above.qd,
        state.arm_solver_pick_above.qdd,
        state.arm_solver_pick_above.spatial_directions,
        state.arm_solver_pick_above.acceleration_energy,
        f_ext_zero_arm_solver_pick_above,
        state.arm_solver_pick_above.tau_ff,
        tau_ctrl_acc_arm_solver_pick_above);
    state.arm_solver_pick_above.rnea->CartToJnt(
        state.arm_solver_pick_above.q,
        state.arm_solver_pick_above.qd,
        state.arm_solver_pick_above.qdd,
        f_ext_zero_arm_solver_pick_above,
        state.arm_solver_pick_above.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_pick_above.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_pick_above.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_pick_above.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_pick_above.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_pick_above.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_pick_above.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_pick_above.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_pick_above.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_pick_above.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_pick_above.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_pick_above.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_pick_above.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_pick_above.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_pick_above.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_pick_above.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_pick_above.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_pick_above.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_pick_above.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_pick_above.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_pick_above.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_pick_above.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_pick_above.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_pick_above.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_pick_above.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_pick_above.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_pick_above.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_pick_above.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_pick_above.tau_ctrl(6);

}

inline void apply_motion_pick_above(
    motion_pick_above_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_pick_above.num_joints; ++i) {
        robot.arm_solver_pick_above.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_pick_above.tau_ctrl(i), i);
    }

}
