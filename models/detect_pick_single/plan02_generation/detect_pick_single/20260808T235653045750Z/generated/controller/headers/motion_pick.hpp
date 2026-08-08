/// Motion: pick
/// Lower TCP straight down to the grasp height at the cube
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_pick_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_pick_solver_state arm_solver_pick;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pk_hold_x;
    motion_spec::runtime::PIDControl ctrl_pk_hold_y;
    motion_spec::runtime::PIDControl ctrl_pk_lower_z;
    motion_spec::runtime::PIDControl ctrl_pk_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pk_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pk_follow_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pk_support_z;
    bool mon_pick_ready_previous = false;
    bool mon_pick_ready_event_triggered = false;

};

inline void reset_motion_pick(motion_pick_state &state) {
    state = motion_pick_state{};
}

inline void init_motion_pick(motion_pick_state &state, const robot_io &robot) {
    if (!state.arm_solver_pick.initialized) {
        state.arm_solver_pick.num_joints = robot.arm_solver_pick.chain->getNrOfJoints();
        state.arm_solver_pick.num_segments = robot.arm_solver_pick.chain->getNrOfSegments();
        state.arm_solver_pick.q = KDL::JntArray(state.arm_solver_pick.num_joints);
        state.arm_solver_pick.qd = KDL::JntArray(state.arm_solver_pick.num_joints);
        state.arm_solver_pick.qdd = KDL::JntArray(state.arm_solver_pick.num_joints);
        state.arm_solver_pick.tau_ff = KDL::JntArray(state.arm_solver_pick.num_joints);
        state.arm_solver_pick.tau_ctrl = KDL::JntArray(state.arm_solver_pick.num_joints);
        state.arm_solver_pick.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_pick.num_spatial_directions = 6;
        state.arm_solver_pick.spatial_directions = KDL::Jacobian(state.arm_solver_pick.num_spatial_directions);
        state.arm_solver_pick.acceleration_energy = KDL::JntArray(state.arm_solver_pick.num_spatial_directions);
        state.arm_solver_pick.f_ext = KDL::Wrenches(state.arm_solver_pick.num_segments);
        state.arm_solver_pick.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver_pick.chain, state.arm_solver_pick.root_acc, state.arm_solver_pick.num_spatial_directions);
        state.arm_solver_pick.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_pick.chain, state.arm_solver_pick.root_acc, state.arm_solver_pick.num_spatial_directions);
        state.arm_solver_pick.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_pick.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_pick.initialized = true;
    }
}

inline void update_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_pick(state, robot);

    mj_kdl::update(robot.arm_solver_pick.robot);
    for (int i = 0; i < state.arm_solver_pick.num_joints; ++i) {
        state.arm_solver_pick.q(i) = robot.arm_solver_pick.robot->jnt_pos_msr[i];
        state.arm_solver_pick.qd(i) = robot.arm_solver_pick.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_pick(state.arm_solver_pick.q, state.arm_solver_pick.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_pick.chain);
        fk.JntToCart(
            state.arm_solver_pick.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_pick.chain);
        fk.JntToCart(
            state.arm_solver_pick.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_pick.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_pick,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_pick.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_pick.robot->model,
                robot.arm_solver_pick.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_pick.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_pick.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.grasp_x = shared.pose_cube_base.p[0];
        shared.grasp_y = shared.pose_cube_base.p[1];

        shared.pick_support_z_add_out = shared.pose_elbow_base.p[2] + shared.pick_support_lift;
        shared.pick_support_z = shared.pick_support_z_add_out;
        state.snapshot_taken = true;
    }
    shared.pick_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
}

inline bool can_start_motion_pick(
    shared_data &shared
) {
    // eval_pick_when_aligned_above
    shared.eval_pick_when_aligned_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal_pose.p, shared.pose_ee_base.p);

    return motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned_above_err, shared.satisfied_band);
}

inline void monitor_when_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.pick_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    // eval_pick_when_aligned_above
    shared.eval_pick_when_aligned_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal_pose.p, shared.pose_ee_base.p);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned_above_err, shared.satisfied_band);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(10);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_PICK_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_PICK_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_pick() {
}

inline void monitor_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_pick_when_aligned_above
    shared.eval_pick_when_aligned_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal_pose.p, shared.pose_ee_base.p);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned_above_err, shared.satisfied_band);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(10);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_PICK_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_PICK_READY] << std::endl;
        }
    }

}

inline void control_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.grasp_x;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.grasp_y;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.grasp_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();

        shared.pose_ee_base_distance_x_err_pick = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_pick = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_pick = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_pk_follow_ori
    shared.pose_diff_ctrl_pk_follow_ori = KDL::diff(shared.pose_ee_base, shared.pick_ori_pose);
    shared.ctrl_pk_follow_ori_err_ang_x = shared.pose_diff_ctrl_pk_follow_ori.rot[0];
    shared.ctrl_pk_follow_ori_err_ang_y = shared.pose_diff_ctrl_pk_follow_ori.rot[1];
    shared.ctrl_pk_follow_ori_err_ang_z = shared.pose_diff_ctrl_pk_follow_ori.rot[2];
    // eval_pick_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_pick = motion_spec::runtime::evaluate_equality_constraint(shared.pick_support_z, shared.pose_elbow_base.p[2]);
    // compute_wrench_force_ctrl_pk_support_z
    shared.wrench_force_ctrl_pk_support_z = KDL::Wrench(shared.direction_ctrl_pk_support_z * shared.force_ctrl_pk_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pk_support_z);
    // ctrl_pk_support_z
    {
        const double _control_signal = state.ctrl_pk_support_z.control(shared.pose_elbow_base_distance_z_err_pick, shared.dt_measured_s, {shared.ctrl_pk_support_z_stiffness, shared.ctrl_pk_support_z_damping, shared.ctrl_pk_support_z_integral_gain});
        shared.force_ctrl_pk_support_z = _control_signal;
        shared.ctrl_pk_support_z_error_integral = state.ctrl_pk_support_z.error_integral();
        shared.ctrl_pk_support_z_previous_error = state.ctrl_pk_support_z.previous_error();
        shared.ctrl_pk_support_z_first_sample = state.ctrl_pk_support_z.is_first_sample();
    }
    // ctrl_pk_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pk_follow_ori_ang_z.control(shared.pose_diff_ctrl_pk_follow_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pk_follow_ori_ang_z_kp, shared.ctrl_pk_follow_ori_ang_z_ki, shared.ctrl_pk_follow_ori_ang_z_kd, shared.ctrl_pk_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pk_follow_ori_ang_z = _control_signal;
        shared.ctrl_pk_follow_ori_ang_z_error_integral = state.ctrl_pk_follow_ori_ang_z.error_integral();
        shared.ctrl_pk_follow_ori_ang_z_previous_error = state.ctrl_pk_follow_ori_ang_z.previous_error();
        shared.ctrl_pk_follow_ori_ang_z_first_sample = state.ctrl_pk_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pk_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pk_follow_ori_ang_y.control(shared.pose_diff_ctrl_pk_follow_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pk_follow_ori_ang_y_kp, shared.ctrl_pk_follow_ori_ang_y_ki, shared.ctrl_pk_follow_ori_ang_y_kd, shared.ctrl_pk_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pk_follow_ori_ang_y = _control_signal;
        shared.ctrl_pk_follow_ori_ang_y_error_integral = state.ctrl_pk_follow_ori_ang_y.error_integral();
        shared.ctrl_pk_follow_ori_ang_y_previous_error = state.ctrl_pk_follow_ori_ang_y.previous_error();
        shared.ctrl_pk_follow_ori_ang_y_first_sample = state.ctrl_pk_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pk_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pk_follow_ori_ang_x.control(shared.pose_diff_ctrl_pk_follow_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pk_follow_ori_ang_x_kp, shared.ctrl_pk_follow_ori_ang_x_ki, shared.ctrl_pk_follow_ori_ang_x_kd, shared.ctrl_pk_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pk_follow_ori_ang_x = _control_signal;
        shared.ctrl_pk_follow_ori_ang_x_error_integral = state.ctrl_pk_follow_ori_ang_x.error_integral();
        shared.ctrl_pk_follow_ori_ang_x_previous_error = state.ctrl_pk_follow_ori_ang_x.previous_error();
        shared.ctrl_pk_follow_ori_ang_x_first_sample = state.ctrl_pk_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pk_lower_z
    {
        const double _control_signal = state.ctrl_pk_lower_z.control(shared.pose_ee_base_distance_z_err_pick, shared.dt_measured_s, {shared.ctrl_pk_lower_z_kp, shared.ctrl_pk_lower_z_ki, shared.ctrl_pk_lower_z_kd, shared.ctrl_pk_lower_z_decay_rate});
        shared.eacc_pose_ee_base_distance_z_pick = _control_signal;
        shared.ctrl_pk_lower_z_error_integral = state.ctrl_pk_lower_z.error_integral();
        shared.ctrl_pk_lower_z_previous_error = state.ctrl_pk_lower_z.previous_error();
        shared.ctrl_pk_lower_z_first_sample = state.ctrl_pk_lower_z.is_first_sample();
    }
    // ctrl_pk_hold_y
    {
        const double _control_signal = state.ctrl_pk_hold_y.control(shared.pose_ee_base_distance_y_err_pick, shared.dt_measured_s, {shared.ctrl_pk_hold_y_kp, shared.ctrl_pk_hold_y_ki, shared.ctrl_pk_hold_y_kd, shared.ctrl_pk_hold_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_pick = _control_signal;
        shared.ctrl_pk_hold_y_error_integral = state.ctrl_pk_hold_y.error_integral();
        shared.ctrl_pk_hold_y_previous_error = state.ctrl_pk_hold_y.previous_error();
        shared.ctrl_pk_hold_y_first_sample = state.ctrl_pk_hold_y.is_first_sample();
    }
    // ctrl_pk_hold_x
    {
        const double _control_signal = state.ctrl_pk_hold_x.control(shared.pose_ee_base_distance_x_err_pick, shared.dt_measured_s, {shared.ctrl_pk_hold_x_kp, shared.ctrl_pk_hold_x_ki, shared.ctrl_pk_hold_x_kd, shared.ctrl_pk_hold_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_pick = _control_signal;
        shared.ctrl_pk_hold_x_error_integral = state.ctrl_pk_hold_x.error_integral();
        shared.ctrl_pk_hold_x_previous_error = state.ctrl_pk_hold_x.previous_error();
        shared.ctrl_pk_hold_x_first_sample = state.ctrl_pk_hold_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_pick.spatial_directions);

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_pick.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_pick;

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_pick.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_pick;

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_pick.acceleration_energy(2) = shared.eacc_pose_ee_base_distance_z_pick;

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_pick.acceleration_energy(3) = shared.eacc_ctrl_pk_follow_ori_ang_x;

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_pick.acceleration_energy(4) = shared.eacc_ctrl_pk_follow_ori_ang_y;

    state.arm_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_pick.acceleration_energy(5) = shared.eacc_ctrl_pk_follow_ori_ang_z;

    KDL::SetToZero(state.arm_solver_pick.tau_ff);

    for (int i = 0; i < state.arm_solver_pick.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver_pick.f_ext[i]);
    }

    state.arm_solver_pick.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver_pick.chain, "half_arm_2_link", "base_link") - 1] += shared.wrench_force_ctrl_pk_support_z;

    KDL::Wrenches f_ext_zero_arm_solver_pick(state.arm_solver_pick.num_segments);
    for (int i = 0; i < state.arm_solver_pick.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_pick[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_pick(state.arm_solver_pick.num_joints);
    state.arm_solver_pick.achd_acc->CartToJnt(
        state.arm_solver_pick.q,
        state.arm_solver_pick.qd,
        state.arm_solver_pick.qdd,
        state.arm_solver_pick.spatial_directions,
        state.arm_solver_pick.acceleration_energy,
        state.arm_solver_pick.f_ext,
        state.arm_solver_pick.tau_ff,
        tau_ctrl_acc_arm_solver_pick);
    state.arm_solver_pick.rnea->CartToJnt(
        state.arm_solver_pick.q,
        state.arm_solver_pick.qd,
        state.arm_solver_pick.qdd,
        f_ext_zero_arm_solver_pick,
        state.arm_solver_pick.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_pick.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_pick.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_pick.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_pick.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_pick.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_pick.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_pick.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_pick.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_pick.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_pick.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_pick.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_pick.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_pick.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_pick.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_pick.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_pick.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_pick.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_pick.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_pick.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_pick.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_pick.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_pick.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_pick.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_pick.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_pick.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_pick.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_pick.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_pick.tau_ctrl(6);

}

inline void apply_motion_pick(
    motion_pick_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_pick.num_joints; ++i) {
        robot.arm_solver_pick.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_pick.tau_ctrl(i), i);
    }

}
