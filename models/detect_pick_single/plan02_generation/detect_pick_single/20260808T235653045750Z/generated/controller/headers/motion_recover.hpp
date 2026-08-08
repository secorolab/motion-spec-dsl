/// Motion: recover
/// Fallback recovery: capture the current pose on entry and hold it
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_recover_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_recover_solver_state arm_solver_recover;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_rc_hold_x;
    motion_spec::runtime::PIDControl ctrl_rc_hold_y;
    motion_spec::runtime::PIDControl ctrl_rc_hold_z;
    motion_spec::runtime::PIDControl ctrl_rc_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_rc_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_rc_hold_orientation_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_rc_support_z;
    bool mon_recover_hold_previous = false;
    bool mon_recover_hold_event_triggered = false;
    double mon_recover_hold_hold_s = 0.0;

};

inline void reset_motion_recover(motion_recover_state &state) {
    state = motion_recover_state{};
}

inline void init_motion_recover(motion_recover_state &state, const robot_io &robot) {
    if (!state.arm_solver_recover.initialized) {
        state.arm_solver_recover.num_joints = robot.arm_solver_recover.chain->getNrOfJoints();
        state.arm_solver_recover.num_segments = robot.arm_solver_recover.chain->getNrOfSegments();
        state.arm_solver_recover.q = KDL::JntArray(state.arm_solver_recover.num_joints);
        state.arm_solver_recover.qd = KDL::JntArray(state.arm_solver_recover.num_joints);
        state.arm_solver_recover.qdd = KDL::JntArray(state.arm_solver_recover.num_joints);
        state.arm_solver_recover.tau_ff = KDL::JntArray(state.arm_solver_recover.num_joints);
        state.arm_solver_recover.tau_ctrl = KDL::JntArray(state.arm_solver_recover.num_joints);
        state.arm_solver_recover.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_recover.num_spatial_directions = 6;
        state.arm_solver_recover.spatial_directions = KDL::Jacobian(state.arm_solver_recover.num_spatial_directions);
        state.arm_solver_recover.acceleration_energy = KDL::JntArray(state.arm_solver_recover.num_spatial_directions);
        state.arm_solver_recover.f_ext = KDL::Wrenches(state.arm_solver_recover.num_segments);
        state.arm_solver_recover.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver_recover.chain, state.arm_solver_recover.root_acc, state.arm_solver_recover.num_spatial_directions);
        state.arm_solver_recover.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_recover.chain, state.arm_solver_recover.root_acc, state.arm_solver_recover.num_spatial_directions);
        state.arm_solver_recover.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_recover.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_recover.initialized = true;
    }
}

inline void update_motion_recover(
    motion_recover_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_recover(state, robot);

    mj_kdl::update(robot.arm_solver_recover.robot);
    for (int i = 0; i < state.arm_solver_recover.num_joints; ++i) {
        state.arm_solver_recover.q(i) = robot.arm_solver_recover.robot->jnt_pos_msr[i];
        state.arm_solver_recover.qd(i) = robot.arm_solver_recover.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_recover(state.arm_solver_recover.q, state.arm_solver_recover.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_recover.chain);
        fk.JntToCart(
            state.arm_solver_recover.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_recover.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_recover.chain);
        fk.JntToCart(
            state.arm_solver_recover.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_recover.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_recover.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_recover,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_recover.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_recover.robot->model,
                robot.arm_solver_recover.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_recover.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_recover.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.recover_hold_orientation_pose = shared.pose_ee_base;

        shared.recover_support_z_add_out = shared.pose_elbow_base.p[2] + shared.recover_support_lift;
        shared.recover_support_z = shared.recover_support_z_add_out;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_recover() {
    return true;
}

inline void monitor_when_motion_recover() {
}

inline void monitor_until_motion_recover() {
}

inline void monitor_motion_recover() {
}

inline void control_motion_recover(
    motion_recover_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.recover_hold_orientation_pose.p[0];
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.recover_hold_orientation_pose.p[1];
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.recover_hold_orientation_pose.p[2];
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();

        shared.pose_ee_base_distance_x_err_recover = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_recover = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_recover = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_rc_hold_orientation
    shared.pose_diff_ctrl_rc_hold_orientation = KDL::diff(shared.pose_ee_base, shared.recover_hold_orientation_pose);
    shared.ctrl_rc_hold_orientation_err_ang_x = shared.pose_diff_ctrl_rc_hold_orientation.rot[0];
    shared.ctrl_rc_hold_orientation_err_ang_y = shared.pose_diff_ctrl_rc_hold_orientation.rot[1];
    shared.ctrl_rc_hold_orientation_err_ang_z = shared.pose_diff_ctrl_rc_hold_orientation.rot[2];
    // eval_recover_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_recover = motion_spec::runtime::evaluate_equality_constraint(shared.recover_support_z, shared.pose_elbow_base.p[2]);
    // compute_wrench_force_ctrl_rc_support_z
    shared.wrench_force_ctrl_rc_support_z = KDL::Wrench(shared.direction_ctrl_rc_support_z * shared.force_ctrl_rc_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_rc_support_z);
    // ctrl_rc_support_z
    {
        const double _control_signal = state.ctrl_rc_support_z.control(shared.pose_elbow_base_distance_z_err_recover, shared.dt_measured_s, {shared.ctrl_rc_support_z_stiffness, shared.ctrl_rc_support_z_damping, shared.ctrl_rc_support_z_integral_gain});
        shared.force_ctrl_rc_support_z = _control_signal;
        shared.ctrl_rc_support_z_error_integral = state.ctrl_rc_support_z.error_integral();
        shared.ctrl_rc_support_z_previous_error = state.ctrl_rc_support_z.previous_error();
        shared.ctrl_rc_support_z_first_sample = state.ctrl_rc_support_z.is_first_sample();
    }
    // ctrl_rc_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_rc_hold_orientation_ang_z.control(shared.pose_diff_ctrl_rc_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_rc_hold_orientation_ang_z_kp, shared.ctrl_rc_hold_orientation_ang_z_ki, shared.ctrl_rc_hold_orientation_ang_z_kd, shared.ctrl_rc_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_rc_hold_orientation_ang_z = _control_signal;
        shared.ctrl_rc_hold_orientation_ang_z_error_integral = state.ctrl_rc_hold_orientation_ang_z.error_integral();
        shared.ctrl_rc_hold_orientation_ang_z_previous_error = state.ctrl_rc_hold_orientation_ang_z.previous_error();
        shared.ctrl_rc_hold_orientation_ang_z_first_sample = state.ctrl_rc_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_rc_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_rc_hold_orientation_ang_y.control(shared.pose_diff_ctrl_rc_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_rc_hold_orientation_ang_y_kp, shared.ctrl_rc_hold_orientation_ang_y_ki, shared.ctrl_rc_hold_orientation_ang_y_kd, shared.ctrl_rc_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_rc_hold_orientation_ang_y = _control_signal;
        shared.ctrl_rc_hold_orientation_ang_y_error_integral = state.ctrl_rc_hold_orientation_ang_y.error_integral();
        shared.ctrl_rc_hold_orientation_ang_y_previous_error = state.ctrl_rc_hold_orientation_ang_y.previous_error();
        shared.ctrl_rc_hold_orientation_ang_y_first_sample = state.ctrl_rc_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_rc_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_rc_hold_orientation_ang_x.control(shared.pose_diff_ctrl_rc_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_rc_hold_orientation_ang_x_kp, shared.ctrl_rc_hold_orientation_ang_x_ki, shared.ctrl_rc_hold_orientation_ang_x_kd, shared.ctrl_rc_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_rc_hold_orientation_ang_x = _control_signal;
        shared.ctrl_rc_hold_orientation_ang_x_error_integral = state.ctrl_rc_hold_orientation_ang_x.error_integral();
        shared.ctrl_rc_hold_orientation_ang_x_previous_error = state.ctrl_rc_hold_orientation_ang_x.previous_error();
        shared.ctrl_rc_hold_orientation_ang_x_first_sample = state.ctrl_rc_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_rc_hold_z
    {
        const double _control_signal = state.ctrl_rc_hold_z.control(shared.pose_ee_base_distance_z_err_recover, shared.dt_measured_s, {shared.ctrl_rc_hold_z_kp, shared.ctrl_rc_hold_z_ki, shared.ctrl_rc_hold_z_kd, shared.ctrl_rc_hold_z_decay_rate});
        shared.eacc_pose_ee_base_distance_z_recover = _control_signal;
        shared.ctrl_rc_hold_z_error_integral = state.ctrl_rc_hold_z.error_integral();
        shared.ctrl_rc_hold_z_previous_error = state.ctrl_rc_hold_z.previous_error();
        shared.ctrl_rc_hold_z_first_sample = state.ctrl_rc_hold_z.is_first_sample();
    }
    // ctrl_rc_hold_y
    {
        const double _control_signal = state.ctrl_rc_hold_y.control(shared.pose_ee_base_distance_y_err_recover, shared.dt_measured_s, {shared.ctrl_rc_hold_y_kp, shared.ctrl_rc_hold_y_ki, shared.ctrl_rc_hold_y_kd, shared.ctrl_rc_hold_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_recover = _control_signal;
        shared.ctrl_rc_hold_y_error_integral = state.ctrl_rc_hold_y.error_integral();
        shared.ctrl_rc_hold_y_previous_error = state.ctrl_rc_hold_y.previous_error();
        shared.ctrl_rc_hold_y_first_sample = state.ctrl_rc_hold_y.is_first_sample();
    }
    // ctrl_rc_hold_x
    {
        const double _control_signal = state.ctrl_rc_hold_x.control(shared.pose_ee_base_distance_x_err_recover, shared.dt_measured_s, {shared.ctrl_rc_hold_x_kp, shared.ctrl_rc_hold_x_ki, shared.ctrl_rc_hold_x_kd, shared.ctrl_rc_hold_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_recover = _control_signal;
        shared.ctrl_rc_hold_x_error_integral = state.ctrl_rc_hold_x.error_integral();
        shared.ctrl_rc_hold_x_previous_error = state.ctrl_rc_hold_x.previous_error();
        shared.ctrl_rc_hold_x_first_sample = state.ctrl_rc_hold_x.is_first_sample();
    }

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.pose_ee_base_distance_x_err_recover, shared.satisfied_band);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_recover_hold_hold_s,
            active,
            shared.dt_measured_s,
            0.1);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(13);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_RECOVER);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_RECOVER] << std::endl;
        }
    }

    KDL::SetToZero(state.arm_solver_recover.spatial_directions);

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_recover.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_recover;

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_recover.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_recover;

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_recover.acceleration_energy(2) = shared.eacc_pose_ee_base_distance_z_recover;

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_recover.acceleration_energy(3) = shared.eacc_ctrl_rc_hold_orientation_ang_x;

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_recover.acceleration_energy(4) = shared.eacc_ctrl_rc_hold_orientation_ang_y;

    state.arm_solver_recover.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_recover.acceleration_energy(5) = shared.eacc_ctrl_rc_hold_orientation_ang_z;

    KDL::SetToZero(state.arm_solver_recover.tau_ff);

    for (int i = 0; i < state.arm_solver_recover.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver_recover.f_ext[i]);
    }

    state.arm_solver_recover.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver_recover.chain, "half_arm_2_link", "base_link") - 1] += shared.wrench_force_ctrl_rc_support_z;

    KDL::Wrenches f_ext_zero_arm_solver_recover(state.arm_solver_recover.num_segments);
    for (int i = 0; i < state.arm_solver_recover.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_recover[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_recover(state.arm_solver_recover.num_joints);
    state.arm_solver_recover.achd_acc->CartToJnt(
        state.arm_solver_recover.q,
        state.arm_solver_recover.qd,
        state.arm_solver_recover.qdd,
        state.arm_solver_recover.spatial_directions,
        state.arm_solver_recover.acceleration_energy,
        state.arm_solver_recover.f_ext,
        state.arm_solver_recover.tau_ff,
        tau_ctrl_acc_arm_solver_recover);
    state.arm_solver_recover.rnea->CartToJnt(
        state.arm_solver_recover.q,
        state.arm_solver_recover.qd,
        state.arm_solver_recover.qdd,
        f_ext_zero_arm_solver_recover,
        state.arm_solver_recover.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_recover.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_recover.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_recover.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_recover.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_recover.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_recover.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_recover.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_recover.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_recover.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_recover.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_recover.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_recover.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_recover.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_recover.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_recover.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_recover.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_recover.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_recover.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_recover.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_recover.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_recover.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_recover.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_recover.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_recover.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_recover.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_recover.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_recover.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_recover.tau_ctrl(6);

}

inline void apply_motion_recover(
    motion_recover_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_recover.num_joints; ++i) {
        robot.arm_solver_recover.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_recover.tau_ctrl(i), i);
    }

}
