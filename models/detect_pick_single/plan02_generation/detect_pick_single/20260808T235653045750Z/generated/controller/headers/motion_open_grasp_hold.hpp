/// Motion: open-grasp-hold
/// Hold TCP at place pose while the gripper opens
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_open_grasp_hold_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_open_grasp_hold_solver_state arm_solver_open_grasp_hold;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_og_hold_x;
    motion_spec::runtime::PIDControl ctrl_og_hold_y;
    motion_spec::runtime::PIDControl ctrl_og_hold_z;
    motion_spec::runtime::PIDControl ctrl_og_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_og_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_og_hold_orientation_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_og_support_z;

    bool mon_open_ready_previous = false;
    bool mon_open_ready_event_triggered = false;

};

inline void reset_motion_open_grasp_hold(motion_open_grasp_hold_state &state) {
    state = motion_open_grasp_hold_state{};
}

inline void init_motion_open_grasp_hold(motion_open_grasp_hold_state &state, const robot_io &robot) {
    if (!state.arm_solver_open_grasp_hold.initialized) {
        state.arm_solver_open_grasp_hold.num_joints = robot.arm_solver_open_grasp_hold.chain->getNrOfJoints();
        state.arm_solver_open_grasp_hold.num_segments = robot.arm_solver_open_grasp_hold.chain->getNrOfSegments();
        state.arm_solver_open_grasp_hold.q = KDL::JntArray(state.arm_solver_open_grasp_hold.num_joints);
        state.arm_solver_open_grasp_hold.qd = KDL::JntArray(state.arm_solver_open_grasp_hold.num_joints);
        state.arm_solver_open_grasp_hold.qdd = KDL::JntArray(state.arm_solver_open_grasp_hold.num_joints);
        state.arm_solver_open_grasp_hold.tau_ff = KDL::JntArray(state.arm_solver_open_grasp_hold.num_joints);
        state.arm_solver_open_grasp_hold.tau_ctrl = KDL::JntArray(state.arm_solver_open_grasp_hold.num_joints);
        state.arm_solver_open_grasp_hold.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_open_grasp_hold.num_spatial_directions = 6;
        state.arm_solver_open_grasp_hold.spatial_directions = KDL::Jacobian(state.arm_solver_open_grasp_hold.num_spatial_directions);
        state.arm_solver_open_grasp_hold.acceleration_energy = KDL::JntArray(state.arm_solver_open_grasp_hold.num_spatial_directions);
        state.arm_solver_open_grasp_hold.f_ext = KDL::Wrenches(state.arm_solver_open_grasp_hold.num_segments);
        state.arm_solver_open_grasp_hold.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver_open_grasp_hold.chain, state.arm_solver_open_grasp_hold.root_acc, state.arm_solver_open_grasp_hold.num_spatial_directions);
        state.arm_solver_open_grasp_hold.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_open_grasp_hold.chain, state.arm_solver_open_grasp_hold.root_acc, state.arm_solver_open_grasp_hold.num_spatial_directions);
        state.arm_solver_open_grasp_hold.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_open_grasp_hold.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_open_grasp_hold.initialized = true;
    }
}

inline void update_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_open_grasp_hold(state, robot);

    mj_kdl::update(robot.arm_solver_open_grasp_hold.robot);
    for (int i = 0; i < state.arm_solver_open_grasp_hold.num_joints; ++i) {
        state.arm_solver_open_grasp_hold.q(i) = robot.arm_solver_open_grasp_hold.robot->jnt_pos_msr[i];
        state.arm_solver_open_grasp_hold.qd(i) = robot.arm_solver_open_grasp_hold.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_open_grasp_hold(state.arm_solver_open_grasp_hold.q, state.arm_solver_open_grasp_hold.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm_solver_open_grasp_hold.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_open_grasp_hold.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm_solver_open_grasp_hold.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_open_grasp_hold.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_open_grasp_hold.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_open_grasp_hold,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_open_grasp_hold.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_open_grasp_hold.robot->model,
                robot.arm_solver_open_grasp_hold.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_open_grasp_hold.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_open_grasp_hold.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.open_grasp_hold_hold_orientation_pose = shared.pose_ee_base;

        shared.open_grasp_hold_support_z_add_out = shared.pose_elbow_base.p[2] + shared.open_grasp_hold_support_lift;
        shared.open_grasp_hold_support_z = shared.open_grasp_hold_support_z_add_out;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_open_grasp_hold(
    shared_data &shared
) {
    // eval_open_grasp_hold_when_at_place
    shared.eval_open_grasp_hold_when_at_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee_base.p[2]);

    return motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at_place_err, shared.satisfied_band);
}

inline void monitor_when_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_open_grasp_hold_when_at_place
    shared.eval_open_grasp_hold_when_at_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee_base.p[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at_place_err, shared.satisfied_band);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_open_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(9);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_OPEN_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_OPEN_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_open_grasp_hold() {
}

inline void monitor_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_open_grasp_hold_when_at_place
    shared.eval_open_grasp_hold_when_at_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee_base.p[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at_place_err, shared.satisfied_band);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_open_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(9);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_OPEN_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_OPEN_READY] << std::endl;
        }
    }

}

inline void control_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.open_grasp_hold_hold_orientation_pose.p[0];
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.open_grasp_hold_hold_orientation_pose.p[1];
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.open_grasp_hold_hold_orientation_pose.p[2];
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();

        shared.pose_ee_base_distance_x_err_open_grasp_hold = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_open_grasp_hold = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_open_grasp_hold = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_og_hold_orientation
    shared.pose_diff_ctrl_og_hold_orientation = KDL::diff(shared.pose_ee_base, shared.open_grasp_hold_hold_orientation_pose);
    shared.ctrl_og_hold_orientation_err_ang_x = shared.pose_diff_ctrl_og_hold_orientation.rot[0];
    shared.ctrl_og_hold_orientation_err_ang_y = shared.pose_diff_ctrl_og_hold_orientation.rot[1];
    shared.ctrl_og_hold_orientation_err_ang_z = shared.pose_diff_ctrl_og_hold_orientation.rot[2];
    // eval_open_grasp_hold_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.open_grasp_hold_support_z, shared.pose_elbow_base.p[2]);
    // compute_wrench_force_ctrl_og_support_z
    shared.wrench_force_ctrl_og_support_z = KDL::Wrench(shared.direction_ctrl_og_support_z * shared.force_ctrl_og_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_og_support_z);
    // eval_open_grasp_hold_while_open_gripper
    shared.gripper_pos_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);
    // ctrl_og_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_og_open_gripper = _control_signal;
    }
    // ctrl_og_support_z
    {
        const double _control_signal = state.ctrl_og_support_z.control(shared.pose_elbow_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og_support_z_stiffness, shared.ctrl_og_support_z_damping, shared.ctrl_og_support_z_integral_gain});
        shared.force_ctrl_og_support_z = _control_signal;
        shared.ctrl_og_support_z_error_integral = state.ctrl_og_support_z.error_integral();
        shared.ctrl_og_support_z_previous_error = state.ctrl_og_support_z.previous_error();
        shared.ctrl_og_support_z_first_sample = state.ctrl_og_support_z.is_first_sample();
    }
    // ctrl_og_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_og_hold_orientation_ang_z.control(shared.pose_diff_ctrl_og_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_og_hold_orientation_ang_z_kp, shared.ctrl_og_hold_orientation_ang_z_ki, shared.ctrl_og_hold_orientation_ang_z_kd, shared.ctrl_og_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_og_hold_orientation_ang_z = _control_signal;
        shared.ctrl_og_hold_orientation_ang_z_error_integral = state.ctrl_og_hold_orientation_ang_z.error_integral();
        shared.ctrl_og_hold_orientation_ang_z_previous_error = state.ctrl_og_hold_orientation_ang_z.previous_error();
        shared.ctrl_og_hold_orientation_ang_z_first_sample = state.ctrl_og_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_og_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_og_hold_orientation_ang_y.control(shared.pose_diff_ctrl_og_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_og_hold_orientation_ang_y_kp, shared.ctrl_og_hold_orientation_ang_y_ki, shared.ctrl_og_hold_orientation_ang_y_kd, shared.ctrl_og_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_og_hold_orientation_ang_y = _control_signal;
        shared.ctrl_og_hold_orientation_ang_y_error_integral = state.ctrl_og_hold_orientation_ang_y.error_integral();
        shared.ctrl_og_hold_orientation_ang_y_previous_error = state.ctrl_og_hold_orientation_ang_y.previous_error();
        shared.ctrl_og_hold_orientation_ang_y_first_sample = state.ctrl_og_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_og_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_og_hold_orientation_ang_x.control(shared.pose_diff_ctrl_og_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_og_hold_orientation_ang_x_kp, shared.ctrl_og_hold_orientation_ang_x_ki, shared.ctrl_og_hold_orientation_ang_x_kd, shared.ctrl_og_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_og_hold_orientation_ang_x = _control_signal;
        shared.ctrl_og_hold_orientation_ang_x_error_integral = state.ctrl_og_hold_orientation_ang_x.error_integral();
        shared.ctrl_og_hold_orientation_ang_x_previous_error = state.ctrl_og_hold_orientation_ang_x.previous_error();
        shared.ctrl_og_hold_orientation_ang_x_first_sample = state.ctrl_og_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_og_hold_z
    {
        const double _control_signal = state.ctrl_og_hold_z.control(shared.pose_ee_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og_hold_z_kp, shared.ctrl_og_hold_z_ki, shared.ctrl_og_hold_z_kd, shared.ctrl_og_hold_z_decay_rate});
        shared.eacc_pose_ee_base_distance_z_open_grasp_hold = _control_signal;
        shared.ctrl_og_hold_z_error_integral = state.ctrl_og_hold_z.error_integral();
        shared.ctrl_og_hold_z_previous_error = state.ctrl_og_hold_z.previous_error();
        shared.ctrl_og_hold_z_first_sample = state.ctrl_og_hold_z.is_first_sample();
    }
    // ctrl_og_hold_y
    {
        const double _control_signal = state.ctrl_og_hold_y.control(shared.pose_ee_base_distance_y_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og_hold_y_kp, shared.ctrl_og_hold_y_ki, shared.ctrl_og_hold_y_kd, shared.ctrl_og_hold_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_open_grasp_hold = _control_signal;
        shared.ctrl_og_hold_y_error_integral = state.ctrl_og_hold_y.error_integral();
        shared.ctrl_og_hold_y_previous_error = state.ctrl_og_hold_y.previous_error();
        shared.ctrl_og_hold_y_first_sample = state.ctrl_og_hold_y.is_first_sample();
    }
    // ctrl_og_hold_x
    {
        const double _control_signal = state.ctrl_og_hold_x.control(shared.pose_ee_base_distance_x_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og_hold_x_kp, shared.ctrl_og_hold_x_ki, shared.ctrl_og_hold_x_kd, shared.ctrl_og_hold_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_open_grasp_hold = _control_signal;
        shared.ctrl_og_hold_x_error_integral = state.ctrl_og_hold_x.error_integral();
        shared.ctrl_og_hold_x_previous_error = state.ctrl_og_hold_x.previous_error();
        shared.ctrl_og_hold_x_first_sample = state.ctrl_og_hold_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_open_grasp_hold.spatial_directions);

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_open_grasp_hold;

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_open_grasp_hold;

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(2) = shared.eacc_pose_ee_base_distance_z_open_grasp_hold;

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(3) = shared.eacc_ctrl_og_hold_orientation_ang_x;

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(4) = shared.eacc_ctrl_og_hold_orientation_ang_y;

    state.arm_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_open_grasp_hold.acceleration_energy(5) = shared.eacc_ctrl_og_hold_orientation_ang_z;

    KDL::SetToZero(state.arm_solver_open_grasp_hold.tau_ff);

    for (int i = 0; i < state.arm_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver_open_grasp_hold.f_ext[i]);
    }

    state.arm_solver_open_grasp_hold.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver_open_grasp_hold.chain, "half_arm_2_link", "base_link") - 1] += shared.wrench_force_ctrl_og_support_z;

    KDL::Wrenches f_ext_zero_arm_solver_open_grasp_hold(state.arm_solver_open_grasp_hold.num_segments);
    for (int i = 0; i < state.arm_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_open_grasp_hold[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_open_grasp_hold(state.arm_solver_open_grasp_hold.num_joints);
    state.arm_solver_open_grasp_hold.achd_acc->CartToJnt(
        state.arm_solver_open_grasp_hold.q,
        state.arm_solver_open_grasp_hold.qd,
        state.arm_solver_open_grasp_hold.qdd,
        state.arm_solver_open_grasp_hold.spatial_directions,
        state.arm_solver_open_grasp_hold.acceleration_energy,
        state.arm_solver_open_grasp_hold.f_ext,
        state.arm_solver_open_grasp_hold.tau_ff,
        tau_ctrl_acc_arm_solver_open_grasp_hold);
    state.arm_solver_open_grasp_hold.rnea->CartToJnt(
        state.arm_solver_open_grasp_hold.q,
        state.arm_solver_open_grasp_hold.qd,
        state.arm_solver_open_grasp_hold.qdd,
        f_ext_zero_arm_solver_open_grasp_hold,
        state.arm_solver_open_grasp_hold.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_open_grasp_hold.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_open_grasp_hold.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_open_grasp_hold.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_open_grasp_hold.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_open_grasp_hold.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_open_grasp_hold.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_open_grasp_hold.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_open_grasp_hold.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_open_grasp_hold.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_open_grasp_hold.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_open_grasp_hold.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_open_grasp_hold.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_open_grasp_hold.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_open_grasp_hold.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_open_grasp_hold.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_open_grasp_hold.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_open_grasp_hold.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_open_grasp_hold.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_open_grasp_hold.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_open_grasp_hold.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_open_grasp_hold.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_open_grasp_hold.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_open_grasp_hold.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_open_grasp_hold.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_open_grasp_hold.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_open_grasp_hold.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_open_grasp_hold.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_open_grasp_hold.tau_ctrl(6);

}

inline void apply_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_open_grasp_hold.num_joints; ++i) {
        robot.arm_solver_open_grasp_hold.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_open_grasp_hold.tau_ctrl(i), i);
    }

    {
        const mjModel *model = robot.arm_solver_home.robot->model;
        int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "g_left_driver_joint");
        const int joint_id = mj_name2id(model, mjOBJ_JOINT, "g_left_driver_joint");
        for (int i = 0; actuator_id < 0 && joint_id >= 0 && i < model->nu; ++i) {
            if (model->actuator_trntype[i] == mjTRN_JOINT
                && model->actuator_trnid[2 * i] == joint_id) actuator_id = i;
        }
        if (actuator_id >= 0) {
            robot.arm_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_og_open_gripper;
        }
    }

}
