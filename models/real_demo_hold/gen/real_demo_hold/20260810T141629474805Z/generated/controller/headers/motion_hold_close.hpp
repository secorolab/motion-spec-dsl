/// Motion: hold-close
/// Hold the starting pose while the gripper closes
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_hold_close_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_hold_close_solver_state arm_solver_hold_close;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_x;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_y;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_z;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_z;

    bool mon_closed_previous = false;
    bool mon_closed_event_triggered = false;
    double mon_closed_hold_s = 0.0;

};

inline void reset_motion_hold_close(motion_hold_close_state &state) {
    state = motion_hold_close_state{};
}

inline void init_motion_hold_close(motion_hold_close_state &state, const robot_io &robot) {
    if (!state.arm_solver_hold_close.initialized) {
        state.arm_solver_hold_close.num_joints = robot.arm_solver_hold_close.chain->getNrOfJoints();
        state.arm_solver_hold_close.num_segments = robot.arm_solver_hold_close.chain->getNrOfSegments();
        state.arm_solver_hold_close.q = KDL::JntArray(state.arm_solver_hold_close.num_joints);
        state.arm_solver_hold_close.qd = KDL::JntArray(state.arm_solver_hold_close.num_joints);
        state.arm_solver_hold_close.qdd = KDL::JntArray(state.arm_solver_hold_close.num_joints);
        state.arm_solver_hold_close.tau_ff = KDL::JntArray(state.arm_solver_hold_close.num_joints);
        state.arm_solver_hold_close.tau_ctrl = KDL::JntArray(state.arm_solver_hold_close.num_joints);
        state.arm_solver_hold_close.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_hold_close.num_spatial_directions = 6;
        state.arm_solver_hold_close.spatial_directions = KDL::Jacobian(state.arm_solver_hold_close.num_spatial_directions);
        state.arm_solver_hold_close.acceleration_energy = KDL::JntArray(state.arm_solver_hold_close.num_spatial_directions);
        state.arm_solver_hold_close.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_hold_close.chain, state.arm_solver_hold_close.root_acc, state.arm_solver_hold_close.num_spatial_directions);

        state.arm_solver_hold_close.initialized = true;
    }
}

inline void update_motion_hold_close(
    motion_hold_close_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_hold_close(state, robot);
    if (robot.ext_force != nullptr) {
        shared.ext_force = *robot.ext_force;
    }

    for (int i = 0; i < state.arm_solver_hold_close.num_joints; ++i) {
        state.arm_solver_hold_close.q(i) = robot.arm_solver_hold_close.state->pos_msr[i];
        state.arm_solver_hold_close.qd(i) = robot.arm_solver_hold_close.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_hold_close(state.arm_solver_hold_close.q, state.arm_solver_hold_close.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_hold_close.chain);
        fk.JntToCart(
            state.arm_solver_hold_close.q,
            shared.pose_ee_base,
            11);
    }

    {
        // The driver reports in the sensor's own frame; the model asked for it at its reference
        // point, seen by another frame. Both come off the same forward kinematics as a pose does.
        const auto ext_force_sample = robot.ft_wrist_ft->read();
        const KDL::Wrench measured_ext_force(
            KDL::Vector(ext_force_sample.value.force[0], ext_force_sample.value.force[1], ext_force_sample.value.force[2]),
            KDL::Vector(ext_force_sample.value.moment[0], ext_force_sample.value.moment[1], ext_force_sample.value.moment[2]));
        const auto frame_ext_force = [&](unsigned int segment) {
            KDL::Frame frame;
            KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_hold_close.chain);
            fk.JntToCart(
                state.arm_solver_hold_close.q,
                frame,
                segment);
            return frame;
        };
        const KDL::Wrench at_ext_force = motion_spec::runtime::transform_wrench(
            measured_ext_force,
            frame_ext_force(8),
            frame_ext_force(8),
            frame_ext_force(0));
        if (shared.ext_force_ft_settle < robot.ft_wrist_ft_bias_samples) {
            ++shared.ext_force_ft_settle;
            shared.ext_force_ft_bias = at_ext_force;   // last settle step is the tare sample
            shared.ext_force = KDL::Wrench::Zero();
        } else {
            shared.ext_force = shared.ext_force_ft_bias - at_ext_force;
        }
    }
    shared.gripper_pos = *robot.arm_solver_hold_close.gripper_pos;

    if (!state.snapshot_taken) {
        shared.start_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_hold_close() {
    return true;
}

inline void monitor_when_motion_hold_close() {
}

inline void monitor_until_motion_hold_close(
    motion_hold_close_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_hold_close_until_closed
    shared.eval_hold_close_until_closed_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper_pos, shared.grasp_threshold);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_hold_close_until_closed_err, shared.default_tolerance_Angle);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_closed_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, real_demo_hold_fsm::E_GRIPPER_CLOSED);
            std::cerr << "[fsm] event   " << real_demo_hold_fsm::EVENT_URIS[real_demo_hold_fsm::E_GRIPPER_CLOSED] << std::endl;
        }
    }

}

inline void monitor_motion_hold_close(
    motion_hold_close_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_hold_close_until_closed
    shared.eval_hold_close_until_closed_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper_pos, shared.grasp_threshold);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_hold_close_until_closed_err, shared.default_tolerance_Angle);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_closed_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, real_demo_hold_fsm::E_GRIPPER_CLOSED);
            std::cerr << "[fsm] event   " << real_demo_hold_fsm::EVENT_URIS[real_demo_hold_fsm::E_GRIPPER_CLOSED] << std::endl;
        }
    }

}

inline void control_motion_hold_close(
    motion_hold_close_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_hold_position
    shared.pose_diff_ctrl_hold_position = KDL::diff(shared.pose_ee_base, shared.start_pose);
    shared.ctrl_hold_position_err_lin_x = shared.pose_diff_ctrl_hold_position.vel[0];
    shared.ctrl_hold_position_err_lin_y = shared.pose_diff_ctrl_hold_position.vel[1];
    shared.ctrl_hold_position_err_lin_z = shared.pose_diff_ctrl_hold_position.vel[2];
    // eval_pose_diff_ctrl_hold_orientation
    shared.pose_diff_ctrl_hold_orientation = KDL::diff(shared.pose_ee_base, shared.start_pose);
    shared.ctrl_hold_orientation_err_ang_x = shared.pose_diff_ctrl_hold_orientation.rot[0];
    shared.ctrl_hold_orientation_err_ang_y = shared.pose_diff_ctrl_hold_orientation.rot[1];
    shared.ctrl_hold_orientation_err_ang_z = shared.pose_diff_ctrl_hold_orientation.rot[2];
    // eval_hold_close_while_close_gripper
    shared.gripper_pos_err_hold_close = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper_pos);
    // ctrl_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_close_gripper = _control_signal;
    }
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

    KDL::SetToZero(state.arm_solver_hold_close.spatial_directions);

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(0) = shared.eacc_ctrl_hold_position_lin_x;

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(1) = shared.eacc_ctrl_hold_position_lin_y;

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(2) = shared.eacc_ctrl_hold_position_lin_z;

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(3) = shared.eacc_ctrl_hold_orientation_ang_x;

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(4) = shared.eacc_ctrl_hold_orientation_ang_y;

    state.arm_solver_hold_close.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_hold_close.acceleration_energy(5) = shared.eacc_ctrl_hold_orientation_ang_z;

    KDL::SetToZero(state.arm_solver_hold_close.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_hold_close(state.arm_solver_hold_close.num_segments);
    for (int i = 0; i < state.arm_solver_hold_close.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_hold_close[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_hold_close(state.arm_solver_hold_close.num_joints);
    state.arm_solver_hold_close.achd_acc->CartToJnt(
        state.arm_solver_hold_close.q,
        state.arm_solver_hold_close.qd,
        state.arm_solver_hold_close.qdd,
        state.arm_solver_hold_close.spatial_directions,
        state.arm_solver_hold_close.acceleration_energy,
        f_ext_zero_arm_solver_hold_close,
        state.arm_solver_hold_close.tau_ff,
        tau_ctrl_acc_arm_solver_hold_close);

    state.arm_solver_hold_close.tau_ctrl = tau_ctrl_acc_arm_solver_hold_close;

    shared.arm_solver_hold_close_q_joint_1 = state.arm_solver_hold_close.q(0);
    shared.arm_solver_hold_close_q_joint_2 = state.arm_solver_hold_close.q(1);
    shared.arm_solver_hold_close_q_joint_3 = state.arm_solver_hold_close.q(2);
    shared.arm_solver_hold_close_q_joint_4 = state.arm_solver_hold_close.q(3);
    shared.arm_solver_hold_close_q_joint_5 = state.arm_solver_hold_close.q(4);
    shared.arm_solver_hold_close_q_joint_6 = state.arm_solver_hold_close.q(5);
    shared.arm_solver_hold_close_q_joint_7 = state.arm_solver_hold_close.q(6);
    shared.arm_solver_hold_close_qd_joint_1 = state.arm_solver_hold_close.qd(0);
    shared.arm_solver_hold_close_qd_joint_2 = state.arm_solver_hold_close.qd(1);
    shared.arm_solver_hold_close_qd_joint_3 = state.arm_solver_hold_close.qd(2);
    shared.arm_solver_hold_close_qd_joint_4 = state.arm_solver_hold_close.qd(3);
    shared.arm_solver_hold_close_qd_joint_5 = state.arm_solver_hold_close.qd(4);
    shared.arm_solver_hold_close_qd_joint_6 = state.arm_solver_hold_close.qd(5);
    shared.arm_solver_hold_close_qd_joint_7 = state.arm_solver_hold_close.qd(6);
    shared.arm_solver_hold_close_qdd_joint_1 = state.arm_solver_hold_close.qdd(0);
    shared.arm_solver_hold_close_qdd_joint_2 = state.arm_solver_hold_close.qdd(1);
    shared.arm_solver_hold_close_qdd_joint_3 = state.arm_solver_hold_close.qdd(2);
    shared.arm_solver_hold_close_qdd_joint_4 = state.arm_solver_hold_close.qdd(3);
    shared.arm_solver_hold_close_qdd_joint_5 = state.arm_solver_hold_close.qdd(4);
    shared.arm_solver_hold_close_qdd_joint_6 = state.arm_solver_hold_close.qdd(5);
    shared.arm_solver_hold_close_qdd_joint_7 = state.arm_solver_hold_close.qdd(6);
    shared.arm_solver_hold_close_tau_ctrl_joint_1 = state.arm_solver_hold_close.tau_ctrl(0);
    shared.arm_solver_hold_close_tau_ctrl_joint_2 = state.arm_solver_hold_close.tau_ctrl(1);
    shared.arm_solver_hold_close_tau_ctrl_joint_3 = state.arm_solver_hold_close.tau_ctrl(2);
    shared.arm_solver_hold_close_tau_ctrl_joint_4 = state.arm_solver_hold_close.tau_ctrl(3);
    shared.arm_solver_hold_close_tau_ctrl_joint_5 = state.arm_solver_hold_close.tau_ctrl(4);
    shared.arm_solver_hold_close_tau_ctrl_joint_6 = state.arm_solver_hold_close.tau_ctrl(5);
    shared.arm_solver_hold_close_tau_ctrl_joint_7 = state.arm_solver_hold_close.tau_ctrl(6);
    shared.arm_solver_hold_close_tau_msr_joint_1 = robot.arm_solver_hold_close.state->eff_msr[0];
    shared.arm_solver_hold_close_tau_msr_joint_2 = robot.arm_solver_hold_close.state->eff_msr[1];
    shared.arm_solver_hold_close_tau_msr_joint_3 = robot.arm_solver_hold_close.state->eff_msr[2];
    shared.arm_solver_hold_close_tau_msr_joint_4 = robot.arm_solver_hold_close.state->eff_msr[3];
    shared.arm_solver_hold_close_tau_msr_joint_5 = robot.arm_solver_hold_close.state->eff_msr[4];
    shared.arm_solver_hold_close_tau_msr_joint_6 = robot.arm_solver_hold_close.state->eff_msr[5];
    shared.arm_solver_hold_close_tau_msr_joint_7 = robot.arm_solver_hold_close.state->eff_msr[6];

}

inline void apply_motion_hold_close(
    motion_hold_close_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_hold_close.num_joints; ++i) {
        robot.arm_solver_hold_close.state->eff_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_hold_close.tau_ctrl(i), i);
    }

    *robot.arm_solver_hold_close.gripper_cmd = shared.cmd_ctrl_close_gripper;

}
