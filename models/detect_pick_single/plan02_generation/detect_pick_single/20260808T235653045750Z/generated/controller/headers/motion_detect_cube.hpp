/// Motion: detect-cube
/// Hold the home pose while the perception server is asked where the cube is
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_detect_cube_state {
    bool active = false;
    int active_steps = 0;
    double motion_start_time = -1.0;
    arm_solver_detect_cube_solver_state arm_solver_detect_cube;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_dt_hold_position_lin_x;
    motion_spec::runtime::PIDControl ctrl_dt_hold_position_lin_y;
    motion_spec::runtime::PIDControl ctrl_dt_hold_position_lin_z;
    motion_spec::runtime::PIDControl ctrl_dt_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_dt_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_dt_hold_orientation_ang_z;
    bool mon_detect_ready_previous = false;
    bool mon_detect_ready_event_triggered = false;

    bool mon_cube_detected_previous = false;
    bool mon_cube_detected_event_triggered = false;

    bool mon_detect_failed_previous = false;
    bool mon_detect_failed_event_triggered = false;

};

inline void reset_motion_detect_cube(motion_detect_cube_state &state) {
    state = motion_detect_cube_state{};
}

inline void init_motion_detect_cube(motion_detect_cube_state &state, const robot_io &robot) {
    if (!state.arm_solver_detect_cube.initialized) {
        state.arm_solver_detect_cube.num_joints = robot.arm_solver_detect_cube.chain->getNrOfJoints();
        state.arm_solver_detect_cube.num_segments = robot.arm_solver_detect_cube.chain->getNrOfSegments();
        state.arm_solver_detect_cube.q = KDL::JntArray(state.arm_solver_detect_cube.num_joints);
        state.arm_solver_detect_cube.qd = KDL::JntArray(state.arm_solver_detect_cube.num_joints);
        state.arm_solver_detect_cube.qdd = KDL::JntArray(state.arm_solver_detect_cube.num_joints);
        state.arm_solver_detect_cube.tau_ff = KDL::JntArray(state.arm_solver_detect_cube.num_joints);
        state.arm_solver_detect_cube.tau_ctrl = KDL::JntArray(state.arm_solver_detect_cube.num_joints);
        state.arm_solver_detect_cube.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_detect_cube.num_spatial_directions = 6;
        state.arm_solver_detect_cube.spatial_directions = KDL::Jacobian(state.arm_solver_detect_cube.num_spatial_directions);
        state.arm_solver_detect_cube.acceleration_energy = KDL::JntArray(state.arm_solver_detect_cube.num_spatial_directions);
        state.arm_solver_detect_cube.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_detect_cube.chain, state.arm_solver_detect_cube.root_acc, state.arm_solver_detect_cube.num_spatial_directions);
        state.arm_solver_detect_cube.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_detect_cube.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_detect_cube.initialized = true;
    }
}

inline void update_motion_detect_cube(
    motion_detect_cube_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_detect_cube(state, robot);
    if (state.motion_start_time < 0.0) {
        state.motion_start_time = shared.clock_time_s;
    }
    shared.hung_elapsed = shared.clock_time_s - state.motion_start_time;

    mj_kdl::update(robot.arm_solver_detect_cube.robot);
    for (int i = 0; i < state.arm_solver_detect_cube.num_joints; ++i) {
        state.arm_solver_detect_cube.q(i) = robot.arm_solver_detect_cube.robot->jnt_pos_msr[i];
        state.arm_solver_detect_cube.qd(i) = robot.arm_solver_detect_cube.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_detect_cube(state.arm_solver_detect_cube.q, state.arm_solver_detect_cube.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_detect_cube.chain);
        fk.JntToCart(
            state.arm_solver_detect_cube.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_detect_cube.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_detect_cube.chain);
        fk.JntToCart(
            state.arm_solver_detect_cube.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_detect_cube.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_detect_cube.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_detect_cube,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_detect_cube.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_detect_cube.robot->model,
                robot.arm_solver_detect_cube.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_detect_cube.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_detect_cube.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.hold_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_detect_cube(
    shared_data &shared
) {
    // eval_detect_cube_when_gripper_ready
    shared.eval_detect_cube_when_gripper_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    return motion_spec::runtime::constraint_satisfied(shared.eval_detect_cube_when_gripper_ready_err, shared.satisfied_band_rot);
}

inline void monitor_when_motion_detect_cube(
    motion_detect_cube_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_detect_cube_when_gripper_ready
    shared.eval_detect_cube_when_gripper_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_detect_cube_when_gripper_ready_err, shared.satisfied_band_rot);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_detect_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(2);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_DETECT_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_DETECT_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_detect_cube(
    motion_detect_cube_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // mon_cube_detected

    // eval_lost

    // eval_detect_cube_until_located

    {
        const bool active = (shared.locate_cube_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_cube_detected_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_CUBE_DETECTED);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_CUBE_DETECTED] << std::endl;
        }
    }

    {
        const bool active = ((shared.locate_cube_status == action_msgs::msg::GoalStatus::STATUS_ABORTED) || (shared.hung_elapsed >= 10.000000));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_detect_failed_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(1);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_DETECT_FAILED);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_DETECT_FAILED] << std::endl;
        }
    }

}

inline void monitor_motion_detect_cube(
    motion_detect_cube_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_detect_cube_when_gripper_ready
    shared.eval_detect_cube_when_gripper_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_detect_cube_when_gripper_ready_err, shared.satisfied_band_rot);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_detect_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(2);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_DETECT_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_DETECT_READY] << std::endl;
        }
    }

    // mon_cube_detected

    // eval_lost

    // eval_detect_cube_until_located

    {
        const bool active = (shared.locate_cube_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_cube_detected_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_CUBE_DETECTED);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_CUBE_DETECTED] << std::endl;
        }
    }

    {
        const bool active = ((shared.locate_cube_status == action_msgs::msg::GoalStatus::STATUS_ABORTED) || (shared.hung_elapsed >= 10.000000));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_detect_failed_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(1);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_DETECT_FAILED);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_DETECT_FAILED] << std::endl;
        }
    }

}

inline void control_motion_detect_cube(
    motion_detect_cube_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_dt_hold_position
    shared.pose_diff_ctrl_dt_hold_position = KDL::diff(shared.pose_ee_base, shared.hold_pose);
    shared.ctrl_dt_hold_position_err_lin_x = shared.pose_diff_ctrl_dt_hold_position.vel[0];
    shared.ctrl_dt_hold_position_err_lin_y = shared.pose_diff_ctrl_dt_hold_position.vel[1];
    shared.ctrl_dt_hold_position_err_lin_z = shared.pose_diff_ctrl_dt_hold_position.vel[2];
    // eval_pose_diff_ctrl_dt_hold_orientation
    shared.pose_diff_ctrl_dt_hold_orientation = KDL::diff(shared.pose_ee_base, shared.hold_pose);
    shared.ctrl_dt_hold_orientation_err_ang_x = shared.pose_diff_ctrl_dt_hold_orientation.rot[0];
    shared.ctrl_dt_hold_orientation_err_ang_y = shared.pose_diff_ctrl_dt_hold_orientation.rot[1];
    shared.ctrl_dt_hold_orientation_err_ang_z = shared.pose_diff_ctrl_dt_hold_orientation.rot[2];
    // ctrl_dt_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_dt_hold_orientation_ang_z.control(shared.pose_diff_ctrl_dt_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_dt_hold_orientation_ang_z_kp, shared.ctrl_dt_hold_orientation_ang_z_ki, shared.ctrl_dt_hold_orientation_ang_z_kd, shared.ctrl_dt_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_dt_hold_orientation_ang_z = _control_signal;
        shared.ctrl_dt_hold_orientation_ang_z_error_integral = state.ctrl_dt_hold_orientation_ang_z.error_integral();
        shared.ctrl_dt_hold_orientation_ang_z_previous_error = state.ctrl_dt_hold_orientation_ang_z.previous_error();
        shared.ctrl_dt_hold_orientation_ang_z_first_sample = state.ctrl_dt_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_dt_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_dt_hold_orientation_ang_y.control(shared.pose_diff_ctrl_dt_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_dt_hold_orientation_ang_y_kp, shared.ctrl_dt_hold_orientation_ang_y_ki, shared.ctrl_dt_hold_orientation_ang_y_kd, shared.ctrl_dt_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_dt_hold_orientation_ang_y = _control_signal;
        shared.ctrl_dt_hold_orientation_ang_y_error_integral = state.ctrl_dt_hold_orientation_ang_y.error_integral();
        shared.ctrl_dt_hold_orientation_ang_y_previous_error = state.ctrl_dt_hold_orientation_ang_y.previous_error();
        shared.ctrl_dt_hold_orientation_ang_y_first_sample = state.ctrl_dt_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_dt_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_dt_hold_orientation_ang_x.control(shared.pose_diff_ctrl_dt_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_dt_hold_orientation_ang_x_kp, shared.ctrl_dt_hold_orientation_ang_x_ki, shared.ctrl_dt_hold_orientation_ang_x_kd, shared.ctrl_dt_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_dt_hold_orientation_ang_x = _control_signal;
        shared.ctrl_dt_hold_orientation_ang_x_error_integral = state.ctrl_dt_hold_orientation_ang_x.error_integral();
        shared.ctrl_dt_hold_orientation_ang_x_previous_error = state.ctrl_dt_hold_orientation_ang_x.previous_error();
        shared.ctrl_dt_hold_orientation_ang_x_first_sample = state.ctrl_dt_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_dt_hold_position_lin_z
    {
        const double _control_signal = state.ctrl_dt_hold_position_lin_z.control(shared.pose_diff_ctrl_dt_hold_position.vel[2], shared.dt_measured_s, {shared.ctrl_dt_hold_position_lin_z_kp, shared.ctrl_dt_hold_position_lin_z_ki, shared.ctrl_dt_hold_position_lin_z_kd, shared.ctrl_dt_hold_position_lin_z_decay_rate});
        shared.eacc_ctrl_dt_hold_position_lin_z = _control_signal;
        shared.ctrl_dt_hold_position_lin_z_error_integral = state.ctrl_dt_hold_position_lin_z.error_integral();
        shared.ctrl_dt_hold_position_lin_z_previous_error = state.ctrl_dt_hold_position_lin_z.previous_error();
        shared.ctrl_dt_hold_position_lin_z_first_sample = state.ctrl_dt_hold_position_lin_z.is_first_sample();
    }
    // ctrl_dt_hold_position_lin_y
    {
        const double _control_signal = state.ctrl_dt_hold_position_lin_y.control(shared.pose_diff_ctrl_dt_hold_position.vel[1], shared.dt_measured_s, {shared.ctrl_dt_hold_position_lin_y_kp, shared.ctrl_dt_hold_position_lin_y_ki, shared.ctrl_dt_hold_position_lin_y_kd, shared.ctrl_dt_hold_position_lin_y_decay_rate});
        shared.eacc_ctrl_dt_hold_position_lin_y = _control_signal;
        shared.ctrl_dt_hold_position_lin_y_error_integral = state.ctrl_dt_hold_position_lin_y.error_integral();
        shared.ctrl_dt_hold_position_lin_y_previous_error = state.ctrl_dt_hold_position_lin_y.previous_error();
        shared.ctrl_dt_hold_position_lin_y_first_sample = state.ctrl_dt_hold_position_lin_y.is_first_sample();
    }
    // ctrl_dt_hold_position_lin_x
    {
        const double _control_signal = state.ctrl_dt_hold_position_lin_x.control(shared.pose_diff_ctrl_dt_hold_position.vel[0], shared.dt_measured_s, {shared.ctrl_dt_hold_position_lin_x_kp, shared.ctrl_dt_hold_position_lin_x_ki, shared.ctrl_dt_hold_position_lin_x_kd, shared.ctrl_dt_hold_position_lin_x_decay_rate});
        shared.eacc_ctrl_dt_hold_position_lin_x = _control_signal;
        shared.ctrl_dt_hold_position_lin_x_error_integral = state.ctrl_dt_hold_position_lin_x.error_integral();
        shared.ctrl_dt_hold_position_lin_x_previous_error = state.ctrl_dt_hold_position_lin_x.previous_error();
        shared.ctrl_dt_hold_position_lin_x_first_sample = state.ctrl_dt_hold_position_lin_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_detect_cube.spatial_directions);

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(0) = shared.eacc_ctrl_dt_hold_position_lin_x;

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(1) = shared.eacc_ctrl_dt_hold_position_lin_y;

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(2) = shared.eacc_ctrl_dt_hold_position_lin_z;

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(3) = shared.eacc_ctrl_dt_hold_orientation_ang_x;

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(4) = shared.eacc_ctrl_dt_hold_orientation_ang_y;

    state.arm_solver_detect_cube.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_detect_cube.acceleration_energy(5) = shared.eacc_ctrl_dt_hold_orientation_ang_z;

    KDL::SetToZero(state.arm_solver_detect_cube.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_detect_cube(state.arm_solver_detect_cube.num_segments);
    for (int i = 0; i < state.arm_solver_detect_cube.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_detect_cube[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_detect_cube(state.arm_solver_detect_cube.num_joints);
    state.arm_solver_detect_cube.achd_acc->CartToJnt(
        state.arm_solver_detect_cube.q,
        state.arm_solver_detect_cube.qd,
        state.arm_solver_detect_cube.qdd,
        state.arm_solver_detect_cube.spatial_directions,
        state.arm_solver_detect_cube.acceleration_energy,
        f_ext_zero_arm_solver_detect_cube,
        state.arm_solver_detect_cube.tau_ff,
        tau_ctrl_acc_arm_solver_detect_cube);
    state.arm_solver_detect_cube.rnea->CartToJnt(
        state.arm_solver_detect_cube.q,
        state.arm_solver_detect_cube.qd,
        state.arm_solver_detect_cube.qdd,
        f_ext_zero_arm_solver_detect_cube,
        state.arm_solver_detect_cube.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_detect_cube.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_detect_cube.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_detect_cube.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_detect_cube.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_detect_cube.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_detect_cube.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_detect_cube.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_detect_cube.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_detect_cube.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_detect_cube.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_detect_cube.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_detect_cube.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_detect_cube.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_detect_cube.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_detect_cube.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_detect_cube.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_detect_cube.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_detect_cube.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_detect_cube.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_detect_cube.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_detect_cube.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_detect_cube.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_detect_cube.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_detect_cube.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_detect_cube.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_detect_cube.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_detect_cube.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_detect_cube.tau_ctrl(6);

}

inline void apply_motion_detect_cube(
    motion_detect_cube_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_detect_cube.num_joints; ++i) {
        robot.arm_solver_detect_cube.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_detect_cube.tau_ctrl(i), i);
    }

}
