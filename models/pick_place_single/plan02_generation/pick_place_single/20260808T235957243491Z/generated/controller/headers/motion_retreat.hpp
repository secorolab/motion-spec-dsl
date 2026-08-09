/// Motion: retreat
/// Move TCP straight up to the pre-grasp height above the place location
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_retreat_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_retreat_solver_state arm_solver_retreat;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_ret_reach_x;
    motion_spec::runtime::PIDControl ctrl_ret_reach_y;
    motion_spec::runtime::PIDControl ctrl_ret_reach_z;
    motion_spec::runtime::PIDControl ctrl_ret_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_ret_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_ret_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_ret_support_z;

    bool mon_retreat_ready_previous = false;
    bool mon_retreat_ready_event_triggered = false;

    bool mon_retreat_settled_previous = false;
    bool mon_retreat_settled_event_triggered = false;

};

inline void reset_motion_retreat(motion_retreat_state &state) {
    state = motion_retreat_state{};
}

inline void init_motion_retreat(motion_retreat_state &state, const robot_io &robot) {
    if (!state.arm_solver_retreat.initialized) {
        state.arm_solver_retreat.num_joints = robot.arm_solver_retreat.chain->getNrOfJoints();
        state.arm_solver_retreat.num_segments = robot.arm_solver_retreat.chain->getNrOfSegments();
        state.arm_solver_retreat.q = KDL::JntArray(state.arm_solver_retreat.num_joints);
        state.arm_solver_retreat.qd = KDL::JntArray(state.arm_solver_retreat.num_joints);
        state.arm_solver_retreat.qdd = KDL::JntArray(state.arm_solver_retreat.num_joints);
        state.arm_solver_retreat.tau_ff = KDL::JntArray(state.arm_solver_retreat.num_joints);
        state.arm_solver_retreat.tau_ctrl = KDL::JntArray(state.arm_solver_retreat.num_joints);
        state.arm_solver_retreat.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_retreat.num_spatial_directions = 6;
        state.arm_solver_retreat.spatial_directions = KDL::Jacobian(state.arm_solver_retreat.num_spatial_directions);
        state.arm_solver_retreat.acceleration_energy = KDL::JntArray(state.arm_solver_retreat.num_spatial_directions);
        state.arm_solver_retreat.f_ext = KDL::Wrenches(state.arm_solver_retreat.num_segments);
        state.arm_solver_retreat.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver_retreat.chain, state.arm_solver_retreat.root_acc, state.arm_solver_retreat.num_spatial_directions);
        state.arm_solver_retreat.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_retreat.chain, state.arm_solver_retreat.root_acc, state.arm_solver_retreat.num_spatial_directions);
        state.arm_solver_retreat.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_retreat.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_retreat.initialized = true;
    }
}

inline void update_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_retreat(state, robot);

    mj_kdl::update(robot.arm_solver_retreat.robot);
    for (int i = 0; i < state.arm_solver_retreat.num_joints; ++i) {
        state.arm_solver_retreat.q(i) = robot.arm_solver_retreat.robot->jnt_pos_msr[i];
        state.arm_solver_retreat.qd(i) = robot.arm_solver_retreat.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_retreat(state.arm_solver_retreat.q, state.arm_solver_retreat.qd);
    {
        KDL::Frame _body_frame_pose_cube_base;
        if (!mj_kdl::get_body_frame(
                robot.arm_solver_retreat.robot->model,
                robot.arm_solver_retreat.robot->data,
                "cube",
                &_body_frame_pose_cube_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm_solver_retreat;
        mj_kdl::get_body_frame(
                robot.arm_solver_retreat.robot->model,
                robot.arm_solver_retreat.robot->data,
                "base_link",
                &_base_world_frame_arm_solver_retreat);
        shared.pose_cube_base = _base_world_frame_arm_solver_retreat.Inverse() * _body_frame_pose_cube_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_retreat.chain);
        fk.JntToCart(
            state.arm_solver_retreat.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_retreat.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_retreat.chain);
        fk.JntToCart(
            state.arm_solver_retreat.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_retreat.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_retreat.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_retreat,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_retreat.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_retreat.robot->model,
                robot.arm_solver_retreat.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_retreat.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_retreat.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.retreat_support_z_add_out = shared.pose_elbow_base.p[2] + shared.retreat_support_lift;
        shared.retreat_support_z = shared.retreat_support_z_add_out;
        state.snapshot_taken = true;
    }
    shared.goal_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
}

inline bool can_start_motion_retreat(
    shared_data &shared
) {
    // eval_retreat_when_released
    shared.eval_retreat_when_released_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    return motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released_err, shared.satisfied_band_rot);
}

inline void monitor_when_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.goal_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    // eval_retreat_when_released
    shared.eval_retreat_when_released_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released_err, shared.satisfied_band_rot);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(11);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_single_fsm::E_RETREAT_READY);
            std::cerr << "[fsm] event   " << pick_place_single_fsm::EVENT_URIS[pick_place_single_fsm::E_RETREAT_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_retreat_until_at_retreat_settled
    shared.eval_retreat_until_at_retreat_settled_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_ee_base.p[2], shared.retreat_z_lo, shared.retreat_z_hi);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat_settled_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_settled_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_single_fsm::E_RETREAT_SETTLED);
            std::cerr << "[fsm] event   " << pick_place_single_fsm::EVENT_URIS[pick_place_single_fsm::E_RETREAT_SETTLED] << std::endl;
        }
    }

}

inline void monitor_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_retreat_when_released
    shared.eval_retreat_when_released_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released_err, shared.satisfied_band_rot);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(11);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_single_fsm::E_RETREAT_READY);
            std::cerr << "[fsm] event   " << pick_place_single_fsm::EVENT_URIS[pick_place_single_fsm::E_RETREAT_READY] << std::endl;
        }
    }

    // eval_retreat_until_at_retreat_settled
    shared.eval_retreat_until_at_retreat_settled_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_ee_base.p[2], shared.retreat_z_lo, shared.retreat_z_hi);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat_settled_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_settled_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_single_fsm::E_RETREAT_SETTLED);
            std::cerr << "[fsm] event   " << pick_place_single_fsm::EVENT_URIS[pick_place_single_fsm::E_RETREAT_SETTLED] << std::endl;
        }
    }

}

inline void control_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.retreat_place_x;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.retreat_place_y;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.retreat_place_above_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();

        shared.pose_ee_base_distance_x_err_retreat = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_retreat = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_retreat = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_ret_align_ori
    shared.pose_diff_ctrl_ret_align_ori = KDL::diff(shared.pose_ee_base, shared.goal_pose_retreat);
    shared.ctrl_ret_align_ori_err_ang_x = shared.pose_diff_ctrl_ret_align_ori.rot[0];
    shared.ctrl_ret_align_ori_err_ang_y = shared.pose_diff_ctrl_ret_align_ori.rot[1];
    shared.ctrl_ret_align_ori_err_ang_z = shared.pose_diff_ctrl_ret_align_ori.rot[2];
    // eval_retreat_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.retreat_support_z, shared.pose_elbow_base.p[2]);
    // compute_wrench_force_ctrl_ret_support_z
    shared.wrench_force_ctrl_ret_support_z = KDL::Wrench(shared.direction_ctrl_ret_support_z * shared.force_ctrl_ret_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_ret_support_z);
    // eval_retreat_while_open_gripper
    shared.gripper_pos_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper_pos);
    // ctrl_ret_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_ret_open_gripper = _control_signal;
    }
    // ctrl_ret_support_z
    {
        const double _control_signal = state.ctrl_ret_support_z.control(shared.pose_elbow_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret_support_z_stiffness, shared.ctrl_ret_support_z_damping, shared.ctrl_ret_support_z_integral_gain});
        shared.force_ctrl_ret_support_z = _control_signal;
        shared.ctrl_ret_support_z_error_integral = state.ctrl_ret_support_z.error_integral();
        shared.ctrl_ret_support_z_previous_error = state.ctrl_ret_support_z.previous_error();
        shared.ctrl_ret_support_z_first_sample = state.ctrl_ret_support_z.is_first_sample();
    }
    // ctrl_ret_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_ret_align_ori_ang_z.control(shared.pose_diff_ctrl_ret_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_ret_align_ori_ang_z_kp, shared.ctrl_ret_align_ori_ang_z_ki, shared.ctrl_ret_align_ori_ang_z_kd, shared.ctrl_ret_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_ret_align_ori_ang_z = _control_signal;
        shared.ctrl_ret_align_ori_ang_z_error_integral = state.ctrl_ret_align_ori_ang_z.error_integral();
        shared.ctrl_ret_align_ori_ang_z_previous_error = state.ctrl_ret_align_ori_ang_z.previous_error();
        shared.ctrl_ret_align_ori_ang_z_first_sample = state.ctrl_ret_align_ori_ang_z.is_first_sample();
    }
    // ctrl_ret_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_ret_align_ori_ang_y.control(shared.pose_diff_ctrl_ret_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_ret_align_ori_ang_y_kp, shared.ctrl_ret_align_ori_ang_y_ki, shared.ctrl_ret_align_ori_ang_y_kd, shared.ctrl_ret_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_ret_align_ori_ang_y = _control_signal;
        shared.ctrl_ret_align_ori_ang_y_error_integral = state.ctrl_ret_align_ori_ang_y.error_integral();
        shared.ctrl_ret_align_ori_ang_y_previous_error = state.ctrl_ret_align_ori_ang_y.previous_error();
        shared.ctrl_ret_align_ori_ang_y_first_sample = state.ctrl_ret_align_ori_ang_y.is_first_sample();
    }
    // ctrl_ret_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_ret_align_ori_ang_x.control(shared.pose_diff_ctrl_ret_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_ret_align_ori_ang_x_kp, shared.ctrl_ret_align_ori_ang_x_ki, shared.ctrl_ret_align_ori_ang_x_kd, shared.ctrl_ret_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_ret_align_ori_ang_x = _control_signal;
        shared.ctrl_ret_align_ori_ang_x_error_integral = state.ctrl_ret_align_ori_ang_x.error_integral();
        shared.ctrl_ret_align_ori_ang_x_previous_error = state.ctrl_ret_align_ori_ang_x.previous_error();
        shared.ctrl_ret_align_ori_ang_x_first_sample = state.ctrl_ret_align_ori_ang_x.is_first_sample();
    }
    // ctrl_ret_reach_z
    {
        const double _control_signal = state.ctrl_ret_reach_z.control(shared.pose_ee_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret_reach_z_kp, shared.ctrl_ret_reach_z_ki, shared.ctrl_ret_reach_z_kd, shared.ctrl_ret_reach_z_decay_rate});
        shared.eacc_pose_ee_base_distance_z_retreat = _control_signal;
        shared.ctrl_ret_reach_z_error_integral = state.ctrl_ret_reach_z.error_integral();
        shared.ctrl_ret_reach_z_previous_error = state.ctrl_ret_reach_z.previous_error();
        shared.ctrl_ret_reach_z_first_sample = state.ctrl_ret_reach_z.is_first_sample();
    }
    // ctrl_ret_reach_y
    {
        const double _control_signal = state.ctrl_ret_reach_y.control(shared.pose_ee_base_distance_y_err_retreat, shared.dt_measured_s, {shared.ctrl_ret_reach_y_kp, shared.ctrl_ret_reach_y_ki, shared.ctrl_ret_reach_y_kd, shared.ctrl_ret_reach_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_retreat = _control_signal;
        shared.ctrl_ret_reach_y_error_integral = state.ctrl_ret_reach_y.error_integral();
        shared.ctrl_ret_reach_y_previous_error = state.ctrl_ret_reach_y.previous_error();
        shared.ctrl_ret_reach_y_first_sample = state.ctrl_ret_reach_y.is_first_sample();
    }
    // ctrl_ret_reach_x
    {
        const double _control_signal = state.ctrl_ret_reach_x.control(shared.pose_ee_base_distance_x_err_retreat, shared.dt_measured_s, {shared.ctrl_ret_reach_x_kp, shared.ctrl_ret_reach_x_ki, shared.ctrl_ret_reach_x_kd, shared.ctrl_ret_reach_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_retreat = _control_signal;
        shared.ctrl_ret_reach_x_error_integral = state.ctrl_ret_reach_x.error_integral();
        shared.ctrl_ret_reach_x_previous_error = state.ctrl_ret_reach_x.previous_error();
        shared.ctrl_ret_reach_x_first_sample = state.ctrl_ret_reach_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_retreat.spatial_directions);

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_retreat.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_retreat;

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_retreat.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_retreat;

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_retreat.acceleration_energy(2) = shared.eacc_pose_ee_base_distance_z_retreat;

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_retreat.acceleration_energy(3) = shared.eacc_ctrl_ret_align_ori_ang_x;

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_retreat.acceleration_energy(4) = shared.eacc_ctrl_ret_align_ori_ang_y;

    state.arm_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_retreat.acceleration_energy(5) = shared.eacc_ctrl_ret_align_ori_ang_z;

    KDL::SetToZero(state.arm_solver_retreat.tau_ff);

    for (int i = 0; i < state.arm_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver_retreat.f_ext[i]);
    }

    state.arm_solver_retreat.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver_retreat.chain, "half_arm_2_link", "base_link") - 1] += shared.wrench_force_ctrl_ret_support_z;

    KDL::Wrenches f_ext_zero_arm_solver_retreat(state.arm_solver_retreat.num_segments);
    for (int i = 0; i < state.arm_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_retreat[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_retreat(state.arm_solver_retreat.num_joints);
    state.arm_solver_retreat.achd_acc->CartToJnt(
        state.arm_solver_retreat.q,
        state.arm_solver_retreat.qd,
        state.arm_solver_retreat.qdd,
        state.arm_solver_retreat.spatial_directions,
        state.arm_solver_retreat.acceleration_energy,
        state.arm_solver_retreat.f_ext,
        state.arm_solver_retreat.tau_ff,
        tau_ctrl_acc_arm_solver_retreat);
    state.arm_solver_retreat.rnea->CartToJnt(
        state.arm_solver_retreat.q,
        state.arm_solver_retreat.qd,
        state.arm_solver_retreat.qdd,
        f_ext_zero_arm_solver_retreat,
        state.arm_solver_retreat.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_retreat.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_retreat.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_retreat.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_retreat.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_retreat.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_retreat.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_retreat.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_retreat.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_retreat.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_retreat.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_retreat.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_retreat.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_retreat.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_retreat.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_retreat.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_retreat.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_retreat.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_retreat.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_retreat.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_retreat.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_retreat.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_retreat.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_retreat.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_retreat.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_retreat.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_retreat.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_retreat.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_retreat.tau_ctrl(6);

}

inline void apply_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_retreat.num_joints; ++i) {
        robot.arm_solver_retreat.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_retreat.tau_ctrl(i), i);
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
            robot.arm_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_ret_open_gripper;
        }
    }

}
