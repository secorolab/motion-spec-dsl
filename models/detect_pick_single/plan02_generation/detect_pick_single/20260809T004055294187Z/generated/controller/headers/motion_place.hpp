/// Motion: place
/// Lower TCP with cube straight down to the place height
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_place_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_place_solver_state arm_solver_place;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pl_reach_x;
    motion_spec::runtime::PIDControl ctrl_pl_reach_y;
    motion_spec::runtime::PIDControl ctrl_pl_reach_z;
    motion_spec::runtime::PIDControl ctrl_pl_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pl_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pl_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pl_support_z;

    bool mon_place_ready_previous = false;
    bool mon_place_ready_event_triggered = false;

    bool mon_place_grasp_lost_previous = false;
    bool mon_place_grasp_lost_event_triggered = false;

};

inline void reset_motion_place(motion_place_state &state) {
    state = motion_place_state{};
}

inline void init_motion_place(motion_place_state &state, const robot_io &robot) {
    if (!state.arm_solver_place.initialized) {
        state.arm_solver_place.num_joints = robot.arm_solver_place.chain->getNrOfJoints();
        state.arm_solver_place.num_segments = robot.arm_solver_place.chain->getNrOfSegments();
        state.arm_solver_place.q = KDL::JntArray(state.arm_solver_place.num_joints);
        state.arm_solver_place.qd = KDL::JntArray(state.arm_solver_place.num_joints);
        state.arm_solver_place.qdd = KDL::JntArray(state.arm_solver_place.num_joints);
        state.arm_solver_place.tau_ff = KDL::JntArray(state.arm_solver_place.num_joints);
        state.arm_solver_place.tau_ctrl = KDL::JntArray(state.arm_solver_place.num_joints);
        state.arm_solver_place.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_place.num_spatial_directions = 6;
        state.arm_solver_place.spatial_directions = KDL::Jacobian(state.arm_solver_place.num_spatial_directions);
        state.arm_solver_place.acceleration_energy = KDL::JntArray(state.arm_solver_place.num_spatial_directions);
        state.arm_solver_place.f_ext = KDL::Wrenches(state.arm_solver_place.num_segments);
        state.arm_solver_place.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver_place.chain, state.arm_solver_place.root_acc, state.arm_solver_place.num_spatial_directions);
        state.arm_solver_place.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_place.chain, state.arm_solver_place.root_acc, state.arm_solver_place.num_spatial_directions);
        state.arm_solver_place.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_place.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_place.initialized = true;
    }
}

inline void update_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_place(state, robot);

    mj_kdl::update(robot.arm_solver_place.robot);
    for (int i = 0; i < state.arm_solver_place.num_joints; ++i) {
        state.arm_solver_place.q(i) = robot.arm_solver_place.robot->jnt_pos_msr[i];
        state.arm_solver_place.qd(i) = robot.arm_solver_place.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_place(state.arm_solver_place.q, state.arm_solver_place.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_place.chain);
        fk.JntToCart(
            state.arm_solver_place.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_place.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_place.chain);
        fk.JntToCart(
            state.arm_solver_place.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_place.chain, "half_arm_2_link", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_place.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_place,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_place.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        double _joint_position_gripper_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm_solver_place.robot->model,
                robot.arm_solver_place.robot->data,
                "g_left_driver_joint",
                &_joint_position_gripper_pos)) {
            shared.gripper_pos = _joint_position_gripper_pos;
        } else {
            shared.gripper_pos = state.arm_solver_place.q(motion_spec::runtime::find_joint_index(*robot.arm_solver_place.chain, "g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.place_support_z_add_out = shared.pose_elbow_base.p[2] + shared.place_support_lift;
        shared.place_support_z = shared.place_support_z_add_out;
        state.snapshot_taken = true;
    }
    shared.goal_pose_place = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.03));
}

inline bool can_start_motion_place(
    shared_data &shared
) {
    // eval_place_when_at_pre_place
    shared.eval_place_when_at_pre_place_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee_base.p[1], shared.place_y_gate);

    return motion_spec::runtime::constraint_satisfied(shared.eval_place_when_at_pre_place_err, shared.default_tolerance_Distance);
}

inline void monitor_when_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.goal_pose_place = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.03));
    // eval_place_when_at_pre_place
    shared.eval_place_when_at_pre_place_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee_base.p[1], shared.place_y_gate);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_place_when_at_pre_place_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_PLACE_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_PLACE_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_place_until_grasp_lost
    shared.eval_place_until_grasp_lost_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper_pos, shared.lost_width);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_place_until_grasp_lost_err, shared.default_tolerance_Angle);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_GRASP_LOST_PLACE);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_GRASP_LOST_PLACE] << std::endl;
        }
    }

}

inline void monitor_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_place_when_at_pre_place
    shared.eval_place_when_at_pre_place_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee_base.p[1], shared.place_y_gate);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_place_when_at_pre_place_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_PLACE_READY);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_PLACE_READY] << std::endl;
        }
    }

    // eval_place_until_grasp_lost
    shared.eval_place_until_grasp_lost_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper_pos, shared.lost_width);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_place_until_grasp_lost_err, shared.default_tolerance_Angle);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_GRASP_LOST_PLACE);
            std::cerr << "[fsm] event   " << detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_GRASP_LOST_PLACE] << std::endl;
        }
    }

}

inline void control_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.place_place_x;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.place_place_y;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.place_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();

        shared.pose_ee_base_distance_x_err_place = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_place = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_place = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_pl_align_ori
    shared.pose_diff_ctrl_pl_align_ori = KDL::diff(shared.pose_ee_base, shared.goal_pose_place);
    shared.ctrl_pl_align_ori_err_ang_x = shared.pose_diff_ctrl_pl_align_ori.rot[0];
    shared.ctrl_pl_align_ori_err_ang_y = shared.pose_diff_ctrl_pl_align_ori.rot[1];
    shared.ctrl_pl_align_ori_err_ang_z = shared.pose_diff_ctrl_pl_align_ori.rot[2];
    // eval_place_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_place = motion_spec::runtime::evaluate_equality_constraint(shared.place_support_z, shared.pose_elbow_base.p[2]);
    // compute_wrench_force_ctrl_pl_support_z
    shared.wrench_force_ctrl_pl_support_z = KDL::Wrench(shared.direction_ctrl_pl_support_z * shared.force_ctrl_pl_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pl_support_z);
    // eval_place_while_close_gripper
    shared.gripper_pos_err_place = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper_pos);
    // ctrl_pl_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_pl_close_gripper = _control_signal;
    }
    // ctrl_pl_support_z
    {
        const double _control_signal = state.ctrl_pl_support_z.control(shared.pose_elbow_base_distance_z_err_place, shared.dt_measured_s, {shared.ctrl_pl_support_z_stiffness, shared.ctrl_pl_support_z_damping, shared.ctrl_pl_support_z_integral_gain});
        shared.force_ctrl_pl_support_z = _control_signal;
        shared.ctrl_pl_support_z_error_integral = state.ctrl_pl_support_z.error_integral();
        shared.ctrl_pl_support_z_previous_error = state.ctrl_pl_support_z.previous_error();
        shared.ctrl_pl_support_z_first_sample = state.ctrl_pl_support_z.is_first_sample();
    }
    // ctrl_pl_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_pl_align_ori_ang_z.control(shared.pose_diff_ctrl_pl_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pl_align_ori_ang_z_kp, shared.ctrl_pl_align_ori_ang_z_ki, shared.ctrl_pl_align_ori_ang_z_kd, shared.ctrl_pl_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pl_align_ori_ang_z = _control_signal;
        shared.ctrl_pl_align_ori_ang_z_error_integral = state.ctrl_pl_align_ori_ang_z.error_integral();
        shared.ctrl_pl_align_ori_ang_z_previous_error = state.ctrl_pl_align_ori_ang_z.previous_error();
        shared.ctrl_pl_align_ori_ang_z_first_sample = state.ctrl_pl_align_ori_ang_z.is_first_sample();
    }
    // ctrl_pl_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_pl_align_ori_ang_y.control(shared.pose_diff_ctrl_pl_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pl_align_ori_ang_y_kp, shared.ctrl_pl_align_ori_ang_y_ki, shared.ctrl_pl_align_ori_ang_y_kd, shared.ctrl_pl_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pl_align_ori_ang_y = _control_signal;
        shared.ctrl_pl_align_ori_ang_y_error_integral = state.ctrl_pl_align_ori_ang_y.error_integral();
        shared.ctrl_pl_align_ori_ang_y_previous_error = state.ctrl_pl_align_ori_ang_y.previous_error();
        shared.ctrl_pl_align_ori_ang_y_first_sample = state.ctrl_pl_align_ori_ang_y.is_first_sample();
    }
    // ctrl_pl_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_pl_align_ori_ang_x.control(shared.pose_diff_ctrl_pl_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pl_align_ori_ang_x_kp, shared.ctrl_pl_align_ori_ang_x_ki, shared.ctrl_pl_align_ori_ang_x_kd, shared.ctrl_pl_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pl_align_ori_ang_x = _control_signal;
        shared.ctrl_pl_align_ori_ang_x_error_integral = state.ctrl_pl_align_ori_ang_x.error_integral();
        shared.ctrl_pl_align_ori_ang_x_previous_error = state.ctrl_pl_align_ori_ang_x.previous_error();
        shared.ctrl_pl_align_ori_ang_x_first_sample = state.ctrl_pl_align_ori_ang_x.is_first_sample();
    }
    // ctrl_pl_reach_z
    {
        const double _control_signal = state.ctrl_pl_reach_z.control(shared.pose_ee_base_distance_z_err_place, shared.dt_measured_s, {shared.ctrl_pl_reach_z_kp, shared.ctrl_pl_reach_z_ki, shared.ctrl_pl_reach_z_kd, shared.ctrl_pl_reach_z_decay_rate});
        shared.eacc_pose_ee_base_distance_z_place = _control_signal;
        shared.ctrl_pl_reach_z_error_integral = state.ctrl_pl_reach_z.error_integral();
        shared.ctrl_pl_reach_z_previous_error = state.ctrl_pl_reach_z.previous_error();
        shared.ctrl_pl_reach_z_first_sample = state.ctrl_pl_reach_z.is_first_sample();
    }
    // ctrl_pl_reach_y
    {
        const double _control_signal = state.ctrl_pl_reach_y.control(shared.pose_ee_base_distance_y_err_place, shared.dt_measured_s, {shared.ctrl_pl_reach_y_kp, shared.ctrl_pl_reach_y_ki, shared.ctrl_pl_reach_y_kd, shared.ctrl_pl_reach_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_place = _control_signal;
        shared.ctrl_pl_reach_y_error_integral = state.ctrl_pl_reach_y.error_integral();
        shared.ctrl_pl_reach_y_previous_error = state.ctrl_pl_reach_y.previous_error();
        shared.ctrl_pl_reach_y_first_sample = state.ctrl_pl_reach_y.is_first_sample();
    }
    // ctrl_pl_reach_x
    {
        const double _control_signal = state.ctrl_pl_reach_x.control(shared.pose_ee_base_distance_x_err_place, shared.dt_measured_s, {shared.ctrl_pl_reach_x_kp, shared.ctrl_pl_reach_x_ki, shared.ctrl_pl_reach_x_kd, shared.ctrl_pl_reach_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_place = _control_signal;
        shared.ctrl_pl_reach_x_error_integral = state.ctrl_pl_reach_x.error_integral();
        shared.ctrl_pl_reach_x_previous_error = state.ctrl_pl_reach_x.previous_error();
        shared.ctrl_pl_reach_x_first_sample = state.ctrl_pl_reach_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_place.spatial_directions);

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_place.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_place;

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_place.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_place;

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_place.acceleration_energy(2) = shared.eacc_pose_ee_base_distance_z_place;

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_place.acceleration_energy(3) = shared.eacc_ctrl_pl_align_ori_ang_x;

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_place.acceleration_energy(4) = shared.eacc_ctrl_pl_align_ori_ang_y;

    state.arm_solver_place.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_place.acceleration_energy(5) = shared.eacc_ctrl_pl_align_ori_ang_z;

    KDL::SetToZero(state.arm_solver_place.tau_ff);

    for (int i = 0; i < state.arm_solver_place.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver_place.f_ext[i]);
    }

    state.arm_solver_place.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver_place.chain, "half_arm_2_link", "base_link") - 1] += shared.wrench_force_ctrl_pl_support_z;

    KDL::Wrenches f_ext_zero_arm_solver_place(state.arm_solver_place.num_segments);
    for (int i = 0; i < state.arm_solver_place.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_place[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_place(state.arm_solver_place.num_joints);
    state.arm_solver_place.achd_acc->CartToJnt(
        state.arm_solver_place.q,
        state.arm_solver_place.qd,
        state.arm_solver_place.qdd,
        state.arm_solver_place.spatial_directions,
        state.arm_solver_place.acceleration_energy,
        state.arm_solver_place.f_ext,
        state.arm_solver_place.tau_ff,
        tau_ctrl_acc_arm_solver_place);
    state.arm_solver_place.rnea->CartToJnt(
        state.arm_solver_place.q,
        state.arm_solver_place.qd,
        state.arm_solver_place.qdd,
        f_ext_zero_arm_solver_place,
        state.arm_solver_place.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_place.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_place.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_place.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_place.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_place.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_place.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_place.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_place.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_place.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_place.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_place.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_place.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_place.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_place.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_place.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_place.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_place.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_place.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_place.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_place.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_place.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_place.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_place.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_place.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_place.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_place.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_place.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_place.tau_ctrl(6);

}

inline void apply_motion_place(
    motion_place_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_place.num_joints; ++i) {
        robot.arm_solver_place.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_place.tau_ctrl(i), i);
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
            robot.arm_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_pl_close_gripper;
        }
    }

}
