/// Motion: forward
/// Move the TCP forward five centimetres
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_forward_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_forward_solver_state arm_solver_forward;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_move_forward;
    motion_spec::runtime::PIDControl ctrl_forward_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_forward_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_forward_ori_ang_z;
    bool mon_forward_done_previous = false;
    bool mon_forward_done_event_triggered = false;

};

inline void reset_motion_forward(motion_forward_state &state) {
    state = motion_forward_state{};
}

inline void init_motion_forward(motion_forward_state &state, const robot_io &robot) {
    if (!state.arm_solver_forward.initialized) {
        state.arm_solver_forward.num_joints = robot.arm_solver_forward.chain->getNrOfJoints();
        state.arm_solver_forward.num_segments = robot.arm_solver_forward.chain->getNrOfSegments();
        state.arm_solver_forward.q = KDL::JntArray(state.arm_solver_forward.num_joints);
        state.arm_solver_forward.qd = KDL::JntArray(state.arm_solver_forward.num_joints);
        state.arm_solver_forward.qdd = KDL::JntArray(state.arm_solver_forward.num_joints);
        state.arm_solver_forward.tau_ff = KDL::JntArray(state.arm_solver_forward.num_joints);
        state.arm_solver_forward.tau_ctrl = KDL::JntArray(state.arm_solver_forward.num_joints);
        state.arm_solver_forward.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_forward.num_spatial_directions = 4;
        state.arm_solver_forward.spatial_directions = KDL::Jacobian(state.arm_solver_forward.num_spatial_directions);
        state.arm_solver_forward.acceleration_energy = KDL::JntArray(state.arm_solver_forward.num_spatial_directions);
        state.arm_solver_forward.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_forward.chain, state.arm_solver_forward.root_acc, state.arm_solver_forward.num_spatial_directions);
        state.arm_solver_forward.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_forward.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_forward.initialized = true;
    }
}

inline void update_motion_forward(
    motion_forward_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_forward(state, robot);
    if (robot.ext_force != nullptr) {
        shared.ext_force = *robot.ext_force;
    }

    mj_kdl::update(robot.arm_solver_forward.robot);
    for (int i = 0; i < state.arm_solver_forward.num_joints; ++i) {
        state.arm_solver_forward.q(i) = robot.arm_solver_forward.robot->jnt_pos_msr[i];
        state.arm_solver_forward.qd(i) = robot.arm_solver_forward.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_forward(state.arm_solver_forward.q, state.arm_solver_forward.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_forward.chain);
        fk.JntToCart(
            state.arm_solver_forward.q,
            shared.pose_bracelet_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_forward.chain, "bracelet_link", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_forward.chain);
        fk.JntToCart(
            state.arm_solver_forward.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_forward.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_forward.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_forward,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_forward.chain, "bracelet_link", "base_link"));
        shared.twist_bracelet_base = tmp.deriv();
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_forward.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_forward,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_forward.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        const mj_kdl::ForceTorqueSensor *ft_ext_force = mj_kdl::find_ft_sensor(robot.arm_solver_forward.robot, "wrist_ft");
        if (!ft_ext_force) {
            throw std::runtime_error("FT sensor 'wrist_ft' is unavailable");
        }
        const auto frame_ext_force = [&](const char *name) {
            KDL::Frame frame;
            if (!mj_kdl::get_site_frame(robot.arm_solver_forward.robot->model, robot.arm_solver_forward.robot->data, name, &frame)
                && !mj_kdl::get_body_frame(robot.arm_solver_forward.robot->model, robot.arm_solver_forward.robot->data, name, &frame)) {
                throw std::runtime_error(std::string("FT wrench frame '") + name + "' is unavailable");
            }
            return frame;
        };
        const KDL::Wrench measured_ext_force = motion_spec::runtime::transform_wrench(
            ft_ext_force->wrench,
            frame_ext_force("wrist_ft_site"),
            frame_ext_force("wrist_ft_site"),
            frame_ext_force("base_link"));
        if (shared.ext_force_ft_settle < robot.ft_wrist_ft_bias_samples) {
            ++shared.ext_force_ft_settle;
            shared.ext_force_ft_bias = measured_ext_force;   // last settle step is the tare sample
            shared.ext_force = KDL::Wrench::Zero();
        } else {
            shared.ext_force = shared.ext_force_ft_bias - measured_ext_force;
        }
    }

    if (!state.snapshot_taken) {
        shared.forward_start_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_forward() {
    return true;
}

inline void monitor_when_motion_forward() {
}

inline void monitor_until_motion_forward(
    motion_forward_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // target_x_add
    shared.target_x = shared.forward_start_pose.p[0] + shared.forward_distance;
    // eval_forward_until_reached_forward
    shared.eval_forward_until_reached_forward_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee_base.p[0], shared.target_x);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_forward_until_reached_forward_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_forward_done_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(6);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORWARD_DONE);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORWARD_DONE] << std::endl;
        }
    }

}

inline void monitor_motion_forward(
    motion_forward_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // target_x_add
    shared.target_x = shared.forward_start_pose.p[0] + shared.forward_distance;
    // eval_forward_until_reached_forward
    shared.eval_forward_until_reached_forward_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee_base.p[0], shared.target_x);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_forward_until_reached_forward_err, shared.default_tolerance_Distance);
        const bool detected = motion_spec::runtime::rising_edge(state.mon_forward_done_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(6);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORWARD_DONE);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORWARD_DONE] << std::endl;
        }
    }

}

inline void control_motion_forward(
    motion_forward_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_forward_ori
    shared.pose_diff_ctrl_forward_ori = KDL::diff(shared.pose_ee_base, shared.forward_start_pose);
    shared.ctrl_forward_ori_err_ang_x = shared.pose_diff_ctrl_forward_ori.rot[0];
    shared.ctrl_forward_ori_err_ang_y = shared.pose_diff_ctrl_forward_ori.rot[1];
    shared.ctrl_forward_ori_err_ang_z = shared.pose_diff_ctrl_forward_ori.rot[2];
    // eval_forward_while_move_forward
    shared.pose_ee_base_distance_x_err_forward = motion_spec::runtime::evaluate_equality_constraint(shared.target_x, shared.pose_ee_base.p[0]);
    // ctrl_forward_ori_ang_z
    {
        const double _control_signal = state.ctrl_forward_ori_ang_z.control(shared.pose_diff_ctrl_forward_ori.rot[2], shared.dt_measured_s, {shared.ctrl_forward_ori_ang_z_kp, shared.ctrl_forward_ori_ang_z_ki, shared.ctrl_forward_ori_ang_z_kd, shared.ctrl_forward_ori_ang_z_decay_rate});
        shared.eacc_ctrl_forward_ori_ang_z = _control_signal;
        shared.ctrl_forward_ori_ang_z_error_integral = state.ctrl_forward_ori_ang_z.error_integral();
        shared.ctrl_forward_ori_ang_z_previous_error = state.ctrl_forward_ori_ang_z.previous_error();
        shared.ctrl_forward_ori_ang_z_first_sample = state.ctrl_forward_ori_ang_z.is_first_sample();
    }
    // ctrl_forward_ori_ang_y
    {
        const double _control_signal = state.ctrl_forward_ori_ang_y.control(shared.pose_diff_ctrl_forward_ori.rot[1], shared.dt_measured_s, {shared.ctrl_forward_ori_ang_y_kp, shared.ctrl_forward_ori_ang_y_ki, shared.ctrl_forward_ori_ang_y_kd, shared.ctrl_forward_ori_ang_y_decay_rate});
        shared.eacc_ctrl_forward_ori_ang_y = _control_signal;
        shared.ctrl_forward_ori_ang_y_error_integral = state.ctrl_forward_ori_ang_y.error_integral();
        shared.ctrl_forward_ori_ang_y_previous_error = state.ctrl_forward_ori_ang_y.previous_error();
        shared.ctrl_forward_ori_ang_y_first_sample = state.ctrl_forward_ori_ang_y.is_first_sample();
    }
    // ctrl_forward_ori_ang_x
    {
        const double _control_signal = state.ctrl_forward_ori_ang_x.control(shared.pose_diff_ctrl_forward_ori.rot[0], shared.dt_measured_s, {shared.ctrl_forward_ori_ang_x_kp, shared.ctrl_forward_ori_ang_x_ki, shared.ctrl_forward_ori_ang_x_kd, shared.ctrl_forward_ori_ang_x_decay_rate});
        shared.eacc_ctrl_forward_ori_ang_x = _control_signal;
        shared.ctrl_forward_ori_ang_x_error_integral = state.ctrl_forward_ori_ang_x.error_integral();
        shared.ctrl_forward_ori_ang_x_previous_error = state.ctrl_forward_ori_ang_x.previous_error();
        shared.ctrl_forward_ori_ang_x_first_sample = state.ctrl_forward_ori_ang_x.is_first_sample();
    }
    // ctrl_move_forward
    {
        const double _control_signal = state.ctrl_move_forward.control(shared.pose_ee_base_distance_x_err_forward, shared.dt_measured_s, {shared.ctrl_move_forward_kp, shared.ctrl_move_forward_ki, shared.ctrl_move_forward_kd, shared.ctrl_move_forward_decay_rate});
        shared.eacc_pose_ee_base_distance_x_forward = _control_signal;
        shared.ctrl_move_forward_error_integral = state.ctrl_move_forward.error_integral();
        shared.ctrl_move_forward_previous_error = state.ctrl_move_forward.previous_error();
        shared.ctrl_move_forward_first_sample = state.ctrl_move_forward.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_forward.spatial_directions);

    state.arm_solver_forward.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_forward.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_forward;

    state.arm_solver_forward.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 1) = 1.0;

    state.arm_solver_forward.acceleration_energy(1) = shared.eacc_ctrl_forward_ori_ang_x;

    state.arm_solver_forward.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 2) = 1.0;

    state.arm_solver_forward.acceleration_energy(2) = shared.eacc_ctrl_forward_ori_ang_y;

    state.arm_solver_forward.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = 1.0;

    state.arm_solver_forward.acceleration_energy(3) = shared.eacc_ctrl_forward_ori_ang_z;

    KDL::SetToZero(state.arm_solver_forward.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_forward(state.arm_solver_forward.num_segments);
    for (int i = 0; i < state.arm_solver_forward.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_forward[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_forward(state.arm_solver_forward.num_joints);
    state.arm_solver_forward.achd_acc->CartToJnt(
        state.arm_solver_forward.q,
        state.arm_solver_forward.qd,
        state.arm_solver_forward.qdd,
        state.arm_solver_forward.spatial_directions,
        state.arm_solver_forward.acceleration_energy,
        f_ext_zero_arm_solver_forward,
        state.arm_solver_forward.tau_ff,
        tau_ctrl_acc_arm_solver_forward);
    state.arm_solver_forward.rnea->CartToJnt(
        state.arm_solver_forward.q,
        state.arm_solver_forward.qd,
        state.arm_solver_forward.qdd,
        f_ext_zero_arm_solver_forward,
        state.arm_solver_forward.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_forward.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_forward.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_forward.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_forward.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_forward.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_forward.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_forward.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_forward.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_forward.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_forward.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_forward.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_forward.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_forward.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_forward.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_forward.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_forward.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_forward.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_forward.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_forward.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_forward.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_forward.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_forward.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_forward.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_forward.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_forward.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_forward.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_forward.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_forward.tau_ctrl(6);

}

inline void apply_motion_forward(
    motion_forward_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_forward.num_joints; ++i) {
        robot.arm_solver_forward.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_forward.tau_ctrl(i), i);
    }

}
