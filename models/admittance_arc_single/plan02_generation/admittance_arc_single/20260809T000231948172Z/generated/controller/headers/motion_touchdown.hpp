/// Motion: touchdown
/// Lower the TCP until it contacts the table
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_touchdown_state {
    bool active = false;
    int active_steps = 0;
    arm_solver_touchdown_solver_state arm_solver_touchdown;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_touchdown_x;
    motion_spec::runtime::PIDControl ctrl_touchdown_y;
    motion_spec::runtime::PIDControl ctrl_touchdown_z;
    motion_spec::runtime::PIDControl ctrl_touchdown_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_touchdown_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_touchdown_ori_ang_z;
    bool mon_contact_previous = false;
    bool mon_contact_event_triggered = false;
    double mon_contact_hold_s = 0.0;

};

inline void reset_motion_touchdown(motion_touchdown_state &state) {
    state = motion_touchdown_state{};
}

inline void init_motion_touchdown(motion_touchdown_state &state, const robot_io &robot) {
    if (!state.arm_solver_touchdown.initialized) {
        state.arm_solver_touchdown.num_joints = robot.arm_solver_touchdown.chain->getNrOfJoints();
        state.arm_solver_touchdown.num_segments = robot.arm_solver_touchdown.chain->getNrOfSegments();
        state.arm_solver_touchdown.q = KDL::JntArray(state.arm_solver_touchdown.num_joints);
        state.arm_solver_touchdown.qd = KDL::JntArray(state.arm_solver_touchdown.num_joints);
        state.arm_solver_touchdown.qdd = KDL::JntArray(state.arm_solver_touchdown.num_joints);
        state.arm_solver_touchdown.tau_ff = KDL::JntArray(state.arm_solver_touchdown.num_joints);
        state.arm_solver_touchdown.tau_ctrl = KDL::JntArray(state.arm_solver_touchdown.num_joints);
        state.arm_solver_touchdown.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm_solver_touchdown.num_spatial_directions = 6;
        state.arm_solver_touchdown.spatial_directions = KDL::Jacobian(state.arm_solver_touchdown.num_spatial_directions);
        state.arm_solver_touchdown.acceleration_energy = KDL::JntArray(state.arm_solver_touchdown.num_spatial_directions);
        state.arm_solver_touchdown.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver_touchdown.chain, state.arm_solver_touchdown.root_acc, state.arm_solver_touchdown.num_spatial_directions);
        state.arm_solver_touchdown.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver_touchdown.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver_touchdown.initialized = true;
    }
}

inline void update_motion_touchdown(
    motion_touchdown_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_touchdown(state, robot);
    if (robot.ext_force != nullptr) {
        shared.ext_force = *robot.ext_force;
    }

    mj_kdl::update(robot.arm_solver_touchdown.robot);
    for (int i = 0; i < state.arm_solver_touchdown.num_joints; ++i) {
        state.arm_solver_touchdown.q(i) = robot.arm_solver_touchdown.robot->jnt_pos_msr[i];
        state.arm_solver_touchdown.qd(i) = robot.arm_solver_touchdown.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver_touchdown(state.arm_solver_touchdown.q, state.arm_solver_touchdown.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_touchdown.chain);
        fk.JntToCart(
            state.arm_solver_touchdown.q,
            shared.pose_bracelet_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_touchdown.chain, "bracelet_link", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver_touchdown.chain);
        fk.JntToCart(
            state.arm_solver_touchdown.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_touchdown.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_touchdown.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_touchdown,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_touchdown.chain, "bracelet_link", "base_link"));
        shared.twist_bracelet_base = tmp.deriv();
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver_touchdown.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver_touchdown,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver_touchdown.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        const mj_kdl::ForceTorqueSensor *ft_ext_force = mj_kdl::find_ft_sensor(robot.arm_solver_touchdown.robot, "wrist_ft");
        if (!ft_ext_force) {
            throw std::runtime_error("FT sensor 'wrist_ft' is unavailable");
        }
        const auto frame_ext_force = [&](const char *name) {
            KDL::Frame frame;
            if (!mj_kdl::get_site_frame(robot.arm_solver_touchdown.robot->model, robot.arm_solver_touchdown.robot->data, name, &frame)
                && !mj_kdl::get_body_frame(robot.arm_solver_touchdown.robot->model, robot.arm_solver_touchdown.robot->data, name, &frame)) {
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
        shared.touchdown_start_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_touchdown() {
    return true;
}

inline void monitor_when_motion_touchdown() {
}

inline void monitor_until_motion_touchdown(
    motion_touchdown_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_touchdown_until_contact_table
    shared.eval_touchdown_until_contact_table_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_ee_base.vel[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_touchdown_until_contact_table_err, shared.satisfied_band_vel);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_contact_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(3);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_CONTACT] << std::endl;
        }
    }

}

inline void monitor_motion_touchdown(
    motion_touchdown_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_touchdown_until_contact_table
    shared.eval_touchdown_until_contact_table_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_ee_base.vel[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_touchdown_until_contact_table_err, shared.satisfied_band_vel);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_contact_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(3);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_CONTACT] << std::endl;
        }
    }

}

inline void control_motion_touchdown(
    motion_touchdown_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.touchdown_start_pose.p[0];
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.touchdown_start_pose.p[1];
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();

        shared.pose_ee_base_distance_x_err_touchdown = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_touchdown = _pose_axis_error_linear_Y;
    }
    // eval_pose_diff_ctrl_touchdown_ori
    shared.pose_diff_ctrl_touchdown_ori = KDL::diff(shared.pose_ee_base, shared.touchdown_start_pose);
    shared.ctrl_touchdown_ori_err_ang_x = shared.pose_diff_ctrl_touchdown_ori.rot[0];
    shared.ctrl_touchdown_ori_err_ang_y = shared.pose_diff_ctrl_touchdown_ori.rot[1];
    shared.ctrl_touchdown_ori_err_ang_z = shared.pose_diff_ctrl_touchdown_ori.rot[2];
    // eval_touchdown_while_lower_z
    shared.twist_ee_base_linear_z_err_touchdown = motion_spec::runtime::evaluate_equality_constraint(shared.descend_vel, shared.twist_ee_base.vel[2]);
    // ctrl_touchdown_ori_ang_z
    {
        const double _control_signal = state.ctrl_touchdown_ori_ang_z.control(shared.pose_diff_ctrl_touchdown_ori.rot[2], shared.dt_measured_s, {shared.ctrl_touchdown_ori_ang_z_kp, shared.ctrl_touchdown_ori_ang_z_ki, shared.ctrl_touchdown_ori_ang_z_kd, shared.ctrl_touchdown_ori_ang_z_decay_rate});
        shared.eacc_ctrl_touchdown_ori_ang_z = _control_signal;
        shared.ctrl_touchdown_ori_ang_z_error_integral = state.ctrl_touchdown_ori_ang_z.error_integral();
        shared.ctrl_touchdown_ori_ang_z_previous_error = state.ctrl_touchdown_ori_ang_z.previous_error();
        shared.ctrl_touchdown_ori_ang_z_first_sample = state.ctrl_touchdown_ori_ang_z.is_first_sample();
    }
    // ctrl_touchdown_ori_ang_y
    {
        const double _control_signal = state.ctrl_touchdown_ori_ang_y.control(shared.pose_diff_ctrl_touchdown_ori.rot[1], shared.dt_measured_s, {shared.ctrl_touchdown_ori_ang_y_kp, shared.ctrl_touchdown_ori_ang_y_ki, shared.ctrl_touchdown_ori_ang_y_kd, shared.ctrl_touchdown_ori_ang_y_decay_rate});
        shared.eacc_ctrl_touchdown_ori_ang_y = _control_signal;
        shared.ctrl_touchdown_ori_ang_y_error_integral = state.ctrl_touchdown_ori_ang_y.error_integral();
        shared.ctrl_touchdown_ori_ang_y_previous_error = state.ctrl_touchdown_ori_ang_y.previous_error();
        shared.ctrl_touchdown_ori_ang_y_first_sample = state.ctrl_touchdown_ori_ang_y.is_first_sample();
    }
    // ctrl_touchdown_ori_ang_x
    {
        const double _control_signal = state.ctrl_touchdown_ori_ang_x.control(shared.pose_diff_ctrl_touchdown_ori.rot[0], shared.dt_measured_s, {shared.ctrl_touchdown_ori_ang_x_kp, shared.ctrl_touchdown_ori_ang_x_ki, shared.ctrl_touchdown_ori_ang_x_kd, shared.ctrl_touchdown_ori_ang_x_decay_rate});
        shared.eacc_ctrl_touchdown_ori_ang_x = _control_signal;
        shared.ctrl_touchdown_ori_ang_x_error_integral = state.ctrl_touchdown_ori_ang_x.error_integral();
        shared.ctrl_touchdown_ori_ang_x_previous_error = state.ctrl_touchdown_ori_ang_x.previous_error();
        shared.ctrl_touchdown_ori_ang_x_first_sample = state.ctrl_touchdown_ori_ang_x.is_first_sample();
    }
    // ctrl_touchdown_z
    {
        const double _control_signal = state.ctrl_touchdown_z.control(shared.twist_ee_base_linear_z_err_touchdown, shared.dt_measured_s, {shared.ctrl_touchdown_z_kp, shared.ctrl_touchdown_z_ki, shared.ctrl_touchdown_z_kd, shared.ctrl_touchdown_z_decay_rate});
        shared.eacc_twist_ee_base_linear_z_touchdown = _control_signal;
        shared.ctrl_touchdown_z_error_integral = state.ctrl_touchdown_z.error_integral();
        shared.ctrl_touchdown_z_previous_error = state.ctrl_touchdown_z.previous_error();
        shared.ctrl_touchdown_z_first_sample = state.ctrl_touchdown_z.is_first_sample();
    }
    // ctrl_touchdown_y
    {
        const double _control_signal = state.ctrl_touchdown_y.control(shared.pose_ee_base_distance_y_err_touchdown, shared.dt_measured_s, {shared.ctrl_touchdown_y_kp, shared.ctrl_touchdown_y_ki, shared.ctrl_touchdown_y_kd, shared.ctrl_touchdown_y_decay_rate});
        shared.eacc_pose_ee_base_distance_y_touchdown = _control_signal;
        shared.ctrl_touchdown_y_error_integral = state.ctrl_touchdown_y.error_integral();
        shared.ctrl_touchdown_y_previous_error = state.ctrl_touchdown_y.previous_error();
        shared.ctrl_touchdown_y_first_sample = state.ctrl_touchdown_y.is_first_sample();
    }
    // ctrl_touchdown_x
    {
        const double _control_signal = state.ctrl_touchdown_x.control(shared.pose_ee_base_distance_x_err_touchdown, shared.dt_measured_s, {shared.ctrl_touchdown_x_kp, shared.ctrl_touchdown_x_ki, shared.ctrl_touchdown_x_kd, shared.ctrl_touchdown_x_decay_rate});
        shared.eacc_pose_ee_base_distance_x_touchdown = _control_signal;
        shared.ctrl_touchdown_x_error_integral = state.ctrl_touchdown_x.error_integral();
        shared.ctrl_touchdown_x_previous_error = state.ctrl_touchdown_x.previous_error();
        shared.ctrl_touchdown_x_first_sample = state.ctrl_touchdown_x.is_first_sample();
    }

    KDL::SetToZero(state.arm_solver_touchdown.spatial_directions);

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(0) = shared.eacc_pose_ee_base_distance_x_touchdown;

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(1) = shared.eacc_pose_ee_base_distance_y_touchdown;

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(2) = shared.eacc_twist_ee_base_linear_z_touchdown;

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(3) = shared.eacc_ctrl_touchdown_ori_ang_x;

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(4) = shared.eacc_ctrl_touchdown_ori_ang_y;

    state.arm_solver_touchdown.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.arm_solver_touchdown.acceleration_energy(5) = shared.eacc_ctrl_touchdown_ori_ang_z;

    KDL::SetToZero(state.arm_solver_touchdown.tau_ff);

    KDL::Wrenches f_ext_zero_arm_solver_touchdown(state.arm_solver_touchdown.num_segments);
    for (int i = 0; i < state.arm_solver_touchdown.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm_solver_touchdown[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm_solver_touchdown(state.arm_solver_touchdown.num_joints);
    state.arm_solver_touchdown.achd_acc->CartToJnt(
        state.arm_solver_touchdown.q,
        state.arm_solver_touchdown.qd,
        state.arm_solver_touchdown.qdd,
        state.arm_solver_touchdown.spatial_directions,
        state.arm_solver_touchdown.acceleration_energy,
        f_ext_zero_arm_solver_touchdown,
        state.arm_solver_touchdown.tau_ff,
        tau_ctrl_acc_arm_solver_touchdown);
    state.arm_solver_touchdown.rnea->CartToJnt(
        state.arm_solver_touchdown.q,
        state.arm_solver_touchdown.qd,
        state.arm_solver_touchdown.qdd,
        f_ext_zero_arm_solver_touchdown,
        state.arm_solver_touchdown.tau_ctrl);
    shared.arm_solver_home_q_joint_1 = state.arm_solver_touchdown.q(0);
    shared.arm_solver_home_q_joint_2 = state.arm_solver_touchdown.q(1);
    shared.arm_solver_home_q_joint_3 = state.arm_solver_touchdown.q(2);
    shared.arm_solver_home_q_joint_4 = state.arm_solver_touchdown.q(3);
    shared.arm_solver_home_q_joint_5 = state.arm_solver_touchdown.q(4);
    shared.arm_solver_home_q_joint_6 = state.arm_solver_touchdown.q(5);
    shared.arm_solver_home_q_joint_7 = state.arm_solver_touchdown.q(6);
    shared.arm_solver_home_qd_joint_1 = state.arm_solver_touchdown.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.arm_solver_touchdown.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.arm_solver_touchdown.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.arm_solver_touchdown.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.arm_solver_touchdown.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.arm_solver_touchdown.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.arm_solver_touchdown.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.arm_solver_touchdown.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.arm_solver_touchdown.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.arm_solver_touchdown.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.arm_solver_touchdown.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.arm_solver_touchdown.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.arm_solver_touchdown.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.arm_solver_touchdown.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.arm_solver_touchdown.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.arm_solver_touchdown.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.arm_solver_touchdown.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.arm_solver_touchdown.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.arm_solver_touchdown.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.arm_solver_touchdown.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.arm_solver_touchdown.tau_ctrl(6);

}

inline void apply_motion_touchdown(
    motion_touchdown_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm_solver_touchdown.num_joints; ++i) {
        robot.arm_solver_touchdown.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm_solver_touchdown.tau_ctrl(i), i);
    }

}
