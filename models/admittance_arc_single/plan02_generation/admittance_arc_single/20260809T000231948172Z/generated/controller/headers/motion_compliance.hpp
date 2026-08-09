/// Motion: compliance
/// Yield along the measured external force
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_compliance_state {
    bool active = false;
    int active_steps = 0;
    rne_arm_solver_compliance_solver_state rne_arm_solver_compliance;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_comply_x;
    motion_spec::runtime::PIDControl ctrl_comply_y;
    motion_spec::runtime::PIDControl ctrl_comply_z;
    motion_spec::runtime::PIDControl ctrl_comply_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_comply_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_comply_ori_ang_z;
    bool mon_released_previous = false;
    bool mon_released_event_triggered = false;
    double mon_released_hold_s = 0.0;

    bool mon_table_contact_previous = false;
    bool mon_table_contact_event_triggered = false;
    double mon_table_contact_hold_s = 0.0;

};

inline void reset_motion_compliance(motion_compliance_state &state) {
    state = motion_compliance_state{};
}

inline void init_motion_compliance(motion_compliance_state &state, const robot_io &robot) {
    if (!state.rne_arm_solver_compliance.initialized) {
        state.rne_arm_solver_compliance.num_joints = robot.rne_arm_solver_compliance.chain->getNrOfJoints();
        state.rne_arm_solver_compliance.num_segments = robot.rne_arm_solver_compliance.chain->getNrOfSegments();
        state.rne_arm_solver_compliance.q = KDL::JntArray(state.rne_arm_solver_compliance.num_joints);
        state.rne_arm_solver_compliance.qd = KDL::JntArray(state.rne_arm_solver_compliance.num_joints);
        state.rne_arm_solver_compliance.qdd = KDL::JntArray(state.rne_arm_solver_compliance.num_joints);
        state.rne_arm_solver_compliance.tau_ff = KDL::JntArray(state.rne_arm_solver_compliance.num_joints);
        state.rne_arm_solver_compliance.tau_ctrl = KDL::JntArray(state.rne_arm_solver_compliance.num_joints);
        state.rne_arm_solver_compliance.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.rne_arm_solver_compliance.f_ext = KDL::Wrenches(state.rne_arm_solver_compliance.num_segments);
        state.rne_arm_solver_compliance.num_spatial_directions = 6;
        state.rne_arm_solver_compliance.spatial_directions = KDL::Jacobian(state.rne_arm_solver_compliance.num_spatial_directions);
        state.rne_arm_solver_compliance.cartesian_acceleration = KDL::JntArray(state.rne_arm_solver_compliance.num_spatial_directions);
        state.rne_arm_solver_compliance.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.rne_arm_solver_compliance.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.rne_arm_solver_compliance.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(*robot.rne_arm_solver_compliance.chain);
        state.rne_arm_solver_compliance.jac_dot_solver = std::make_unique<KDL::ChainJntToJacDotSolver>(*robot.rne_arm_solver_compliance.chain);

        state.rne_arm_solver_compliance.initialized = true;
    }
}

inline void update_motion_compliance(
    motion_compliance_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_compliance(state, robot);
    if (robot.ext_force != nullptr) {
        shared.ext_force = *robot.ext_force;
    }

    mj_kdl::update(robot.rne_arm_solver_compliance.robot);
    for (int i = 0; i < state.rne_arm_solver_compliance.num_joints; ++i) {
        state.rne_arm_solver_compliance.q(i) = robot.rne_arm_solver_compliance.robot->jnt_pos_msr[i];
        state.rne_arm_solver_compliance.qd(i) = robot.rne_arm_solver_compliance.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_rne_arm_solver_compliance(state.rne_arm_solver_compliance.q, state.rne_arm_solver_compliance.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.rne_arm_solver_compliance.chain);
        fk.JntToCart(
            state.rne_arm_solver_compliance.q,
            shared.pose_bracelet_base,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_compliance.chain, "bracelet_link", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.rne_arm_solver_compliance.chain);
        fk.JntToCart(
            state.rne_arm_solver_compliance.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_compliance.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.rne_arm_solver_compliance.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_rne_arm_solver_compliance,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_compliance.chain, "bracelet_link", "base_link"));
        shared.twist_bracelet_base = tmp.deriv();
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.rne_arm_solver_compliance.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_rne_arm_solver_compliance,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_compliance.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        const mj_kdl::ForceTorqueSensor *ft_ext_force = mj_kdl::find_ft_sensor(robot.rne_arm_solver_compliance.robot, "wrist_ft");
        if (!ft_ext_force) {
            throw std::runtime_error("FT sensor 'wrist_ft' is unavailable");
        }
        const auto frame_ext_force = [&](const char *name) {
            KDL::Frame frame;
            if (!mj_kdl::get_site_frame(robot.rne_arm_solver_compliance.robot->model, robot.rne_arm_solver_compliance.robot->data, name, &frame)
                && !mj_kdl::get_body_frame(robot.rne_arm_solver_compliance.robot->model, robot.rne_arm_solver_compliance.robot->data, name, &frame)) {
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
        shared.hold_orientation = shared.pose_ee_base;
        state.snapshot_taken = true;
    }

    if (robot.fsm_events && consume_event(robot.fsm_events, admittance_arc_single_fsm::E_ADMITTANCE_ENTERED)) {
            shared.hold_orientation = shared.pose_ee_base;
    }

}

inline bool can_start_motion_compliance() {
    return true;
}

inline void monitor_when_motion_compliance() {
}

inline void monitor_until_motion_compliance(
    motion_compliance_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_released_x
    shared.eval_released_x_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[0], shared.neg_release_threshold, shared.release_threshold);
    // eval_released_y
    shared.eval_released_y_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[1], shared.neg_release_threshold, shared.release_threshold);
    // eval_released_z
    shared.eval_released_z_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[2], shared.neg_release_threshold, shared.release_threshold);
    // eval_near_table
    shared.eval_near_table_err = motion_spec::runtime::evaluate_less_than_constraint(shared.pose_bracelet_base.p[2], shared.bracelet_contact_z);
    // eval_contact_table
    shared.eval_contact_table_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_bracelet_base.vel[2]);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_released_x_err, 0.0) && motion_spec::runtime::constraint_satisfied(shared.eval_released_y_err, 0.0) && motion_spec::runtime::constraint_satisfied(shared.eval_released_z_err, 0.0));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_released_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORCE_GONE);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORCE_GONE] << std::endl;
        }
    }

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_near_table_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_contact_table_err, shared.satisfied_band_vel));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_table_contact_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(9);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_TABLE_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_TABLE_CONTACT] << std::endl;
        }
    }

}

inline void monitor_motion_compliance(
    motion_compliance_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_released_x
    shared.eval_released_x_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[0], shared.neg_release_threshold, shared.release_threshold);
    // eval_released_y
    shared.eval_released_y_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[1], shared.neg_release_threshold, shared.release_threshold);
    // eval_released_z
    shared.eval_released_z_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.ext_force.force[2], shared.neg_release_threshold, shared.release_threshold);
    // eval_near_table
    shared.eval_near_table_err = motion_spec::runtime::evaluate_less_than_constraint(shared.pose_bracelet_base.p[2], shared.bracelet_contact_z);
    // eval_contact_table
    shared.eval_contact_table_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_bracelet_base.vel[2]);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_released_x_err, 0.0) && motion_spec::runtime::constraint_satisfied(shared.eval_released_y_err, 0.0) && motion_spec::runtime::constraint_satisfied(shared.eval_released_z_err, 0.0));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_released_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORCE_GONE);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORCE_GONE] << std::endl;
        }
    }

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_near_table_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_contact_table_err, shared.satisfied_band_vel));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_table_contact_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(9);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_TABLE_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_TABLE_CONTACT] << std::endl;
        }
    }

}

inline void control_motion_compliance(
    motion_compliance_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // admit_comply_x_ctrl_comply_x
    shared.comply_x_ctrl_comply_x_admit_ref = state.ctrl_comply_x.adm_filter.step(shared.ext_force.force[0], shared.admit_comply_x_ctrl_comply_x_mass, shared.admit_comply_x_ctrl_comply_x_damping, shared.admit_comply_x_ctrl_comply_x_stiffness, shared.admit_comply_x_ctrl_comply_x_maximum_velocity, shared.dt_measured_s);
    // admit_comply_y_ctrl_comply_y
    shared.comply_y_ctrl_comply_y_admit_ref = state.ctrl_comply_y.adm_filter.step(shared.ext_force.force[1], shared.admit_comply_y_ctrl_comply_y_mass, shared.admit_comply_y_ctrl_comply_y_damping, shared.admit_comply_y_ctrl_comply_y_stiffness, shared.admit_comply_y_ctrl_comply_y_maximum_velocity, shared.dt_measured_s);
    // admit_comply_z_ctrl_comply_z
    shared.comply_z_ctrl_comply_z_admit_ref = state.ctrl_comply_z.adm_filter.step(shared.ext_force.force[2], shared.admit_comply_z_ctrl_comply_z_mass, shared.admit_comply_z_ctrl_comply_z_damping, shared.admit_comply_z_ctrl_comply_z_stiffness, shared.admit_comply_z_ctrl_comply_z_maximum_velocity, shared.dt_measured_s);

    {
        KDL::Twist _pose_axis_target_pose_axis_error_twist_ee_base = KDL::Twist::Zero();
        _pose_axis_target_pose_axis_error_twist_ee_base.vel[0] = shared.comply_x_ctrl_comply_x_admit_ref;
        _pose_axis_target_pose_axis_error_twist_ee_base.vel[1] = shared.comply_y_ctrl_comply_y_admit_ref;
        _pose_axis_target_pose_axis_error_twist_ee_base.vel[2] = shared.comply_z_ctrl_comply_z_admit_ref;
        const KDL::Twist _pose_axis_error_pose_axis_error_twist_ee_base = _pose_axis_target_pose_axis_error_twist_ee_base - shared.twist_ee_base;
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_twist_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_twist_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_twist_ee_base.vel.z();

        shared.twist_ee_base_linear_x_err_compliance = _pose_axis_error_linear_X;
        shared.twist_ee_base_linear_y_err_compliance = _pose_axis_error_linear_Y;
        shared.twist_ee_base_linear_z_err_compliance = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_comply_ori
    shared.pose_diff_ctrl_comply_ori = KDL::diff(shared.pose_ee_base, shared.hold_orientation);
    shared.ctrl_comply_ori_err_ang_x = shared.pose_diff_ctrl_comply_ori.rot[0];
    shared.ctrl_comply_ori_err_ang_y = shared.pose_diff_ctrl_comply_ori.rot[1];
    shared.ctrl_comply_ori_err_ang_z = shared.pose_diff_ctrl_comply_ori.rot[2];
    // ctrl_comply_ori_ang_z
    {
        const double _control_signal = state.ctrl_comply_ori_ang_z.control(shared.pose_diff_ctrl_comply_ori.rot[2], -(shared.twist_ee_base.rot[2]), shared.dt_measured_s, {shared.ctrl_comply_ori_ang_z_kp, shared.ctrl_comply_ori_ang_z_ki, shared.ctrl_comply_ori_ang_z_kd, shared.ctrl_comply_ori_ang_z_decay_rate});
        shared.acc_ctrl_comply_ori_ang_z = _control_signal;
        shared.ctrl_comply_ori_ang_z_error_integral = state.ctrl_comply_ori_ang_z.error_integral();
        shared.ctrl_comply_ori_ang_z_previous_error = state.ctrl_comply_ori_ang_z.previous_error();
        shared.ctrl_comply_ori_ang_z_first_sample = state.ctrl_comply_ori_ang_z.is_first_sample();
    }
    // ctrl_comply_ori_ang_y
    {
        const double _control_signal = state.ctrl_comply_ori_ang_y.control(shared.pose_diff_ctrl_comply_ori.rot[1], -(shared.twist_ee_base.rot[1]), shared.dt_measured_s, {shared.ctrl_comply_ori_ang_y_kp, shared.ctrl_comply_ori_ang_y_ki, shared.ctrl_comply_ori_ang_y_kd, shared.ctrl_comply_ori_ang_y_decay_rate});
        shared.acc_ctrl_comply_ori_ang_y = _control_signal;
        shared.ctrl_comply_ori_ang_y_error_integral = state.ctrl_comply_ori_ang_y.error_integral();
        shared.ctrl_comply_ori_ang_y_previous_error = state.ctrl_comply_ori_ang_y.previous_error();
        shared.ctrl_comply_ori_ang_y_first_sample = state.ctrl_comply_ori_ang_y.is_first_sample();
    }
    // ctrl_comply_ori_ang_x
    {
        const double _control_signal = state.ctrl_comply_ori_ang_x.control(shared.pose_diff_ctrl_comply_ori.rot[0], -(shared.twist_ee_base.rot[0]), shared.dt_measured_s, {shared.ctrl_comply_ori_ang_x_kp, shared.ctrl_comply_ori_ang_x_ki, shared.ctrl_comply_ori_ang_x_kd, shared.ctrl_comply_ori_ang_x_decay_rate});
        shared.acc_ctrl_comply_ori_ang_x = _control_signal;
        shared.ctrl_comply_ori_ang_x_error_integral = state.ctrl_comply_ori_ang_x.error_integral();
        shared.ctrl_comply_ori_ang_x_previous_error = state.ctrl_comply_ori_ang_x.previous_error();
        shared.ctrl_comply_ori_ang_x_first_sample = state.ctrl_comply_ori_ang_x.is_first_sample();
    }
    // ctrl_comply_z
    {
        const double _control_signal = state.ctrl_comply_z.control(shared.twist_ee_base_linear_z_err_compliance, shared.dt_measured_s, {shared.ctrl_comply_z_kp, shared.ctrl_comply_z_ki, shared.ctrl_comply_z_kd, shared.ctrl_comply_z_decay_rate, true, shared.neg_comply_int_max, shared.comply_int_max});
        shared.acc_twist_ee_base_linear_z_compliance = _control_signal;
        shared.ctrl_comply_z_error_integral = state.ctrl_comply_z.error_integral();
        shared.ctrl_comply_z_previous_error = state.ctrl_comply_z.previous_error();
        shared.ctrl_comply_z_first_sample = state.ctrl_comply_z.is_first_sample();
    }
    // ctrl_comply_y
    {
        const double _control_signal = state.ctrl_comply_y.control(shared.twist_ee_base_linear_y_err_compliance, shared.dt_measured_s, {shared.ctrl_comply_y_kp, shared.ctrl_comply_y_ki, shared.ctrl_comply_y_kd, shared.ctrl_comply_y_decay_rate, true, shared.neg_comply_int_max, shared.comply_int_max});
        shared.acc_twist_ee_base_linear_y_compliance = _control_signal;
        shared.ctrl_comply_y_error_integral = state.ctrl_comply_y.error_integral();
        shared.ctrl_comply_y_previous_error = state.ctrl_comply_y.previous_error();
        shared.ctrl_comply_y_first_sample = state.ctrl_comply_y.is_first_sample();
    }
    // ctrl_comply_x
    {
        const double _control_signal = state.ctrl_comply_x.control(shared.twist_ee_base_linear_x_err_compliance, shared.dt_measured_s, {shared.ctrl_comply_x_kp, shared.ctrl_comply_x_ki, shared.ctrl_comply_x_kd, shared.ctrl_comply_x_decay_rate, true, shared.neg_comply_int_max, shared.comply_int_max});
        shared.acc_twist_ee_base_linear_x_compliance = _control_signal;
        shared.ctrl_comply_x_error_integral = state.ctrl_comply_x.error_integral();
        shared.ctrl_comply_x_previous_error = state.ctrl_comply_x.previous_error();
        shared.ctrl_comply_x_first_sample = state.ctrl_comply_x.is_first_sample();
    }

    KDL::SetToZero(state.rne_arm_solver_compliance.spatial_directions);

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(0) = shared.acc_twist_ee_base_linear_x_compliance;

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(1) = shared.acc_twist_ee_base_linear_y_compliance;

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(2) = shared.acc_twist_ee_base_linear_z_compliance;

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(3) = shared.acc_ctrl_comply_ori_ang_x;

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(4) = shared.acc_ctrl_comply_ori_ang_y;

    state.rne_arm_solver_compliance.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.rne_arm_solver_compliance.cartesian_acceleration(5) = shared.acc_ctrl_comply_ori_ang_z;

    KDL::SetToZero(state.rne_arm_solver_compliance.tau_ff);
    for (int i = 0; i < state.rne_arm_solver_compliance.num_segments; ++i) {
        KDL::SetToZero(state.rne_arm_solver_compliance.f_ext[i]);
    }
    KDL::Wrenches f_ext_zero_rne_arm_solver_compliance(state.rne_arm_solver_compliance.num_segments);
    for (int i = 0; i < state.rne_arm_solver_compliance.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_rne_arm_solver_compliance[i]);
    }
    KDL::Jacobian jac_rne_arm_solver_compliance(state.rne_arm_solver_compliance.num_joints);
    state.rne_arm_solver_compliance.jac_solver->JntToJac(state.rne_arm_solver_compliance.q, jac_rne_arm_solver_compliance);
    // Task-space velocity-product term Jdot*qdot (base frame, ee ref point):
    // xdd = J*qdd + Jdot*qdot, so subtract it from the desired task accel before
    // resolving the joint acceleration consumed by RNEA. This Cartesian-to-joint
    // mapping is an upstream kinematic adapter, not part of RNEA itself.
    KDL::Twist jdot_qd_rne_arm_solver_compliance;
    state.rne_arm_solver_compliance.jac_dot_solver->JntToJacDot(
        KDL::JntArrayVel(state.rne_arm_solver_compliance.q, state.rne_arm_solver_compliance.qd), jdot_qd_rne_arm_solver_compliance);
    Eigen::Matrix<double, 6, 1> jdot_qd_vec_rne_arm_solver_compliance;
    jdot_qd_vec_rne_arm_solver_compliance << jdot_qd_rne_arm_solver_compliance.vel.x(), jdot_qd_rne_arm_solver_compliance.vel.y(), jdot_qd_rne_arm_solver_compliance.vel.z(),
                               jdot_qd_rne_arm_solver_compliance.rot.x(), jdot_qd_rne_arm_solver_compliance.rot.y(), jdot_qd_rne_arm_solver_compliance.rot.z();
    const Eigen::VectorXd task_bias_rne_arm_solver_compliance = state.rne_arm_solver_compliance.spatial_directions.data.transpose() * jdot_qd_vec_rne_arm_solver_compliance;
    KDL::JntArray qdd_des_rne_arm_solver_compliance(state.rne_arm_solver_compliance.num_joints);
    motion_spec::runtime::resolve_cartesian_acceleration(
        jac_rne_arm_solver_compliance,
        state.rne_arm_solver_compliance.spatial_directions,
        state.rne_arm_solver_compliance.cartesian_acceleration,
        &task_bias_rne_arm_solver_compliance,
        qdd_des_rne_arm_solver_compliance);
    state.rne_arm_solver_compliance.rnea->CartToJnt(
        state.rne_arm_solver_compliance.q,
        state.rne_arm_solver_compliance.qd,
        qdd_des_rne_arm_solver_compliance,
        state.rne_arm_solver_compliance.f_ext,
        state.rne_arm_solver_compliance.tau_ctrl);
    // Publish the resolved acceleration RNE was given; ACHD writes state.qdd itself, this is the
    // RNE path's equivalent. Copied after CartToJnt, never passed in as the input in its place.
    state.rne_arm_solver_compliance.qdd = qdd_des_rne_arm_solver_compliance;
    for (int i = 0; i < state.rne_arm_solver_compliance.num_joints; ++i) {
        state.rne_arm_solver_compliance.tau_ctrl(i) += state.rne_arm_solver_compliance.tau_ff(i);
    }
    shared.arm_solver_home_q_joint_1 = state.rne_arm_solver_compliance.q(0);
    shared.arm_solver_home_q_joint_2 = state.rne_arm_solver_compliance.q(1);
    shared.arm_solver_home_q_joint_3 = state.rne_arm_solver_compliance.q(2);
    shared.arm_solver_home_q_joint_4 = state.rne_arm_solver_compliance.q(3);
    shared.arm_solver_home_q_joint_5 = state.rne_arm_solver_compliance.q(4);
    shared.arm_solver_home_q_joint_6 = state.rne_arm_solver_compliance.q(5);
    shared.arm_solver_home_q_joint_7 = state.rne_arm_solver_compliance.q(6);
    shared.arm_solver_home_qd_joint_1 = state.rne_arm_solver_compliance.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.rne_arm_solver_compliance.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.rne_arm_solver_compliance.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.rne_arm_solver_compliance.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.rne_arm_solver_compliance.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.rne_arm_solver_compliance.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.rne_arm_solver_compliance.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.rne_arm_solver_compliance.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.rne_arm_solver_compliance.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.rne_arm_solver_compliance.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.rne_arm_solver_compliance.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.rne_arm_solver_compliance.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.rne_arm_solver_compliance.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.rne_arm_solver_compliance.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.rne_arm_solver_compliance.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.rne_arm_solver_compliance.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.rne_arm_solver_compliance.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.rne_arm_solver_compliance.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.rne_arm_solver_compliance.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.rne_arm_solver_compliance.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.rne_arm_solver_compliance.tau_ctrl(6);

}

inline void apply_motion_compliance(
    motion_compliance_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.rne_arm_solver_compliance.num_joints; ++i) {
        robot.rne_arm_solver_compliance.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.rne_arm_solver_compliance.tau_ctrl(i), i);
    }

}
