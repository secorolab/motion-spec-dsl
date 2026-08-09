/// Motion: arc-motion
/// Trace an arc toward the table and yield to an external X force
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_arc_motion_state {
    bool active = false;
    int active_steps = 0;
    rne_arm_solver_arc_motion_solver_state rne_arm_solver_arc_motion;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_follow_tangent;
    motion_spec::runtime::PIDControl ctrl_follow_position_lin_normal_a;
    motion_spec::runtime::PIDControl ctrl_follow_position_lin_normal_b;
    motion_spec::runtime::PIDControl ctrl_arc_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_arc_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_arc_orientation_ang_z;
    bool mon_arc_advancing_arc_advancing = false;
    bool mon_force_x_previous = false;
    bool mon_force_x_event_triggered = false;
    double mon_force_x_hold_s = 0.0;

    bool mon_force_y_previous = false;
    bool mon_force_y_event_triggered = false;
    double mon_force_y_hold_s = 0.0;

    bool mon_arc_complete_previous = false;
    bool mon_arc_complete_event_triggered = false;
    double mon_arc_complete_hold_s = 0.0;

};

inline void reset_motion_arc_motion(motion_arc_motion_state &state) {
    state = motion_arc_motion_state{};
}

inline void init_motion_arc_motion(motion_arc_motion_state &state, const robot_io &robot) {
    if (!state.rne_arm_solver_arc_motion.initialized) {
        state.rne_arm_solver_arc_motion.num_joints = robot.rne_arm_solver_arc_motion.chain->getNrOfJoints();
        state.rne_arm_solver_arc_motion.num_segments = robot.rne_arm_solver_arc_motion.chain->getNrOfSegments();
        state.rne_arm_solver_arc_motion.q = KDL::JntArray(state.rne_arm_solver_arc_motion.num_joints);
        state.rne_arm_solver_arc_motion.qd = KDL::JntArray(state.rne_arm_solver_arc_motion.num_joints);
        state.rne_arm_solver_arc_motion.qdd = KDL::JntArray(state.rne_arm_solver_arc_motion.num_joints);
        state.rne_arm_solver_arc_motion.tau_ff = KDL::JntArray(state.rne_arm_solver_arc_motion.num_joints);
        state.rne_arm_solver_arc_motion.tau_ctrl = KDL::JntArray(state.rne_arm_solver_arc_motion.num_joints);
        state.rne_arm_solver_arc_motion.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.rne_arm_solver_arc_motion.f_ext = KDL::Wrenches(state.rne_arm_solver_arc_motion.num_segments);
        state.rne_arm_solver_arc_motion.num_spatial_directions = 6;
        state.rne_arm_solver_arc_motion.spatial_directions = KDL::Jacobian(state.rne_arm_solver_arc_motion.num_spatial_directions);
        state.rne_arm_solver_arc_motion.cartesian_acceleration = KDL::JntArray(state.rne_arm_solver_arc_motion.num_spatial_directions);
        state.rne_arm_solver_arc_motion.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.rne_arm_solver_arc_motion.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.rne_arm_solver_arc_motion.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(*robot.rne_arm_solver_arc_motion.chain);
        state.rne_arm_solver_arc_motion.jac_dot_solver = std::make_unique<KDL::ChainJntToJacDotSolver>(*robot.rne_arm_solver_arc_motion.chain);

        state.rne_arm_solver_arc_motion.initialized = true;
    }
}

inline void update_motion_arc_motion(
    motion_arc_motion_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_arc_motion(state, robot);
    if (robot.ext_force != nullptr) {
        shared.ext_force = *robot.ext_force;
    }

    mj_kdl::update(robot.rne_arm_solver_arc_motion.robot);
    for (int i = 0; i < state.rne_arm_solver_arc_motion.num_joints; ++i) {
        state.rne_arm_solver_arc_motion.q(i) = robot.rne_arm_solver_arc_motion.robot->jnt_pos_msr[i];
        state.rne_arm_solver_arc_motion.qd(i) = robot.rne_arm_solver_arc_motion.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_rne_arm_solver_arc_motion(state.rne_arm_solver_arc_motion.q, state.rne_arm_solver_arc_motion.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.rne_arm_solver_arc_motion.chain);
        fk.JntToCart(
            state.rne_arm_solver_arc_motion.q,
            shared.pose_bracelet_base,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_arc_motion.chain, "bracelet_link", "base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.rne_arm_solver_arc_motion.chain);
        fk.JntToCart(
            state.rne_arm_solver_arc_motion.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_arc_motion.chain, "g_pinch", "base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.rne_arm_solver_arc_motion.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_rne_arm_solver_arc_motion,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_arc_motion.chain, "bracelet_link", "base_link"));
        shared.twist_bracelet_base = tmp.deriv();
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.rne_arm_solver_arc_motion.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_rne_arm_solver_arc_motion,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.rne_arm_solver_arc_motion.chain, "g_pinch", "base_link"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        const mj_kdl::ForceTorqueSensor *ft_ext_force = mj_kdl::find_ft_sensor(robot.rne_arm_solver_arc_motion.robot, "wrist_ft");
        if (!ft_ext_force) {
            throw std::runtime_error("FT sensor 'wrist_ft' is unavailable");
        }
        const auto frame_ext_force = [&](const char *name) {
            KDL::Frame frame;
            if (!mj_kdl::get_site_frame(robot.rne_arm_solver_arc_motion.robot->model, robot.rne_arm_solver_arc_motion.robot->data, name, &frame)
                && !mj_kdl::get_body_frame(robot.rne_arm_solver_arc_motion.robot->model, robot.rne_arm_solver_arc_motion.robot->data, name, &frame)) {
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
        shared.arc_motion_start_pose = shared.pose_ee_base;
        shared.initial_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }

    if (robot.fsm_events && consume_event(robot.fsm_events, admittance_arc_single_fsm::E_ARC_ENTERED)) {
            shared.arc_motion_start_pose = shared.pose_ee_base;
    }

    shared.end_pose = KDL::Frame(
        KDL::Rotation::Quaternion(0.0, 0.0, 0.3826834323650898, 0.9238795325112867) * shared.initial_pose.M,
        KDL::Vector(shared.end_x, shared.end_y, shared.end_z));
}

inline bool can_start_motion_arc_motion() {
    return true;
}

inline void monitor_when_motion_arc_motion(
    shared_data &shared
) {
    shared.end_pose = KDL::Frame(
        KDL::Rotation::Quaternion(0.0, 0.0, 0.3826834323650898, 0.9238795325112867) * shared.initial_pose.M,
        KDL::Vector(shared.end_x, shared.end_y, shared.end_z));
}

inline void monitor_until_motion_arc_motion(
    motion_arc_motion_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // end_y_hi_add
    shared.end_y_hi = shared.initial_pose.p[1] + shared.chord_y_hi;
    // end_y_lo_add
    shared.end_y_lo = shared.initial_pose.p[1] + shared.chord_y_lo;
    // eval_arc_motion_until_near_table
    shared.eval_arc_motion_until_near_table_err = motion_spec::runtime::evaluate_less_than_constraint(shared.pose_ee_base.p[2], shared.contact_z);
    // eval_arc_motion_until_at_target_y
    shared.eval_arc_motion_until_at_target_y_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_ee_base.p[1], shared.end_y_lo, shared.end_y_hi);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_until_near_table_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_until_at_target_y_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_arc_complete_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(1);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_ARC_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_ARC_CONTACT] << std::endl;
        }
    }

}

inline void monitor_motion_arc_motion(
    motion_arc_motion_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // end_y_hi_add
    shared.end_y_hi = shared.initial_pose.p[1] + shared.chord_y_hi;
    // end_y_lo_add
    shared.end_y_lo = shared.initial_pose.p[1] + shared.chord_y_lo;
    // eval_arc_motion_until_near_table
    shared.eval_arc_motion_until_near_table_err = motion_spec::runtime::evaluate_less_than_constraint(shared.pose_ee_base.p[2], shared.contact_z);
    // eval_arc_motion_until_at_target_y
    shared.eval_arc_motion_until_at_target_y_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_ee_base.p[1], shared.end_y_lo, shared.end_y_hi);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_until_near_table_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_until_at_target_y_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_arc_complete_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(1);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_ARC_CONTACT);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_ARC_CONTACT] << std::endl;
        }
    }

}

inline void control_motion_arc_motion(
    motion_arc_motion_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events) {
    // end_x_add
    shared.end_x = shared.initial_pose.p[0] + shared.chord_x;
    // end_y_add
    shared.end_y = shared.initial_pose.p[1] + shared.chord_y;
    // end_z_add
    shared.end_z = shared.initial_pose.p[2] + shared.chord_z;
    // projection_arc_path
    {
        KDL::Vector _n = shared.path_normal;
        _n.Normalize();
        // Circular arc from start to end that bows out of the chord by `amplitude` (the
        // sagitta). center, radius and swept angle are derived so the path begins at start,
        // ends exactly at end, and apexes amplitude away from the chord midpoint.
        // amplitude == |chord|/2 reproduces a clean semicircle (swept angle pi).
        const KDL::Vector _chord = shared.end_pose.p - shared.arc_motion_start_pose.p;
        const double _a = shared.arc_height;
        const double _chordlen = _chord.Norm();
        // A dynamically-resolved start/end can still land on a degenerate chord at runtime;
        // hold at start rather than dividing by zero.
        const bool _valid = _chordlen > 1e-9 && _a > 1e-9;
        KDL::Vector _center = shared.arc_motion_start_pose.p;
        KDL::Vector _v0 = KDL::Vector::Zero();
        double _theta = 0.0;
        if (_valid) {
            const KDL::Vector _chord_hat = _chord * (1.0 / _chordlen);
            const KDL::Vector _mid = 0.5 * (shared.arc_motion_start_pose.p + shared.end_pose.p);
            const double _m = 0.5 * _chordlen;
            // plane-normal is authored perpendicular to the chord, so normal x chord_hat is
            // the unit bulge direction directly, with no fallback axis needed.
            const KDL::Vector _perp = _n * _chord_hat;
            const double _R = (_a * _a + _m * _m) / (2.0 * _a);
            const double _d = (_a * _a - _m * _m) / (2.0 * _a);
            _center = _mid + _d * _perp;
            _theta = 2.0 * std::acos(std::max(-1.0, std::min(1.0, -_d / _R)));
            _v0 = shared.arc_motion_start_pose.p - _center;
        }
        const KDL::Frame _rot_start(shared.arc_motion_start_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const KDL::Frame _rot_goal(shared.end_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            const double _sc = _valid ? motion_spec::runtime::clamp01(_s) : 0.0;
            const KDL::Vector _pos = _valid
                ? _center + KDL::Rotation::Rot(_n, -_theta * _sc) * _v0
                : shared.arc_motion_start_pose.p;
            return KDL::Frame(
                KDL::addDelta(_rot_start, KDL::diff(_rot_start, _rot_goal), _sc).M, _pos);
        };
        shared.arc_path_s = motion_spec::runtime::path_project(
            _path_eval, shared.pose_ee_base.p, shared.arc_path_s);
    }
    // frame_arc_path
    {
        KDL::Vector _n = shared.path_normal;
        _n.Normalize();
        // Circular arc from start to end that bows out of the chord by `amplitude` (the
        // sagitta). center, radius and swept angle are derived so the path begins at start,
        // ends exactly at end, and apexes amplitude away from the chord midpoint.
        // amplitude == |chord|/2 reproduces a clean semicircle (swept angle pi).
        const KDL::Vector _chord = shared.end_pose.p - shared.arc_motion_start_pose.p;
        const double _a = shared.arc_height;
        const double _chordlen = _chord.Norm();
        // A dynamically-resolved start/end can still land on a degenerate chord at runtime;
        // hold at start rather than dividing by zero.
        const bool _valid = _chordlen > 1e-9 && _a > 1e-9;
        KDL::Vector _center = shared.arc_motion_start_pose.p;
        KDL::Vector _v0 = KDL::Vector::Zero();
        double _theta = 0.0;
        if (_valid) {
            const KDL::Vector _chord_hat = _chord * (1.0 / _chordlen);
            const KDL::Vector _mid = 0.5 * (shared.arc_motion_start_pose.p + shared.end_pose.p);
            const double _m = 0.5 * _chordlen;
            // plane-normal is authored perpendicular to the chord, so normal x chord_hat is
            // the unit bulge direction directly, with no fallback axis needed.
            const KDL::Vector _perp = _n * _chord_hat;
            const double _R = (_a * _a + _m * _m) / (2.0 * _a);
            const double _d = (_a * _a - _m * _m) / (2.0 * _a);
            _center = _mid + _d * _perp;
            _theta = 2.0 * std::acos(std::max(-1.0, std::min(1.0, -_d / _R)));
            _v0 = shared.arc_motion_start_pose.p - _center;
        }
        const KDL::Frame _rot_start(shared.arc_motion_start_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const KDL::Frame _rot_goal(shared.end_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            const double _sc = _valid ? motion_spec::runtime::clamp01(_s) : 0.0;
            const KDL::Vector _pos = _valid
                ? _center + KDL::Rotation::Rot(_n, -_theta * _sc) * _v0
                : shared.arc_motion_start_pose.p;
            return KDL::Frame(
                KDL::addDelta(_rot_start, KDL::diff(_rot_start, _rot_goal), _sc).M, _pos);
        };
        motion_spec::runtime::path_frame(
            _path_eval, shared.arc_path_s,
            shared.arc_path_tangent, shared.arc_path_normal_a, shared.arc_path_normal_b);
    }
    // along_arc_path
    shared.arc_path_along_speed = KDL::dot(shared.twist_ee_base.vel, shared.arc_path_tangent);
    // evaluator_arc_path
    {
        KDL::Vector _n = shared.path_normal;
        _n.Normalize();
        // Circular arc from start to end that bows out of the chord by `amplitude` (the
        // sagitta). center, radius and swept angle are derived so the path begins at start,
        // ends exactly at end, and apexes amplitude away from the chord midpoint.
        // amplitude == |chord|/2 reproduces a clean semicircle (swept angle pi).
        const KDL::Vector _chord = shared.end_pose.p - shared.arc_motion_start_pose.p;
        const double _a = shared.arc_height;
        const double _chordlen = _chord.Norm();
        // A dynamically-resolved start/end can still land on a degenerate chord at runtime;
        // hold at start rather than dividing by zero.
        const bool _valid = _chordlen > 1e-9 && _a > 1e-9;
        KDL::Vector _center = shared.arc_motion_start_pose.p;
        KDL::Vector _v0 = KDL::Vector::Zero();
        double _theta = 0.0;
        if (_valid) {
            const KDL::Vector _chord_hat = _chord * (1.0 / _chordlen);
            const KDL::Vector _mid = 0.5 * (shared.arc_motion_start_pose.p + shared.end_pose.p);
            const double _m = 0.5 * _chordlen;
            // plane-normal is authored perpendicular to the chord, so normal x chord_hat is
            // the unit bulge direction directly, with no fallback axis needed.
            const KDL::Vector _perp = _n * _chord_hat;
            const double _R = (_a * _a + _m * _m) / (2.0 * _a);
            const double _d = (_a * _a - _m * _m) / (2.0 * _a);
            _center = _mid + _d * _perp;
            _theta = 2.0 * std::acos(std::max(-1.0, std::min(1.0, -_d / _R)));
            _v0 = shared.arc_motion_start_pose.p - _center;
        }
        const KDL::Frame _rot_start(shared.arc_motion_start_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const KDL::Frame _rot_goal(shared.end_pose.M, KDL::Vector(0.0, 0.0, 0.0));
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            const double _sc = _valid ? motion_spec::runtime::clamp01(_s) : 0.0;
            const KDL::Vector _pos = _valid
                ? _center + KDL::Rotation::Rot(_n, -_theta * _sc) * _v0
                : shared.arc_motion_start_pose.p;
            return KDL::Frame(
                KDL::addDelta(_rot_start, KDL::diff(_rot_start, _rot_goal), _sc).M, _pos);
        };
        shared.reference = _path_eval(shared.arc_path_s);
    }
    // eval_pose_diff_ctrl_follow_position
    shared.pose_diff_ctrl_follow_position = KDL::diff(shared.pose_ee_base, shared.reference);
    shared.ctrl_follow_position_err_lin_normal_a = KDL::dot(shared.pose_diff_ctrl_follow_position.vel, shared.arc_path_normal_a);
    shared.ctrl_follow_position_err_lin_normal_b = KDL::dot(shared.pose_diff_ctrl_follow_position.vel, shared.arc_path_normal_b);
    // eval_pose_diff_ctrl_arc_orientation
    shared.pose_diff_ctrl_arc_orientation = KDL::diff(shared.pose_ee_base, shared.reference);
    shared.ctrl_arc_orientation_err_ang_x = shared.pose_diff_ctrl_arc_orientation.rot[0];
    shared.ctrl_arc_orientation_err_ang_y = shared.pose_diff_ctrl_arc_orientation.rot[1];
    shared.ctrl_arc_orientation_err_ang_z = shared.pose_diff_ctrl_arc_orientation.rot[2];
    // eval_arc_motion_while_follow_tangent
    shared.arc_path_along_speed_err_arc_motion = motion_spec::runtime::evaluate_equality_constraint(shared.arc_speed, shared.arc_path_along_speed);
    // eval_arc_motion_while_advance
    shared.eval_arc_motion_while_advance_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.arc_path_along_speed, shared.min_arc_speed);
    // eval_arc_motion_while_force_x
    shared.eval_arc_motion_while_force_x_err = motion_spec::runtime::evaluate_outside_constraint(shared.ext_force.force[0], shared.neg_force_threshold, shared.force_threshold);
    // eval_arc_motion_while_force_y
    shared.eval_arc_motion_while_force_y_err = motion_spec::runtime::evaluate_outside_constraint(shared.ext_force.force[1], shared.neg_force_threshold, shared.force_threshold);
    // ctrl_arc_orientation_ang_z
    {
        const double _control_signal = state.ctrl_arc_orientation_ang_z.control(shared.pose_diff_ctrl_arc_orientation.rot[2], -(shared.twist_ee_base.rot[2]), shared.dt_measured_s, {shared.ctrl_arc_orientation_ang_z_kp, shared.ctrl_arc_orientation_ang_z_ki, shared.ctrl_arc_orientation_ang_z_kd, shared.ctrl_arc_orientation_ang_z_decay_rate});
        shared.acc_ctrl_arc_orientation_ang_z = _control_signal;
        shared.ctrl_arc_orientation_ang_z_error_integral = state.ctrl_arc_orientation_ang_z.error_integral();
        shared.ctrl_arc_orientation_ang_z_previous_error = state.ctrl_arc_orientation_ang_z.previous_error();
        shared.ctrl_arc_orientation_ang_z_first_sample = state.ctrl_arc_orientation_ang_z.is_first_sample();
    }
    // ctrl_arc_orientation_ang_y
    {
        const double _control_signal = state.ctrl_arc_orientation_ang_y.control(shared.pose_diff_ctrl_arc_orientation.rot[1], -(shared.twist_ee_base.rot[1]), shared.dt_measured_s, {shared.ctrl_arc_orientation_ang_y_kp, shared.ctrl_arc_orientation_ang_y_ki, shared.ctrl_arc_orientation_ang_y_kd, shared.ctrl_arc_orientation_ang_y_decay_rate});
        shared.acc_ctrl_arc_orientation_ang_y = _control_signal;
        shared.ctrl_arc_orientation_ang_y_error_integral = state.ctrl_arc_orientation_ang_y.error_integral();
        shared.ctrl_arc_orientation_ang_y_previous_error = state.ctrl_arc_orientation_ang_y.previous_error();
        shared.ctrl_arc_orientation_ang_y_first_sample = state.ctrl_arc_orientation_ang_y.is_first_sample();
    }
    // ctrl_arc_orientation_ang_x
    {
        const double _control_signal = state.ctrl_arc_orientation_ang_x.control(shared.pose_diff_ctrl_arc_orientation.rot[0], -(shared.twist_ee_base.rot[0]), shared.dt_measured_s, {shared.ctrl_arc_orientation_ang_x_kp, shared.ctrl_arc_orientation_ang_x_ki, shared.ctrl_arc_orientation_ang_x_kd, shared.ctrl_arc_orientation_ang_x_decay_rate});
        shared.acc_ctrl_arc_orientation_ang_x = _control_signal;
        shared.ctrl_arc_orientation_ang_x_error_integral = state.ctrl_arc_orientation_ang_x.error_integral();
        shared.ctrl_arc_orientation_ang_x_previous_error = state.ctrl_arc_orientation_ang_x.previous_error();
        shared.ctrl_arc_orientation_ang_x_first_sample = state.ctrl_arc_orientation_ang_x.is_first_sample();
    }
    // ctrl_follow_position_lin_normal_b
    {
        const double _control_signal = state.ctrl_follow_position_lin_normal_b.control(KDL::dot(shared.pose_diff_ctrl_follow_position.vel, shared.arc_path_normal_b), -(KDL::dot(shared.twist_ee_base.vel, shared.arc_path_normal_b)), shared.dt_measured_s, {shared.ctrl_follow_position_lin_normal_b_kp, shared.ctrl_follow_position_lin_normal_b_ki, shared.ctrl_follow_position_lin_normal_b_kd, shared.ctrl_follow_position_lin_normal_b_decay_rate});
        shared.acc_ctrl_follow_position_lin_normal_b = _control_signal;
        shared.ctrl_follow_position_lin_normal_b_error_integral = state.ctrl_follow_position_lin_normal_b.error_integral();
        shared.ctrl_follow_position_lin_normal_b_previous_error = state.ctrl_follow_position_lin_normal_b.previous_error();
        shared.ctrl_follow_position_lin_normal_b_first_sample = state.ctrl_follow_position_lin_normal_b.is_first_sample();
    }
    // ctrl_follow_position_lin_normal_a
    {
        const double _control_signal = state.ctrl_follow_position_lin_normal_a.control(KDL::dot(shared.pose_diff_ctrl_follow_position.vel, shared.arc_path_normal_a), -(KDL::dot(shared.twist_ee_base.vel, shared.arc_path_normal_a)), shared.dt_measured_s, {shared.ctrl_follow_position_lin_normal_a_kp, shared.ctrl_follow_position_lin_normal_a_ki, shared.ctrl_follow_position_lin_normal_a_kd, shared.ctrl_follow_position_lin_normal_a_decay_rate});
        shared.acc_ctrl_follow_position_lin_normal_a = _control_signal;
        shared.ctrl_follow_position_lin_normal_a_error_integral = state.ctrl_follow_position_lin_normal_a.error_integral();
        shared.ctrl_follow_position_lin_normal_a_previous_error = state.ctrl_follow_position_lin_normal_a.previous_error();
        shared.ctrl_follow_position_lin_normal_a_first_sample = state.ctrl_follow_position_lin_normal_a.is_first_sample();
    }
    // ctrl_follow_tangent
    {
        const double _control_signal = state.ctrl_follow_tangent.control(shared.arc_path_along_speed_err_arc_motion, shared.dt_measured_s, {shared.ctrl_follow_tangent_kp, shared.ctrl_follow_tangent_ki, shared.ctrl_follow_tangent_kd, shared.ctrl_follow_tangent_decay_rate});
        shared.acc_arc_path_along_speed_arc_motion = _control_signal;
        shared.ctrl_follow_tangent_error_integral = state.ctrl_follow_tangent.error_integral();
        shared.ctrl_follow_tangent_previous_error = state.ctrl_follow_tangent.previous_error();
        shared.ctrl_follow_tangent_first_sample = state.ctrl_follow_tangent.is_first_sample();
    }

    motion_spec::runtime::set_flag(state.mon_arc_advancing_arc_advancing, motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_while_advance_err, 0.0));

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_while_force_x_err, 0.0);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_force_x_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(4);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORCE_DETECTED);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORCE_DETECTED] << std::endl;
        }
    }

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.eval_arc_motion_while_force_y_err, 0.0);
        const bool detected = motion_spec::runtime::sustained_edge(
            state.mon_force_y_hold_s,
            active,
            shared.dt_measured_s,
            0.3);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(4);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, admittance_arc_single_fsm::E_FORCE_DETECTED);
            std::cerr << "[fsm] event   " << admittance_arc_single_fsm::EVENT_URIS[admittance_arc_single_fsm::E_FORCE_DETECTED] << std::endl;
        }
    }

    KDL::SetToZero(state.rne_arm_solver_arc_motion.spatial_directions);

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = shared.arc_path_tangent.x();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = shared.arc_path_tangent.y();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = shared.arc_path_tangent.z();

    state.rne_arm_solver_arc_motion.cartesian_acceleration(0) = shared.acc_arc_path_along_speed_arc_motion;

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = shared.arc_path_normal_a.x();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = shared.arc_path_normal_a.y();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = shared.arc_path_normal_a.z();

    state.rne_arm_solver_arc_motion.cartesian_acceleration(1) = shared.acc_ctrl_follow_position_lin_normal_a;

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = shared.arc_path_normal_b.x();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = shared.arc_path_normal_b.y();
    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = shared.arc_path_normal_b.z();

    state.rne_arm_solver_arc_motion.cartesian_acceleration(2) = shared.acc_ctrl_follow_position_lin_normal_b;

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;

    state.rne_arm_solver_arc_motion.cartesian_acceleration(3) = shared.acc_ctrl_arc_orientation_ang_x;

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;

    state.rne_arm_solver_arc_motion.cartesian_acceleration(4) = shared.acc_ctrl_arc_orientation_ang_y;

    state.rne_arm_solver_arc_motion.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;

    state.rne_arm_solver_arc_motion.cartesian_acceleration(5) = shared.acc_ctrl_arc_orientation_ang_z;

    KDL::SetToZero(state.rne_arm_solver_arc_motion.tau_ff);
    for (int i = 0; i < state.rne_arm_solver_arc_motion.num_segments; ++i) {
        KDL::SetToZero(state.rne_arm_solver_arc_motion.f_ext[i]);
    }
    KDL::Wrenches f_ext_zero_rne_arm_solver_arc_motion(state.rne_arm_solver_arc_motion.num_segments);
    for (int i = 0; i < state.rne_arm_solver_arc_motion.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_rne_arm_solver_arc_motion[i]);
    }
    KDL::Jacobian jac_rne_arm_solver_arc_motion(state.rne_arm_solver_arc_motion.num_joints);
    state.rne_arm_solver_arc_motion.jac_solver->JntToJac(state.rne_arm_solver_arc_motion.q, jac_rne_arm_solver_arc_motion);
    // Task-space velocity-product term Jdot*qdot (base frame, ee ref point):
    // xdd = J*qdd + Jdot*qdot, so subtract it from the desired task accel before
    // resolving the joint acceleration consumed by RNEA. This Cartesian-to-joint
    // mapping is an upstream kinematic adapter, not part of RNEA itself.
    KDL::Twist jdot_qd_rne_arm_solver_arc_motion;
    state.rne_arm_solver_arc_motion.jac_dot_solver->JntToJacDot(
        KDL::JntArrayVel(state.rne_arm_solver_arc_motion.q, state.rne_arm_solver_arc_motion.qd), jdot_qd_rne_arm_solver_arc_motion);
    Eigen::Matrix<double, 6, 1> jdot_qd_vec_rne_arm_solver_arc_motion;
    jdot_qd_vec_rne_arm_solver_arc_motion << jdot_qd_rne_arm_solver_arc_motion.vel.x(), jdot_qd_rne_arm_solver_arc_motion.vel.y(), jdot_qd_rne_arm_solver_arc_motion.vel.z(),
                               jdot_qd_rne_arm_solver_arc_motion.rot.x(), jdot_qd_rne_arm_solver_arc_motion.rot.y(), jdot_qd_rne_arm_solver_arc_motion.rot.z();
    const Eigen::VectorXd task_bias_rne_arm_solver_arc_motion = state.rne_arm_solver_arc_motion.spatial_directions.data.transpose() * jdot_qd_vec_rne_arm_solver_arc_motion;
    KDL::JntArray qdd_des_rne_arm_solver_arc_motion(state.rne_arm_solver_arc_motion.num_joints);
    motion_spec::runtime::resolve_cartesian_acceleration(
        jac_rne_arm_solver_arc_motion,
        state.rne_arm_solver_arc_motion.spatial_directions,
        state.rne_arm_solver_arc_motion.cartesian_acceleration,
        &task_bias_rne_arm_solver_arc_motion,
        qdd_des_rne_arm_solver_arc_motion);
    state.rne_arm_solver_arc_motion.rnea->CartToJnt(
        state.rne_arm_solver_arc_motion.q,
        state.rne_arm_solver_arc_motion.qd,
        qdd_des_rne_arm_solver_arc_motion,
        state.rne_arm_solver_arc_motion.f_ext,
        state.rne_arm_solver_arc_motion.tau_ctrl);
    // Publish the resolved acceleration RNE was given; ACHD writes state.qdd itself, this is the
    // RNE path's equivalent. Copied after CartToJnt, never passed in as the input in its place.
    state.rne_arm_solver_arc_motion.qdd = qdd_des_rne_arm_solver_arc_motion;
    for (int i = 0; i < state.rne_arm_solver_arc_motion.num_joints; ++i) {
        state.rne_arm_solver_arc_motion.tau_ctrl(i) += state.rne_arm_solver_arc_motion.tau_ff(i);
    }
    shared.arm_solver_home_q_joint_1 = state.rne_arm_solver_arc_motion.q(0);
    shared.arm_solver_home_q_joint_2 = state.rne_arm_solver_arc_motion.q(1);
    shared.arm_solver_home_q_joint_3 = state.rne_arm_solver_arc_motion.q(2);
    shared.arm_solver_home_q_joint_4 = state.rne_arm_solver_arc_motion.q(3);
    shared.arm_solver_home_q_joint_5 = state.rne_arm_solver_arc_motion.q(4);
    shared.arm_solver_home_q_joint_6 = state.rne_arm_solver_arc_motion.q(5);
    shared.arm_solver_home_q_joint_7 = state.rne_arm_solver_arc_motion.q(6);
    shared.arm_solver_home_qd_joint_1 = state.rne_arm_solver_arc_motion.qd(0);
    shared.arm_solver_home_qd_joint_2 = state.rne_arm_solver_arc_motion.qd(1);
    shared.arm_solver_home_qd_joint_3 = state.rne_arm_solver_arc_motion.qd(2);
    shared.arm_solver_home_qd_joint_4 = state.rne_arm_solver_arc_motion.qd(3);
    shared.arm_solver_home_qd_joint_5 = state.rne_arm_solver_arc_motion.qd(4);
    shared.arm_solver_home_qd_joint_6 = state.rne_arm_solver_arc_motion.qd(5);
    shared.arm_solver_home_qd_joint_7 = state.rne_arm_solver_arc_motion.qd(6);
    shared.arm_solver_home_qdd_joint_1 = state.rne_arm_solver_arc_motion.qdd(0);
    shared.arm_solver_home_qdd_joint_2 = state.rne_arm_solver_arc_motion.qdd(1);
    shared.arm_solver_home_qdd_joint_3 = state.rne_arm_solver_arc_motion.qdd(2);
    shared.arm_solver_home_qdd_joint_4 = state.rne_arm_solver_arc_motion.qdd(3);
    shared.arm_solver_home_qdd_joint_5 = state.rne_arm_solver_arc_motion.qdd(4);
    shared.arm_solver_home_qdd_joint_6 = state.rne_arm_solver_arc_motion.qdd(5);
    shared.arm_solver_home_qdd_joint_7 = state.rne_arm_solver_arc_motion.qdd(6);
    shared.arm_solver_home_tau_ctrl_joint_1 = state.rne_arm_solver_arc_motion.tau_ctrl(0);
    shared.arm_solver_home_tau_ctrl_joint_2 = state.rne_arm_solver_arc_motion.tau_ctrl(1);
    shared.arm_solver_home_tau_ctrl_joint_3 = state.rne_arm_solver_arc_motion.tau_ctrl(2);
    shared.arm_solver_home_tau_ctrl_joint_4 = state.rne_arm_solver_arc_motion.tau_ctrl(3);
    shared.arm_solver_home_tau_ctrl_joint_5 = state.rne_arm_solver_arc_motion.tau_ctrl(4);
    shared.arm_solver_home_tau_ctrl_joint_6 = state.rne_arm_solver_arc_motion.tau_ctrl(5);
    shared.arm_solver_home_tau_ctrl_joint_7 = state.rne_arm_solver_arc_motion.tau_ctrl(6);

}

inline void apply_motion_arc_motion(
    motion_arc_motion_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.rne_arm_solver_arc_motion.num_joints; ++i) {
        robot.rne_arm_solver_arc_motion.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.rne_arm_solver_arc_motion.tau_ctrl(i), i);
    }

}
