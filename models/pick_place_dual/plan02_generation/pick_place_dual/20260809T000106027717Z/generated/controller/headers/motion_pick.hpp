/// Motion: pick
/// Lower both TCPs straight down to the grasp height at their cubes
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_pick_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_pick_solver_state arm1_solver_pick;
    arm2_solver_pick_solver_state arm2_solver_pick;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pk1_hold_x;
    motion_spec::runtime::PIDControl ctrl_pk1_hold_y;
    motion_spec::runtime::PIDControl ctrl_pk1_lower_z;
    motion_spec::runtime::PIDControl ctrl_pk1_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pk1_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pk1_follow_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pk1_support_z;
    motion_spec::runtime::PIDControl ctrl_pk2_hold_x;
    motion_spec::runtime::PIDControl ctrl_pk2_hold_y;
    motion_spec::runtime::PIDControl ctrl_pk2_lower_z;
    motion_spec::runtime::PIDControl ctrl_pk2_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pk2_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pk2_follow_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pk2_support_z;
    bool mon_pick_ready_previous = false;
    bool mon_pick_ready_event_triggered = false;

};

inline void reset_motion_pick(motion_pick_state &state) {
    state = motion_pick_state{};
}

inline void init_motion_pick(motion_pick_state &state, const robot_io &robot) {
    if (!state.arm1_solver_pick.initialized) {
        state.arm1_solver_pick.num_joints = robot.arm1_solver_pick.chain->getNrOfJoints();
        state.arm1_solver_pick.num_segments = robot.arm1_solver_pick.chain->getNrOfSegments();
        state.arm1_solver_pick.q = KDL::JntArray(state.arm1_solver_pick.num_joints);
        state.arm1_solver_pick.qd = KDL::JntArray(state.arm1_solver_pick.num_joints);
        state.arm1_solver_pick.qdd = KDL::JntArray(state.arm1_solver_pick.num_joints);
        state.arm1_solver_pick.tau_ff = KDL::JntArray(state.arm1_solver_pick.num_joints);
        state.arm1_solver_pick.tau_ctrl = KDL::JntArray(state.arm1_solver_pick.num_joints);
        state.arm1_solver_pick.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_pick.num_spatial_directions = 6;
        state.arm1_solver_pick.spatial_directions = KDL::Jacobian(state.arm1_solver_pick.num_spatial_directions);
        state.arm1_solver_pick.acceleration_energy = KDL::JntArray(state.arm1_solver_pick.num_spatial_directions);
        state.arm1_solver_pick.f_ext = KDL::Wrenches(state.arm1_solver_pick.num_segments);
        state.arm1_solver_pick.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm1_solver_pick.chain, state.arm1_solver_pick.root_acc, state.arm1_solver_pick.num_spatial_directions);
        state.arm1_solver_pick.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_pick.chain, state.arm1_solver_pick.root_acc, state.arm1_solver_pick.num_spatial_directions);
        state.arm1_solver_pick.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_pick.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_pick.initialized = true;
    }
    if (!state.arm2_solver_pick.initialized) {
        state.arm2_solver_pick.num_joints = robot.arm2_solver_pick.chain->getNrOfJoints();
        state.arm2_solver_pick.num_segments = robot.arm2_solver_pick.chain->getNrOfSegments();
        state.arm2_solver_pick.q = KDL::JntArray(state.arm2_solver_pick.num_joints);
        state.arm2_solver_pick.qd = KDL::JntArray(state.arm2_solver_pick.num_joints);
        state.arm2_solver_pick.qdd = KDL::JntArray(state.arm2_solver_pick.num_joints);
        state.arm2_solver_pick.tau_ff = KDL::JntArray(state.arm2_solver_pick.num_joints);
        state.arm2_solver_pick.tau_ctrl = KDL::JntArray(state.arm2_solver_pick.num_joints);
        state.arm2_solver_pick.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_pick.num_spatial_directions = 6;
        state.arm2_solver_pick.spatial_directions = KDL::Jacobian(state.arm2_solver_pick.num_spatial_directions);
        state.arm2_solver_pick.acceleration_energy = KDL::JntArray(state.arm2_solver_pick.num_spatial_directions);
        state.arm2_solver_pick.f_ext = KDL::Wrenches(state.arm2_solver_pick.num_segments);
        state.arm2_solver_pick.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm2_solver_pick.chain, state.arm2_solver_pick.root_acc, state.arm2_solver_pick.num_spatial_directions);
        state.arm2_solver_pick.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_pick.chain, state.arm2_solver_pick.root_acc, state.arm2_solver_pick.num_spatial_directions);
        state.arm2_solver_pick.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_pick.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_pick.initialized = true;
    }
}

inline void update_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_pick(state, robot);

    mj_kdl::update(robot.arm1_solver_pick.robot);
    for (int i = 0; i < state.arm1_solver_pick.num_joints; ++i) {
        state.arm1_solver_pick.q(i) = robot.arm1_solver_pick.robot->jnt_pos_msr[i];
        state.arm1_solver_pick.qd(i) = robot.arm1_solver_pick.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_pick(state.arm1_solver_pick.q, state.arm1_solver_pick.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_pick.robot->model,
                robot.arm1_solver_pick.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_pick;
        mj_kdl::get_body_frame(
                robot.arm1_solver_pick.robot->model,
                robot.arm1_solver_pick.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_pick);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_pick.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_pick.chain);
        fk.JntToCart(
            state.arm1_solver_pick.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_pick.chain);
        fk.JntToCart(
            state.arm1_solver_pick.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_pick.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_pick,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_pick.robot->model,
                robot.arm1_solver_pick.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_pick.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_pick.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_pick.robot);
    for (int i = 0; i < state.arm2_solver_pick.num_joints; ++i) {
        state.arm2_solver_pick.q(i) = robot.arm2_solver_pick.robot->jnt_pos_msr[i];
        state.arm2_solver_pick.qd(i) = robot.arm2_solver_pick.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_pick(state.arm2_solver_pick.q, state.arm2_solver_pick.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_pick.robot->model,
                robot.arm2_solver_pick.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_pick;
        mj_kdl::get_body_frame(
                robot.arm2_solver_pick.robot->model,
                robot.arm2_solver_pick.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_pick);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_pick.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_pick.chain);
        fk.JntToCart(
            state.arm2_solver_pick.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_pick.chain);
        fk.JntToCart(
            state.arm2_solver_pick.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_pick.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_pick,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_pick.robot->model,
                robot.arm2_solver_pick.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_pick.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_pick.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.grasp1_x = shared.pose_cube1_base.p[0];
        shared.grasp1_y = shared.pose_cube1_base.p[1];
        shared.grasp2_x = shared.pose_cube2_base.p[0];
        shared.grasp2_y = shared.pose_cube2_base.p[1];

        shared.pick_support1_z_add_out = shared.pose_elbow1_base.p[2] + shared.pick_support_lift;
        shared.pick_support1_z = shared.pick_support1_z_add_out;

        shared.pick_support2_z_add_out = shared.pose_elbow2_base.p[2] + shared.pick_support_lift;
        shared.pick_support2_z = shared.pick_support2_z_add_out;
        state.snapshot_taken = true;
    }
    shared.pick1_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    shared.pick2_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
}

inline bool can_start_motion_pick(
    shared_data &shared
) {
    // eval_pick_when_aligned1_above
    shared.eval_pick_when_aligned1_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose.p, shared.pose_ee1_base.p);
    // eval_pick_when_aligned2_above
    shared.eval_pick_when_aligned2_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose.p, shared.pose_ee2_base.p);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned1_above_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned2_above_err, shared.satisfied_band));
}

inline void monitor_when_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.pick1_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    shared.pick2_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    // eval_pick_when_aligned1_above
    shared.eval_pick_when_aligned1_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose.p, shared.pose_ee1_base.p);
    // eval_pick_when_aligned2_above
    shared.eval_pick_when_aligned2_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose.p, shared.pose_ee2_base.p);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned1_above_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned2_above_err, shared.satisfied_band));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(7);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PICK_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PICK_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_pick() {
}

inline void monitor_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_pick_when_aligned1_above
    shared.eval_pick_when_aligned1_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose.p, shared.pose_ee1_base.p);
    // eval_pick_when_aligned2_above
    shared.eval_pick_when_aligned2_above_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose.p, shared.pose_ee2_base.p);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned1_above_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_when_aligned2_above_err, shared.satisfied_band));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(7);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PICK_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PICK_READY] << std::endl;
        }
    }

}

inline void control_motion_pick(
    motion_pick_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // profile_lower1_z_ctrl_pk1_lower_z
    {
        if (!state.ctrl_pk1_lower_z.vp_init) {
            state.ctrl_pk1_lower_z.vp_setpoint = shared.pose_ee1_base.p[2];
            state.ctrl_pk1_lower_z.vp_velocity = shared.twist_ee1_base.vel[2];
            state.ctrl_pk1_lower_z.vp_accel = 0.0;
            state.ctrl_pk1_lower_z.vp_init = true;
        }
        shared.lower1_z_ctrl_pk1_lower_z_profile_ref = motion_spec::runtime::velocity_profile_step(
            state.ctrl_pk1_lower_z.vp_setpoint,
            state.ctrl_pk1_lower_z.vp_velocity,
            state.ctrl_pk1_lower_z.vp_accel,
            shared.grasp_z,
            shared.max_lower_velocity,
            shared.max_lower_acceleration,
            shared.max_lower_jerk,
            shared.dt_measured_s,
            motion_spec::runtime::VelocityProfileShape::SCurve);
        state.ctrl_pk1_lower_z.vp_setpoint = shared.lower1_z_ctrl_pk1_lower_z_profile_ref;
    }
    // profile_lower2_z_ctrl_pk2_lower_z
    {
        if (!state.ctrl_pk2_lower_z.vp_init) {
            state.ctrl_pk2_lower_z.vp_setpoint = shared.pose_ee2_base.p[2];
            state.ctrl_pk2_lower_z.vp_velocity = shared.twist_ee2_base.vel[2];
            state.ctrl_pk2_lower_z.vp_accel = 0.0;
            state.ctrl_pk2_lower_z.vp_init = true;
        }
        shared.lower2_z_ctrl_pk2_lower_z_profile_ref = motion_spec::runtime::velocity_profile_step(
            state.ctrl_pk2_lower_z.vp_setpoint,
            state.ctrl_pk2_lower_z.vp_velocity,
            state.ctrl_pk2_lower_z.vp_accel,
            shared.grasp_z,
            shared.max_lower_velocity,
            shared.max_lower_acceleration,
            shared.max_lower_jerk,
            shared.dt_measured_s,
            motion_spec::runtime::VelocityProfileShape::SCurve);
        state.ctrl_pk2_lower_z.vp_setpoint = shared.lower2_z_ctrl_pk2_lower_z_profile_ref;
    }

    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee1_base = shared.pose_ee1_base;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[0] = shared.grasp1_x;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[1] = shared.grasp1_y;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[2] = shared.lower1_z_ctrl_pk1_lower_z_profile_ref;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee1_base = KDL::diff(shared.pose_ee1_base, _pose_axis_target_pose_axis_error_pose_ee1_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.z();

        shared.pose_ee1_base_distance_x_err_pick = _pose_axis_error_linear_X;
        shared.pose_ee1_base_distance_y_err_pick = _pose_axis_error_linear_Y;
        shared.pose_ee1_base_distance_z_err_pick = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee2_base = shared.pose_ee2_base;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[0] = shared.grasp2_x;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[1] = shared.grasp2_y;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[2] = shared.lower2_z_ctrl_pk2_lower_z_profile_ref;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee2_base = KDL::diff(shared.pose_ee2_base, _pose_axis_target_pose_axis_error_pose_ee2_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.z();

        shared.pose_ee2_base_distance_x_err_pick = _pose_axis_error_linear_X;
        shared.pose_ee2_base_distance_y_err_pick = _pose_axis_error_linear_Y;
        shared.pose_ee2_base_distance_z_err_pick = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_pk1_follow_ori
    shared.pose_diff_ctrl_pk1_follow_ori = KDL::diff(shared.pose_ee1_base, shared.pick1_ori_pose);
    shared.ctrl_pk1_follow_ori_err_ang_x = shared.pose_diff_ctrl_pk1_follow_ori.rot[0];
    shared.ctrl_pk1_follow_ori_err_ang_y = shared.pose_diff_ctrl_pk1_follow_ori.rot[1];
    shared.ctrl_pk1_follow_ori_err_ang_z = shared.pose_diff_ctrl_pk1_follow_ori.rot[2];
    // eval_pose_diff_ctrl_pk2_follow_ori
    shared.pose_diff_ctrl_pk2_follow_ori = KDL::diff(shared.pose_ee2_base, shared.pick2_ori_pose);
    shared.ctrl_pk2_follow_ori_err_ang_x = shared.pose_diff_ctrl_pk2_follow_ori.rot[0];
    shared.ctrl_pk2_follow_ori_err_ang_y = shared.pose_diff_ctrl_pk2_follow_ori.rot[1];
    shared.ctrl_pk2_follow_ori_err_ang_z = shared.pose_diff_ctrl_pk2_follow_ori.rot[2];
    // eval_pick_while_support1_elbow_z
    shared.pose_elbow1_base_distance_z_err_pick = motion_spec::runtime::evaluate_equality_constraint(shared.pick_support1_z, shared.pose_elbow1_base.p[2]);
    // eval_pick_while_support2_elbow_z
    shared.pose_elbow2_base_distance_z_err_pick = motion_spec::runtime::evaluate_equality_constraint(shared.pick_support2_z, shared.pose_elbow2_base.p[2]);
    // compute_wrench_force_ctrl_pk1_support_z
    shared.wrench_force_ctrl_pk1_support_z = KDL::Wrench(shared.direction_ctrl_pk1_support_z * shared.force_ctrl_pk1_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pk1_support_z);
    // compute_wrench_force_ctrl_pk2_support_z
    shared.wrench_force_ctrl_pk2_support_z = KDL::Wrench(shared.direction_ctrl_pk2_support_z * shared.force_ctrl_pk2_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pk2_support_z);
    // ctrl_pk2_support_z
    {
        const double _control_signal = state.ctrl_pk2_support_z.control(shared.pose_elbow2_base_distance_z_err_pick, shared.dt_measured_s, {shared.ctrl_pk2_support_z_stiffness, shared.ctrl_pk2_support_z_damping, shared.ctrl_pk2_support_z_integral_gain});
        shared.force_ctrl_pk2_support_z = _control_signal;
        shared.ctrl_pk2_support_z_error_integral = state.ctrl_pk2_support_z.error_integral();
        shared.ctrl_pk2_support_z_previous_error = state.ctrl_pk2_support_z.previous_error();
        shared.ctrl_pk2_support_z_first_sample = state.ctrl_pk2_support_z.is_first_sample();
    }
    // ctrl_pk2_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pk2_follow_ori_ang_z.control(shared.pose_diff_ctrl_pk2_follow_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pk2_follow_ori_ang_z_kp, shared.ctrl_pk2_follow_ori_ang_z_ki, shared.ctrl_pk2_follow_ori_ang_z_kd, shared.ctrl_pk2_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pk2_follow_ori_ang_z = _control_signal;
        shared.ctrl_pk2_follow_ori_ang_z_error_integral = state.ctrl_pk2_follow_ori_ang_z.error_integral();
        shared.ctrl_pk2_follow_ori_ang_z_previous_error = state.ctrl_pk2_follow_ori_ang_z.previous_error();
        shared.ctrl_pk2_follow_ori_ang_z_first_sample = state.ctrl_pk2_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pk2_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pk2_follow_ori_ang_y.control(shared.pose_diff_ctrl_pk2_follow_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pk2_follow_ori_ang_y_kp, shared.ctrl_pk2_follow_ori_ang_y_ki, shared.ctrl_pk2_follow_ori_ang_y_kd, shared.ctrl_pk2_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pk2_follow_ori_ang_y = _control_signal;
        shared.ctrl_pk2_follow_ori_ang_y_error_integral = state.ctrl_pk2_follow_ori_ang_y.error_integral();
        shared.ctrl_pk2_follow_ori_ang_y_previous_error = state.ctrl_pk2_follow_ori_ang_y.previous_error();
        shared.ctrl_pk2_follow_ori_ang_y_first_sample = state.ctrl_pk2_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pk2_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pk2_follow_ori_ang_x.control(shared.pose_diff_ctrl_pk2_follow_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pk2_follow_ori_ang_x_kp, shared.ctrl_pk2_follow_ori_ang_x_ki, shared.ctrl_pk2_follow_ori_ang_x_kd, shared.ctrl_pk2_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pk2_follow_ori_ang_x = _control_signal;
        shared.ctrl_pk2_follow_ori_ang_x_error_integral = state.ctrl_pk2_follow_ori_ang_x.error_integral();
        shared.ctrl_pk2_follow_ori_ang_x_previous_error = state.ctrl_pk2_follow_ori_ang_x.previous_error();
        shared.ctrl_pk2_follow_ori_ang_x_first_sample = state.ctrl_pk2_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pk2_lower_z
    {
        const double _control_signal = state.ctrl_pk2_lower_z.control(shared.pose_ee2_base_distance_z_err_pick, -(shared.twist_ee2_base.vel[2]), shared.dt_measured_s, {shared.ctrl_pk2_lower_z_kp, shared.ctrl_pk2_lower_z_ki, shared.ctrl_pk2_lower_z_kd, shared.ctrl_pk2_lower_z_decay_rate});
        shared.eacc_pose_ee2_base_distance_z_pick = _control_signal;
        shared.ctrl_pk2_lower_z_error_integral = state.ctrl_pk2_lower_z.error_integral();
        shared.ctrl_pk2_lower_z_previous_error = state.ctrl_pk2_lower_z.previous_error();
        shared.ctrl_pk2_lower_z_first_sample = state.ctrl_pk2_lower_z.is_first_sample();
    }
    // ctrl_pk2_hold_y
    {
        const double _control_signal = state.ctrl_pk2_hold_y.control(shared.pose_ee2_base_distance_y_err_pick, shared.dt_measured_s, {shared.ctrl_pk2_hold_y_kp, shared.ctrl_pk2_hold_y_ki, shared.ctrl_pk2_hold_y_kd, shared.ctrl_pk2_hold_y_decay_rate});
        shared.eacc_pose_ee2_base_distance_y_pick = _control_signal;
        shared.ctrl_pk2_hold_y_error_integral = state.ctrl_pk2_hold_y.error_integral();
        shared.ctrl_pk2_hold_y_previous_error = state.ctrl_pk2_hold_y.previous_error();
        shared.ctrl_pk2_hold_y_first_sample = state.ctrl_pk2_hold_y.is_first_sample();
    }
    // ctrl_pk2_hold_x
    {
        const double _control_signal = state.ctrl_pk2_hold_x.control(shared.pose_ee2_base_distance_x_err_pick, shared.dt_measured_s, {shared.ctrl_pk2_hold_x_kp, shared.ctrl_pk2_hold_x_ki, shared.ctrl_pk2_hold_x_kd, shared.ctrl_pk2_hold_x_decay_rate});
        shared.eacc_pose_ee2_base_distance_x_pick = _control_signal;
        shared.ctrl_pk2_hold_x_error_integral = state.ctrl_pk2_hold_x.error_integral();
        shared.ctrl_pk2_hold_x_previous_error = state.ctrl_pk2_hold_x.previous_error();
        shared.ctrl_pk2_hold_x_first_sample = state.ctrl_pk2_hold_x.is_first_sample();
    }
    // ctrl_pk1_support_z
    {
        const double _control_signal = state.ctrl_pk1_support_z.control(shared.pose_elbow1_base_distance_z_err_pick, shared.dt_measured_s, {shared.ctrl_pk1_support_z_stiffness, shared.ctrl_pk1_support_z_damping, shared.ctrl_pk1_support_z_integral_gain});
        shared.force_ctrl_pk1_support_z = _control_signal;
        shared.ctrl_pk1_support_z_error_integral = state.ctrl_pk1_support_z.error_integral();
        shared.ctrl_pk1_support_z_previous_error = state.ctrl_pk1_support_z.previous_error();
        shared.ctrl_pk1_support_z_first_sample = state.ctrl_pk1_support_z.is_first_sample();
    }
    // ctrl_pk1_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pk1_follow_ori_ang_z.control(shared.pose_diff_ctrl_pk1_follow_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pk1_follow_ori_ang_z_kp, shared.ctrl_pk1_follow_ori_ang_z_ki, shared.ctrl_pk1_follow_ori_ang_z_kd, shared.ctrl_pk1_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pk1_follow_ori_ang_z = _control_signal;
        shared.ctrl_pk1_follow_ori_ang_z_error_integral = state.ctrl_pk1_follow_ori_ang_z.error_integral();
        shared.ctrl_pk1_follow_ori_ang_z_previous_error = state.ctrl_pk1_follow_ori_ang_z.previous_error();
        shared.ctrl_pk1_follow_ori_ang_z_first_sample = state.ctrl_pk1_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pk1_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pk1_follow_ori_ang_y.control(shared.pose_diff_ctrl_pk1_follow_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pk1_follow_ori_ang_y_kp, shared.ctrl_pk1_follow_ori_ang_y_ki, shared.ctrl_pk1_follow_ori_ang_y_kd, shared.ctrl_pk1_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pk1_follow_ori_ang_y = _control_signal;
        shared.ctrl_pk1_follow_ori_ang_y_error_integral = state.ctrl_pk1_follow_ori_ang_y.error_integral();
        shared.ctrl_pk1_follow_ori_ang_y_previous_error = state.ctrl_pk1_follow_ori_ang_y.previous_error();
        shared.ctrl_pk1_follow_ori_ang_y_first_sample = state.ctrl_pk1_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pk1_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pk1_follow_ori_ang_x.control(shared.pose_diff_ctrl_pk1_follow_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pk1_follow_ori_ang_x_kp, shared.ctrl_pk1_follow_ori_ang_x_ki, shared.ctrl_pk1_follow_ori_ang_x_kd, shared.ctrl_pk1_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pk1_follow_ori_ang_x = _control_signal;
        shared.ctrl_pk1_follow_ori_ang_x_error_integral = state.ctrl_pk1_follow_ori_ang_x.error_integral();
        shared.ctrl_pk1_follow_ori_ang_x_previous_error = state.ctrl_pk1_follow_ori_ang_x.previous_error();
        shared.ctrl_pk1_follow_ori_ang_x_first_sample = state.ctrl_pk1_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pk1_lower_z
    {
        const double _control_signal = state.ctrl_pk1_lower_z.control(shared.pose_ee1_base_distance_z_err_pick, -(shared.twist_ee1_base.vel[2]), shared.dt_measured_s, {shared.ctrl_pk1_lower_z_kp, shared.ctrl_pk1_lower_z_ki, shared.ctrl_pk1_lower_z_kd, shared.ctrl_pk1_lower_z_decay_rate});
        shared.eacc_pose_ee1_base_distance_z_pick = _control_signal;
        shared.ctrl_pk1_lower_z_error_integral = state.ctrl_pk1_lower_z.error_integral();
        shared.ctrl_pk1_lower_z_previous_error = state.ctrl_pk1_lower_z.previous_error();
        shared.ctrl_pk1_lower_z_first_sample = state.ctrl_pk1_lower_z.is_first_sample();
    }
    // ctrl_pk1_hold_y
    {
        const double _control_signal = state.ctrl_pk1_hold_y.control(shared.pose_ee1_base_distance_y_err_pick, shared.dt_measured_s, {shared.ctrl_pk1_hold_y_kp, shared.ctrl_pk1_hold_y_ki, shared.ctrl_pk1_hold_y_kd, shared.ctrl_pk1_hold_y_decay_rate});
        shared.eacc_pose_ee1_base_distance_y_pick = _control_signal;
        shared.ctrl_pk1_hold_y_error_integral = state.ctrl_pk1_hold_y.error_integral();
        shared.ctrl_pk1_hold_y_previous_error = state.ctrl_pk1_hold_y.previous_error();
        shared.ctrl_pk1_hold_y_first_sample = state.ctrl_pk1_hold_y.is_first_sample();
    }
    // ctrl_pk1_hold_x
    {
        const double _control_signal = state.ctrl_pk1_hold_x.control(shared.pose_ee1_base_distance_x_err_pick, shared.dt_measured_s, {shared.ctrl_pk1_hold_x_kp, shared.ctrl_pk1_hold_x_ki, shared.ctrl_pk1_hold_x_kd, shared.ctrl_pk1_hold_x_decay_rate});
        shared.eacc_pose_ee1_base_distance_x_pick = _control_signal;
        shared.ctrl_pk1_hold_x_error_integral = state.ctrl_pk1_hold_x.error_integral();
        shared.ctrl_pk1_hold_x_previous_error = state.ctrl_pk1_hold_x.previous_error();
        shared.ctrl_pk1_hold_x_first_sample = state.ctrl_pk1_hold_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_pick.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_pick_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_0(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_0.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_0 =
            alpha_frame_arm1_solver_pick_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_pick_0[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_pick_0[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_pick_0[2];
    }

    state.arm1_solver_pick.acceleration_energy(0) = shared.eacc_pose_ee1_base_distance_x_pick;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_1(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_1.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_1 =
            alpha_frame_arm1_solver_pick_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_pick_1[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_pick_1[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_pick_1[2];
    }

    state.arm1_solver_pick.acceleration_energy(1) = shared.eacc_pose_ee1_base_distance_y_pick;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_2(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_2.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_2 =
            alpha_frame_arm1_solver_pick_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_pick_2[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_pick_2[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_pick_2[2];
    }

    state.arm1_solver_pick.acceleration_energy(2) = shared.eacc_pose_ee1_base_distance_z_pick;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_3(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_3.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_3 =
            alpha_frame_arm1_solver_pick_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_pick_3[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_pick_3[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_pick_3[2];
    }

    state.arm1_solver_pick.acceleration_energy(3) = shared.eacc_ctrl_pk1_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_4(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_4.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_4 =
            alpha_frame_arm1_solver_pick_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_pick_4[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_pick_4[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_pick_4[2];
    }

    state.arm1_solver_pick.acceleration_energy(4) = shared.eacc_ctrl_pk1_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_5(*robot.arm1_solver_pick.chain);
        alpha_fk_arm1_solver_pick_5.JntToCart(
            state.arm1_solver_pick.q,
            alpha_frame_arm1_solver_pick_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_5 =
            alpha_frame_arm1_solver_pick_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_pick_5[0];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_pick_5[1];
        state.arm1_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_pick_5[2];
    }

    state.arm1_solver_pick.acceleration_energy(5) = shared.eacc_ctrl_pk1_follow_ori_ang_z;

    KDL::SetToZero(state.arm1_solver_pick.tau_ff);

    for (int i = 0; i < state.arm1_solver_pick.num_segments; ++i) {
        KDL::SetToZero(state.arm1_solver_pick.f_ext[i]);
    }

    state.arm1_solver_pick.f_ext[motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick.chain, "half_arm_2_link", "kinova1_base_link") - 1] += shared.wrench_force_ctrl_pk1_support_z;

    KDL::Wrenches f_ext_zero_arm1_solver_pick(state.arm1_solver_pick.num_segments);
    for (int i = 0; i < state.arm1_solver_pick.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_pick[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_pick(state.arm1_solver_pick.num_joints);
    state.arm1_solver_pick.achd_acc->CartToJnt(
        state.arm1_solver_pick.q,
        state.arm1_solver_pick.qd,
        state.arm1_solver_pick.qdd,
        state.arm1_solver_pick.spatial_directions,
        state.arm1_solver_pick.acceleration_energy,
        state.arm1_solver_pick.f_ext,
        state.arm1_solver_pick.tau_ff,
        tau_ctrl_acc_arm1_solver_pick);
    state.arm1_solver_pick.rnea->CartToJnt(
        state.arm1_solver_pick.q,
        state.arm1_solver_pick.qd,
        state.arm1_solver_pick.qdd,
        f_ext_zero_arm1_solver_pick,
        state.arm1_solver_pick.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_pick.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_pick.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_pick.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_pick.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_pick.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_pick.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_pick.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_pick.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_pick.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_pick.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_pick.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_pick.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_pick.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_pick.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_pick.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_pick.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_pick.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_pick.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_pick.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_pick.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_pick.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_pick.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_pick.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_pick.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_pick.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_pick.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_pick.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_pick.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_pick.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_pick_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_0(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_0.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_0 =
            alpha_frame_arm2_solver_pick_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_pick_0[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_pick_0[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_pick_0[2];
    }

    state.arm2_solver_pick.acceleration_energy(0) = shared.eacc_pose_ee2_base_distance_x_pick;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_1(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_1.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_1 =
            alpha_frame_arm2_solver_pick_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_pick_1[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_pick_1[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_pick_1[2];
    }

    state.arm2_solver_pick.acceleration_energy(1) = shared.eacc_pose_ee2_base_distance_y_pick;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_2(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_2.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_2 =
            alpha_frame_arm2_solver_pick_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_pick_2[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_pick_2[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_pick_2[2];
    }

    state.arm2_solver_pick.acceleration_energy(2) = shared.eacc_pose_ee2_base_distance_z_pick;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_3(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_3.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_3 =
            alpha_frame_arm2_solver_pick_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_pick_3[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_pick_3[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_pick_3[2];
    }

    state.arm2_solver_pick.acceleration_energy(3) = shared.eacc_ctrl_pk2_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_4(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_4.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_4 =
            alpha_frame_arm2_solver_pick_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_pick_4[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_pick_4[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_pick_4[2];
    }

    state.arm2_solver_pick.acceleration_energy(4) = shared.eacc_ctrl_pk2_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_5(*robot.arm2_solver_pick.chain);
        alpha_fk_arm2_solver_pick_5.JntToCart(
            state.arm2_solver_pick.q,
            alpha_frame_arm2_solver_pick_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_5 =
            alpha_frame_arm2_solver_pick_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_pick_5[0];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_pick_5[1];
        state.arm2_solver_pick.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_pick_5[2];
    }

    state.arm2_solver_pick.acceleration_energy(5) = shared.eacc_ctrl_pk2_follow_ori_ang_z;

    KDL::SetToZero(state.arm2_solver_pick.tau_ff);

    for (int i = 0; i < state.arm2_solver_pick.num_segments; ++i) {
        KDL::SetToZero(state.arm2_solver_pick.f_ext[i]);
    }

    state.arm2_solver_pick.f_ext[motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick.chain, "half_arm_2_link", "kinova2_base_link") - 1] += shared.wrench_force_ctrl_pk2_support_z;

    KDL::Wrenches f_ext_zero_arm2_solver_pick(state.arm2_solver_pick.num_segments);
    for (int i = 0; i < state.arm2_solver_pick.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_pick[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_pick(state.arm2_solver_pick.num_joints);
    state.arm2_solver_pick.achd_acc->CartToJnt(
        state.arm2_solver_pick.q,
        state.arm2_solver_pick.qd,
        state.arm2_solver_pick.qdd,
        state.arm2_solver_pick.spatial_directions,
        state.arm2_solver_pick.acceleration_energy,
        state.arm2_solver_pick.f_ext,
        state.arm2_solver_pick.tau_ff,
        tau_ctrl_acc_arm2_solver_pick);
    state.arm2_solver_pick.rnea->CartToJnt(
        state.arm2_solver_pick.q,
        state.arm2_solver_pick.qd,
        state.arm2_solver_pick.qdd,
        f_ext_zero_arm2_solver_pick,
        state.arm2_solver_pick.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_pick.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_pick.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_pick.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_pick.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_pick.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_pick.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_pick.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_pick.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_pick.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_pick.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_pick.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_pick.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_pick.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_pick.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_pick.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_pick.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_pick.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_pick.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_pick.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_pick.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_pick.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_pick.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_pick.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_pick.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_pick.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_pick.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_pick.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_pick.tau_ctrl(6);

}

inline void apply_motion_pick(
    motion_pick_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_pick.num_joints; ++i) {
        robot.arm1_solver_pick.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_pick.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_pick.num_joints; ++i) {
        robot.arm2_solver_pick.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_pick.tau_ctrl(i), i);
    }

}
