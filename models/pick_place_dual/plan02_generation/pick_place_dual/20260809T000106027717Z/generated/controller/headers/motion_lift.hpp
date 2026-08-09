/// Motion: lift
/// Raise both TCPs straight up while holding the cubes
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_lift_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_lift_solver_state arm1_solver_lift;
    arm2_solver_lift_solver_state arm2_solver_lift;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_lt1_hold_x;
    motion_spec::runtime::PIDControl ctrl_lt1_hold_y;
    motion_spec::runtime::PIDControl ctrl_lt1_lift_z;
    motion_spec::runtime::PIDControl ctrl_lt1_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_lt1_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_lt1_follow_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_lt1_support_z;

    motion_spec::runtime::PIDControl ctrl_lt2_hold_x;
    motion_spec::runtime::PIDControl ctrl_lt2_hold_y;
    motion_spec::runtime::PIDControl ctrl_lt2_lift_z;
    motion_spec::runtime::PIDControl ctrl_lt2_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_lt2_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_lt2_follow_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_lt2_support_z;

    bool mon_lift_ready_previous = false;
    bool mon_lift_ready_event_triggered = false;

    bool mon_lift_grasp_lost_previous = false;
    bool mon_lift_grasp_lost_event_triggered = false;

};

inline void reset_motion_lift(motion_lift_state &state) {
    state = motion_lift_state{};
}

inline void init_motion_lift(motion_lift_state &state, const robot_io &robot) {
    if (!state.arm1_solver_lift.initialized) {
        state.arm1_solver_lift.num_joints = robot.arm1_solver_lift.chain->getNrOfJoints();
        state.arm1_solver_lift.num_segments = robot.arm1_solver_lift.chain->getNrOfSegments();
        state.arm1_solver_lift.q = KDL::JntArray(state.arm1_solver_lift.num_joints);
        state.arm1_solver_lift.qd = KDL::JntArray(state.arm1_solver_lift.num_joints);
        state.arm1_solver_lift.qdd = KDL::JntArray(state.arm1_solver_lift.num_joints);
        state.arm1_solver_lift.tau_ff = KDL::JntArray(state.arm1_solver_lift.num_joints);
        state.arm1_solver_lift.tau_ctrl = KDL::JntArray(state.arm1_solver_lift.num_joints);
        state.arm1_solver_lift.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_lift.num_spatial_directions = 6;
        state.arm1_solver_lift.spatial_directions = KDL::Jacobian(state.arm1_solver_lift.num_spatial_directions);
        state.arm1_solver_lift.acceleration_energy = KDL::JntArray(state.arm1_solver_lift.num_spatial_directions);
        state.arm1_solver_lift.f_ext = KDL::Wrenches(state.arm1_solver_lift.num_segments);
        state.arm1_solver_lift.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm1_solver_lift.chain, state.arm1_solver_lift.root_acc, state.arm1_solver_lift.num_spatial_directions);
        state.arm1_solver_lift.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_lift.chain, state.arm1_solver_lift.root_acc, state.arm1_solver_lift.num_spatial_directions);
        state.arm1_solver_lift.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_lift.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_lift.initialized = true;
    }
    if (!state.arm2_solver_lift.initialized) {
        state.arm2_solver_lift.num_joints = robot.arm2_solver_lift.chain->getNrOfJoints();
        state.arm2_solver_lift.num_segments = robot.arm2_solver_lift.chain->getNrOfSegments();
        state.arm2_solver_lift.q = KDL::JntArray(state.arm2_solver_lift.num_joints);
        state.arm2_solver_lift.qd = KDL::JntArray(state.arm2_solver_lift.num_joints);
        state.arm2_solver_lift.qdd = KDL::JntArray(state.arm2_solver_lift.num_joints);
        state.arm2_solver_lift.tau_ff = KDL::JntArray(state.arm2_solver_lift.num_joints);
        state.arm2_solver_lift.tau_ctrl = KDL::JntArray(state.arm2_solver_lift.num_joints);
        state.arm2_solver_lift.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_lift.num_spatial_directions = 6;
        state.arm2_solver_lift.spatial_directions = KDL::Jacobian(state.arm2_solver_lift.num_spatial_directions);
        state.arm2_solver_lift.acceleration_energy = KDL::JntArray(state.arm2_solver_lift.num_spatial_directions);
        state.arm2_solver_lift.f_ext = KDL::Wrenches(state.arm2_solver_lift.num_segments);
        state.arm2_solver_lift.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm2_solver_lift.chain, state.arm2_solver_lift.root_acc, state.arm2_solver_lift.num_spatial_directions);
        state.arm2_solver_lift.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_lift.chain, state.arm2_solver_lift.root_acc, state.arm2_solver_lift.num_spatial_directions);
        state.arm2_solver_lift.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_lift.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_lift.initialized = true;
    }
}

inline void update_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_lift(state, robot);

    mj_kdl::update(robot.arm1_solver_lift.robot);
    for (int i = 0; i < state.arm1_solver_lift.num_joints; ++i) {
        state.arm1_solver_lift.q(i) = robot.arm1_solver_lift.robot->jnt_pos_msr[i];
        state.arm1_solver_lift.qd(i) = robot.arm1_solver_lift.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_lift(state.arm1_solver_lift.q, state.arm1_solver_lift.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_lift.robot->model,
                robot.arm1_solver_lift.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_lift;
        mj_kdl::get_body_frame(
                robot.arm1_solver_lift.robot->model,
                robot.arm1_solver_lift.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_lift);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_lift.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_lift.chain);
        fk.JntToCart(
            state.arm1_solver_lift.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_lift.chain);
        fk.JntToCart(
            state.arm1_solver_lift.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_lift.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_lift,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_lift.robot->model,
                robot.arm1_solver_lift.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_lift.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_lift.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_lift.robot);
    for (int i = 0; i < state.arm2_solver_lift.num_joints; ++i) {
        state.arm2_solver_lift.q(i) = robot.arm2_solver_lift.robot->jnt_pos_msr[i];
        state.arm2_solver_lift.qd(i) = robot.arm2_solver_lift.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_lift(state.arm2_solver_lift.q, state.arm2_solver_lift.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_lift.robot->model,
                robot.arm2_solver_lift.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_lift;
        mj_kdl::get_body_frame(
                robot.arm2_solver_lift.robot->model,
                robot.arm2_solver_lift.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_lift);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_lift.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_lift.chain);
        fk.JntToCart(
            state.arm2_solver_lift.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_lift.chain);
        fk.JntToCart(
            state.arm2_solver_lift.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_lift.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_lift,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_lift.robot->model,
                robot.arm2_solver_lift.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_lift.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_lift.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.lift1_start_x = shared.pose_ee1_base.p[0];
        shared.lift1_start_y = shared.pose_ee1_base.p[1];
        shared.lift2_start_x = shared.pose_ee2_base.p[0];
        shared.lift2_start_y = shared.pose_ee2_base.p[1];

        shared.lift_support1_z_add_out = shared.pose_elbow1_base.p[2] + shared.lift_support_lift;
        shared.lift_support1_z = shared.lift_support1_z_add_out;

        shared.lift_support2_z_add_out = shared.pose_elbow2_base.p[2] + shared.lift_support_lift;
        shared.lift_support2_z = shared.lift_support2_z_add_out;
        state.snapshot_taken = true;
    }
    shared.lift1_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    shared.lift2_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
}

inline bool can_start_motion_lift(
    shared_data &shared
) {
    // eval_lift_when_grasped1
    shared.eval_lift_when_grasped1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper1_pos, shared.grasp_threshold);
    // eval_lift_when_grasped2
    shared.eval_lift_when_grasped2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper2_pos, shared.grasp_threshold);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped1_err, shared.default_tolerance_Angle) && motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped2_err, shared.default_tolerance_Angle));
}

inline void monitor_when_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.lift1_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    shared.lift2_ori_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.0, 0.0, 0.0));
    // eval_lift_when_grasped1
    shared.eval_lift_when_grasped1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper1_pos, shared.grasp_threshold);
    // eval_lift_when_grasped2
    shared.eval_lift_when_grasped2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper2_pos, shared.grasp_threshold);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped1_err, shared.default_tolerance_Angle) && motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped2_err, shared.default_tolerance_Angle));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_lift_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(4);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_LIFT_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_LIFT_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // distance_grasp_lost1_distance_derived_invert_start
    shared.distance_grasp_lost1_distance_derived_inverse_start = shared.pose_ee1_base.Inverse();
    // distance_grasp_lost1_distance_derived_compose_relative_pose
    shared.distance_grasp_lost1_distance_derived_relative_pose = shared.distance_grasp_lost1_distance_derived_inverse_start * shared.pose_cube1_base;
    // distance_grasp_lost1_distance_derived_magnitude
    shared.distance_grasp_lost1_distance = shared.distance_grasp_lost1_distance_derived_relative_pose.p.Norm();
    // distance_grasp_lost2_distance_derived_invert_start
    shared.distance_grasp_lost2_distance_derived_inverse_start = shared.pose_ee2_base.Inverse();
    // distance_grasp_lost2_distance_derived_compose_relative_pose
    shared.distance_grasp_lost2_distance_derived_relative_pose = shared.distance_grasp_lost2_distance_derived_inverse_start * shared.pose_cube2_base;
    // distance_grasp_lost2_distance_derived_magnitude
    shared.distance_grasp_lost2_distance = shared.distance_grasp_lost2_distance_derived_relative_pose.p.Norm();
    // eval_lift_until_grasp_lost1
    shared.eval_lift_until_grasp_lost1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost1_distance, shared.lost_dist);
    // eval_lift_until_grasp_lost2
    shared.eval_lift_until_grasp_lost2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost2_distance, shared.lost_dist);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_lift_until_grasp_lost1_err, shared.default_tolerance_Distance) || motion_spec::runtime::constraint_satisfied(shared.eval_lift_until_grasp_lost2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_lift_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_GRASP_LOST_LIFT);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_GRASP_LOST_LIFT] << std::endl;
        }
    }

}

inline void monitor_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_lift_when_grasped1
    shared.eval_lift_when_grasped1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper1_pos, shared.grasp_threshold);
    // eval_lift_when_grasped2
    shared.eval_lift_when_grasped2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.gripper2_pos, shared.grasp_threshold);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped1_err, shared.default_tolerance_Angle) && motion_spec::runtime::constraint_satisfied(shared.eval_lift_when_grasped2_err, shared.default_tolerance_Angle));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_lift_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(4);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_LIFT_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_LIFT_READY] << std::endl;
        }
    }

    // distance_grasp_lost1_distance_derived_invert_start
    shared.distance_grasp_lost1_distance_derived_inverse_start = shared.pose_ee1_base.Inverse();
    // distance_grasp_lost1_distance_derived_compose_relative_pose
    shared.distance_grasp_lost1_distance_derived_relative_pose = shared.distance_grasp_lost1_distance_derived_inverse_start * shared.pose_cube1_base;
    // distance_grasp_lost1_distance_derived_magnitude
    shared.distance_grasp_lost1_distance = shared.distance_grasp_lost1_distance_derived_relative_pose.p.Norm();
    // distance_grasp_lost2_distance_derived_invert_start
    shared.distance_grasp_lost2_distance_derived_inverse_start = shared.pose_ee2_base.Inverse();
    // distance_grasp_lost2_distance_derived_compose_relative_pose
    shared.distance_grasp_lost2_distance_derived_relative_pose = shared.distance_grasp_lost2_distance_derived_inverse_start * shared.pose_cube2_base;
    // distance_grasp_lost2_distance_derived_magnitude
    shared.distance_grasp_lost2_distance = shared.distance_grasp_lost2_distance_derived_relative_pose.p.Norm();
    // eval_lift_until_grasp_lost1
    shared.eval_lift_until_grasp_lost1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost1_distance, shared.lost_dist);
    // eval_lift_until_grasp_lost2
    shared.eval_lift_until_grasp_lost2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost2_distance, shared.lost_dist);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_lift_until_grasp_lost1_err, shared.default_tolerance_Distance) || motion_spec::runtime::constraint_satisfied(shared.eval_lift_until_grasp_lost2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_lift_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(0);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_GRASP_LOST_LIFT);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_GRASP_LOST_LIFT] << std::endl;
        }
    }

}

inline void control_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee1_base = shared.pose_ee1_base;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[0] = shared.lift1_start_x;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[1] = shared.lift1_start_y;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[2] = shared.lift_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee1_base = KDL::diff(shared.pose_ee1_base, _pose_axis_target_pose_axis_error_pose_ee1_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.z();

        shared.pose_ee1_base_distance_x_err_lift = _pose_axis_error_linear_X;
        shared.pose_ee1_base_distance_y_err_lift = _pose_axis_error_linear_Y;
        shared.pose_ee1_base_distance_z_err_lift = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee2_base = shared.pose_ee2_base;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[0] = shared.lift2_start_x;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[1] = shared.lift2_start_y;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[2] = shared.lift_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee2_base = KDL::diff(shared.pose_ee2_base, _pose_axis_target_pose_axis_error_pose_ee2_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.z();

        shared.pose_ee2_base_distance_x_err_lift = _pose_axis_error_linear_X;
        shared.pose_ee2_base_distance_y_err_lift = _pose_axis_error_linear_Y;
        shared.pose_ee2_base_distance_z_err_lift = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_lt1_follow_ori
    shared.pose_diff_ctrl_lt1_follow_ori = KDL::diff(shared.pose_ee1_base, shared.lift1_ori_pose);
    shared.ctrl_lt1_follow_ori_err_ang_x = shared.pose_diff_ctrl_lt1_follow_ori.rot[0];
    shared.ctrl_lt1_follow_ori_err_ang_y = shared.pose_diff_ctrl_lt1_follow_ori.rot[1];
    shared.ctrl_lt1_follow_ori_err_ang_z = shared.pose_diff_ctrl_lt1_follow_ori.rot[2];
    // eval_pose_diff_ctrl_lt2_follow_ori
    shared.pose_diff_ctrl_lt2_follow_ori = KDL::diff(shared.pose_ee2_base, shared.lift2_ori_pose);
    shared.ctrl_lt2_follow_ori_err_ang_x = shared.pose_diff_ctrl_lt2_follow_ori.rot[0];
    shared.ctrl_lt2_follow_ori_err_ang_y = shared.pose_diff_ctrl_lt2_follow_ori.rot[1];
    shared.ctrl_lt2_follow_ori_err_ang_z = shared.pose_diff_ctrl_lt2_follow_ori.rot[2];
    // eval_lift_while_support1_elbow_z
    shared.pose_elbow1_base_distance_z_err_lift = motion_spec::runtime::evaluate_equality_constraint(shared.lift_support1_z, shared.pose_elbow1_base.p[2]);
    // eval_lift_while_support2_elbow_z
    shared.pose_elbow2_base_distance_z_err_lift = motion_spec::runtime::evaluate_equality_constraint(shared.lift_support2_z, shared.pose_elbow2_base.p[2]);
    // compute_wrench_force_ctrl_lt1_support_z
    shared.wrench_force_ctrl_lt1_support_z = KDL::Wrench(shared.direction_ctrl_lt1_support_z * shared.force_ctrl_lt1_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_lt1_support_z);
    // compute_wrench_force_ctrl_lt2_support_z
    shared.wrench_force_ctrl_lt2_support_z = KDL::Wrench(shared.direction_ctrl_lt2_support_z * shared.force_ctrl_lt2_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_lt2_support_z);
    // eval_lift_while_close1_gripper
    shared.gripper1_pos_err_lift = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper1_pos);
    // eval_lift_while_close2_gripper
    shared.gripper2_pos_err_lift = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper2_pos);
    // ctrl_lt2_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_lt2_close_gripper = _control_signal;
    }
    // ctrl_lt2_support_z
    {
        const double _control_signal = state.ctrl_lt2_support_z.control(shared.pose_elbow2_base_distance_z_err_lift, shared.dt_measured_s, {shared.ctrl_lt2_support_z_stiffness, shared.ctrl_lt2_support_z_damping, shared.ctrl_lt2_support_z_integral_gain});
        shared.force_ctrl_lt2_support_z = _control_signal;
        shared.ctrl_lt2_support_z_error_integral = state.ctrl_lt2_support_z.error_integral();
        shared.ctrl_lt2_support_z_previous_error = state.ctrl_lt2_support_z.previous_error();
        shared.ctrl_lt2_support_z_first_sample = state.ctrl_lt2_support_z.is_first_sample();
    }
    // ctrl_lt2_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_lt2_follow_ori_ang_z.control(shared.pose_diff_ctrl_lt2_follow_ori.rot[2], shared.dt_measured_s, {shared.ctrl_lt2_follow_ori_ang_z_kp, shared.ctrl_lt2_follow_ori_ang_z_ki, shared.ctrl_lt2_follow_ori_ang_z_kd, shared.ctrl_lt2_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_lt2_follow_ori_ang_z = _control_signal;
        shared.ctrl_lt2_follow_ori_ang_z_error_integral = state.ctrl_lt2_follow_ori_ang_z.error_integral();
        shared.ctrl_lt2_follow_ori_ang_z_previous_error = state.ctrl_lt2_follow_ori_ang_z.previous_error();
        shared.ctrl_lt2_follow_ori_ang_z_first_sample = state.ctrl_lt2_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_lt2_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_lt2_follow_ori_ang_y.control(shared.pose_diff_ctrl_lt2_follow_ori.rot[1], shared.dt_measured_s, {shared.ctrl_lt2_follow_ori_ang_y_kp, shared.ctrl_lt2_follow_ori_ang_y_ki, shared.ctrl_lt2_follow_ori_ang_y_kd, shared.ctrl_lt2_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_lt2_follow_ori_ang_y = _control_signal;
        shared.ctrl_lt2_follow_ori_ang_y_error_integral = state.ctrl_lt2_follow_ori_ang_y.error_integral();
        shared.ctrl_lt2_follow_ori_ang_y_previous_error = state.ctrl_lt2_follow_ori_ang_y.previous_error();
        shared.ctrl_lt2_follow_ori_ang_y_first_sample = state.ctrl_lt2_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_lt2_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_lt2_follow_ori_ang_x.control(shared.pose_diff_ctrl_lt2_follow_ori.rot[0], shared.dt_measured_s, {shared.ctrl_lt2_follow_ori_ang_x_kp, shared.ctrl_lt2_follow_ori_ang_x_ki, shared.ctrl_lt2_follow_ori_ang_x_kd, shared.ctrl_lt2_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_lt2_follow_ori_ang_x = _control_signal;
        shared.ctrl_lt2_follow_ori_ang_x_error_integral = state.ctrl_lt2_follow_ori_ang_x.error_integral();
        shared.ctrl_lt2_follow_ori_ang_x_previous_error = state.ctrl_lt2_follow_ori_ang_x.previous_error();
        shared.ctrl_lt2_follow_ori_ang_x_first_sample = state.ctrl_lt2_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_lt2_lift_z
    {
        const double _control_signal = state.ctrl_lt2_lift_z.control(shared.pose_ee2_base_distance_z_err_lift, shared.dt_measured_s, {shared.ctrl_lt2_lift_z_kp, shared.ctrl_lt2_lift_z_ki, shared.ctrl_lt2_lift_z_kd, shared.ctrl_lt2_lift_z_decay_rate});
        shared.eacc_pose_ee2_base_distance_z_lift = _control_signal;
        shared.ctrl_lt2_lift_z_error_integral = state.ctrl_lt2_lift_z.error_integral();
        shared.ctrl_lt2_lift_z_previous_error = state.ctrl_lt2_lift_z.previous_error();
        shared.ctrl_lt2_lift_z_first_sample = state.ctrl_lt2_lift_z.is_first_sample();
    }
    // ctrl_lt2_hold_y
    {
        const double _control_signal = state.ctrl_lt2_hold_y.control(shared.pose_ee2_base_distance_y_err_lift, shared.dt_measured_s, {shared.ctrl_lt2_hold_y_kp, shared.ctrl_lt2_hold_y_ki, shared.ctrl_lt2_hold_y_kd, shared.ctrl_lt2_hold_y_decay_rate});
        shared.eacc_pose_ee2_base_distance_y_lift = _control_signal;
        shared.ctrl_lt2_hold_y_error_integral = state.ctrl_lt2_hold_y.error_integral();
        shared.ctrl_lt2_hold_y_previous_error = state.ctrl_lt2_hold_y.previous_error();
        shared.ctrl_lt2_hold_y_first_sample = state.ctrl_lt2_hold_y.is_first_sample();
    }
    // ctrl_lt2_hold_x
    {
        const double _control_signal = state.ctrl_lt2_hold_x.control(shared.pose_ee2_base_distance_x_err_lift, shared.dt_measured_s, {shared.ctrl_lt2_hold_x_kp, shared.ctrl_lt2_hold_x_ki, shared.ctrl_lt2_hold_x_kd, shared.ctrl_lt2_hold_x_decay_rate});
        shared.eacc_pose_ee2_base_distance_x_lift = _control_signal;
        shared.ctrl_lt2_hold_x_error_integral = state.ctrl_lt2_hold_x.error_integral();
        shared.ctrl_lt2_hold_x_previous_error = state.ctrl_lt2_hold_x.previous_error();
        shared.ctrl_lt2_hold_x_first_sample = state.ctrl_lt2_hold_x.is_first_sample();
    }
    // ctrl_lt1_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_lt1_close_gripper = _control_signal;
    }
    // ctrl_lt1_support_z
    {
        const double _control_signal = state.ctrl_lt1_support_z.control(shared.pose_elbow1_base_distance_z_err_lift, shared.dt_measured_s, {shared.ctrl_lt1_support_z_stiffness, shared.ctrl_lt1_support_z_damping, shared.ctrl_lt1_support_z_integral_gain});
        shared.force_ctrl_lt1_support_z = _control_signal;
        shared.ctrl_lt1_support_z_error_integral = state.ctrl_lt1_support_z.error_integral();
        shared.ctrl_lt1_support_z_previous_error = state.ctrl_lt1_support_z.previous_error();
        shared.ctrl_lt1_support_z_first_sample = state.ctrl_lt1_support_z.is_first_sample();
    }
    // ctrl_lt1_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_lt1_follow_ori_ang_z.control(shared.pose_diff_ctrl_lt1_follow_ori.rot[2], shared.dt_measured_s, {shared.ctrl_lt1_follow_ori_ang_z_kp, shared.ctrl_lt1_follow_ori_ang_z_ki, shared.ctrl_lt1_follow_ori_ang_z_kd, shared.ctrl_lt1_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_lt1_follow_ori_ang_z = _control_signal;
        shared.ctrl_lt1_follow_ori_ang_z_error_integral = state.ctrl_lt1_follow_ori_ang_z.error_integral();
        shared.ctrl_lt1_follow_ori_ang_z_previous_error = state.ctrl_lt1_follow_ori_ang_z.previous_error();
        shared.ctrl_lt1_follow_ori_ang_z_first_sample = state.ctrl_lt1_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_lt1_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_lt1_follow_ori_ang_y.control(shared.pose_diff_ctrl_lt1_follow_ori.rot[1], shared.dt_measured_s, {shared.ctrl_lt1_follow_ori_ang_y_kp, shared.ctrl_lt1_follow_ori_ang_y_ki, shared.ctrl_lt1_follow_ori_ang_y_kd, shared.ctrl_lt1_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_lt1_follow_ori_ang_y = _control_signal;
        shared.ctrl_lt1_follow_ori_ang_y_error_integral = state.ctrl_lt1_follow_ori_ang_y.error_integral();
        shared.ctrl_lt1_follow_ori_ang_y_previous_error = state.ctrl_lt1_follow_ori_ang_y.previous_error();
        shared.ctrl_lt1_follow_ori_ang_y_first_sample = state.ctrl_lt1_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_lt1_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_lt1_follow_ori_ang_x.control(shared.pose_diff_ctrl_lt1_follow_ori.rot[0], shared.dt_measured_s, {shared.ctrl_lt1_follow_ori_ang_x_kp, shared.ctrl_lt1_follow_ori_ang_x_ki, shared.ctrl_lt1_follow_ori_ang_x_kd, shared.ctrl_lt1_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_lt1_follow_ori_ang_x = _control_signal;
        shared.ctrl_lt1_follow_ori_ang_x_error_integral = state.ctrl_lt1_follow_ori_ang_x.error_integral();
        shared.ctrl_lt1_follow_ori_ang_x_previous_error = state.ctrl_lt1_follow_ori_ang_x.previous_error();
        shared.ctrl_lt1_follow_ori_ang_x_first_sample = state.ctrl_lt1_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_lt1_lift_z
    {
        const double _control_signal = state.ctrl_lt1_lift_z.control(shared.pose_ee1_base_distance_z_err_lift, shared.dt_measured_s, {shared.ctrl_lt1_lift_z_kp, shared.ctrl_lt1_lift_z_ki, shared.ctrl_lt1_lift_z_kd, shared.ctrl_lt1_lift_z_decay_rate});
        shared.eacc_pose_ee1_base_distance_z_lift = _control_signal;
        shared.ctrl_lt1_lift_z_error_integral = state.ctrl_lt1_lift_z.error_integral();
        shared.ctrl_lt1_lift_z_previous_error = state.ctrl_lt1_lift_z.previous_error();
        shared.ctrl_lt1_lift_z_first_sample = state.ctrl_lt1_lift_z.is_first_sample();
    }
    // ctrl_lt1_hold_y
    {
        const double _control_signal = state.ctrl_lt1_hold_y.control(shared.pose_ee1_base_distance_y_err_lift, shared.dt_measured_s, {shared.ctrl_lt1_hold_y_kp, shared.ctrl_lt1_hold_y_ki, shared.ctrl_lt1_hold_y_kd, shared.ctrl_lt1_hold_y_decay_rate});
        shared.eacc_pose_ee1_base_distance_y_lift = _control_signal;
        shared.ctrl_lt1_hold_y_error_integral = state.ctrl_lt1_hold_y.error_integral();
        shared.ctrl_lt1_hold_y_previous_error = state.ctrl_lt1_hold_y.previous_error();
        shared.ctrl_lt1_hold_y_first_sample = state.ctrl_lt1_hold_y.is_first_sample();
    }
    // ctrl_lt1_hold_x
    {
        const double _control_signal = state.ctrl_lt1_hold_x.control(shared.pose_ee1_base_distance_x_err_lift, shared.dt_measured_s, {shared.ctrl_lt1_hold_x_kp, shared.ctrl_lt1_hold_x_ki, shared.ctrl_lt1_hold_x_kd, shared.ctrl_lt1_hold_x_decay_rate});
        shared.eacc_pose_ee1_base_distance_x_lift = _control_signal;
        shared.ctrl_lt1_hold_x_error_integral = state.ctrl_lt1_hold_x.error_integral();
        shared.ctrl_lt1_hold_x_previous_error = state.ctrl_lt1_hold_x.previous_error();
        shared.ctrl_lt1_hold_x_first_sample = state.ctrl_lt1_hold_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_lift.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_lift_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_0(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_0.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_0 =
            alpha_frame_arm1_solver_lift_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_lift_0[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_lift_0[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_lift_0[2];
    }

    state.arm1_solver_lift.acceleration_energy(0) = shared.eacc_pose_ee1_base_distance_x_lift;

    {
        KDL::Frame alpha_frame_arm1_solver_lift_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_1(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_1.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_1 =
            alpha_frame_arm1_solver_lift_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_lift_1[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_lift_1[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_lift_1[2];
    }

    state.arm1_solver_lift.acceleration_energy(1) = shared.eacc_pose_ee1_base_distance_y_lift;

    {
        KDL::Frame alpha_frame_arm1_solver_lift_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_2(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_2.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_2 =
            alpha_frame_arm1_solver_lift_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_lift_2[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_lift_2[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_lift_2[2];
    }

    state.arm1_solver_lift.acceleration_energy(2) = shared.eacc_pose_ee1_base_distance_z_lift;

    {
        KDL::Frame alpha_frame_arm1_solver_lift_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_3(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_3.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_3 =
            alpha_frame_arm1_solver_lift_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_lift_3[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_lift_3[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_lift_3[2];
    }

    state.arm1_solver_lift.acceleration_energy(3) = shared.eacc_ctrl_lt1_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_lift_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_4(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_4.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_4 =
            alpha_frame_arm1_solver_lift_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_lift_4[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_lift_4[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_lift_4[2];
    }

    state.arm1_solver_lift.acceleration_energy(4) = shared.eacc_ctrl_lt1_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_lift_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_lift_5(*robot.arm1_solver_lift.chain);
        alpha_fk_arm1_solver_lift_5.JntToCart(
            state.arm1_solver_lift.q,
            alpha_frame_arm1_solver_lift_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_lift_5 =
            alpha_frame_arm1_solver_lift_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_lift_5[0];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_lift_5[1];
        state.arm1_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_lift_5[2];
    }

    state.arm1_solver_lift.acceleration_energy(5) = shared.eacc_ctrl_lt1_follow_ori_ang_z;

    KDL::SetToZero(state.arm1_solver_lift.tau_ff);

    for (int i = 0; i < state.arm1_solver_lift.num_segments; ++i) {
        KDL::SetToZero(state.arm1_solver_lift.f_ext[i]);
    }

    state.arm1_solver_lift.f_ext[motion_spec::runtime::find_segment_index(*robot.arm1_solver_lift.chain, "half_arm_2_link", "kinova1_base_link") - 1] += shared.wrench_force_ctrl_lt1_support_z;

    KDL::Wrenches f_ext_zero_arm1_solver_lift(state.arm1_solver_lift.num_segments);
    for (int i = 0; i < state.arm1_solver_lift.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_lift[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_lift(state.arm1_solver_lift.num_joints);
    state.arm1_solver_lift.achd_acc->CartToJnt(
        state.arm1_solver_lift.q,
        state.arm1_solver_lift.qd,
        state.arm1_solver_lift.qdd,
        state.arm1_solver_lift.spatial_directions,
        state.arm1_solver_lift.acceleration_energy,
        state.arm1_solver_lift.f_ext,
        state.arm1_solver_lift.tau_ff,
        tau_ctrl_acc_arm1_solver_lift);
    state.arm1_solver_lift.rnea->CartToJnt(
        state.arm1_solver_lift.q,
        state.arm1_solver_lift.qd,
        state.arm1_solver_lift.qdd,
        f_ext_zero_arm1_solver_lift,
        state.arm1_solver_lift.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_lift.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_lift.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_lift.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_lift.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_lift.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_lift.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_lift.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_lift.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_lift.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_lift.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_lift.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_lift.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_lift.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_lift.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_lift.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_lift.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_lift.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_lift.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_lift.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_lift.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_lift.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_lift.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_lift.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_lift.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_lift.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_lift.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_lift.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_lift.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_lift.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_lift_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_0(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_0.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_0 =
            alpha_frame_arm2_solver_lift_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_lift_0[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_lift_0[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_lift_0[2];
    }

    state.arm2_solver_lift.acceleration_energy(0) = shared.eacc_pose_ee2_base_distance_x_lift;

    {
        KDL::Frame alpha_frame_arm2_solver_lift_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_1(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_1.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_1 =
            alpha_frame_arm2_solver_lift_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_lift_1[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_lift_1[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_lift_1[2];
    }

    state.arm2_solver_lift.acceleration_energy(1) = shared.eacc_pose_ee2_base_distance_y_lift;

    {
        KDL::Frame alpha_frame_arm2_solver_lift_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_2(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_2.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_2 =
            alpha_frame_arm2_solver_lift_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_lift_2[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_lift_2[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_lift_2[2];
    }

    state.arm2_solver_lift.acceleration_energy(2) = shared.eacc_pose_ee2_base_distance_z_lift;

    {
        KDL::Frame alpha_frame_arm2_solver_lift_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_3(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_3.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_3 =
            alpha_frame_arm2_solver_lift_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_lift_3[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_lift_3[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_lift_3[2];
    }

    state.arm2_solver_lift.acceleration_energy(3) = shared.eacc_ctrl_lt2_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_lift_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_4(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_4.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_4 =
            alpha_frame_arm2_solver_lift_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_lift_4[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_lift_4[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_lift_4[2];
    }

    state.arm2_solver_lift.acceleration_energy(4) = shared.eacc_ctrl_lt2_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_lift_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_lift_5(*robot.arm2_solver_lift.chain);
        alpha_fk_arm2_solver_lift_5.JntToCart(
            state.arm2_solver_lift.q,
            alpha_frame_arm2_solver_lift_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_lift_5 =
            alpha_frame_arm2_solver_lift_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_lift_5[0];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_lift_5[1];
        state.arm2_solver_lift.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_lift_5[2];
    }

    state.arm2_solver_lift.acceleration_energy(5) = shared.eacc_ctrl_lt2_follow_ori_ang_z;

    KDL::SetToZero(state.arm2_solver_lift.tau_ff);

    for (int i = 0; i < state.arm2_solver_lift.num_segments; ++i) {
        KDL::SetToZero(state.arm2_solver_lift.f_ext[i]);
    }

    state.arm2_solver_lift.f_ext[motion_spec::runtime::find_segment_index(*robot.arm2_solver_lift.chain, "half_arm_2_link", "kinova2_base_link") - 1] += shared.wrench_force_ctrl_lt2_support_z;

    KDL::Wrenches f_ext_zero_arm2_solver_lift(state.arm2_solver_lift.num_segments);
    for (int i = 0; i < state.arm2_solver_lift.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_lift[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_lift(state.arm2_solver_lift.num_joints);
    state.arm2_solver_lift.achd_acc->CartToJnt(
        state.arm2_solver_lift.q,
        state.arm2_solver_lift.qd,
        state.arm2_solver_lift.qdd,
        state.arm2_solver_lift.spatial_directions,
        state.arm2_solver_lift.acceleration_energy,
        state.arm2_solver_lift.f_ext,
        state.arm2_solver_lift.tau_ff,
        tau_ctrl_acc_arm2_solver_lift);
    state.arm2_solver_lift.rnea->CartToJnt(
        state.arm2_solver_lift.q,
        state.arm2_solver_lift.qd,
        state.arm2_solver_lift.qdd,
        f_ext_zero_arm2_solver_lift,
        state.arm2_solver_lift.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_lift.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_lift.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_lift.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_lift.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_lift.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_lift.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_lift.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_lift.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_lift.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_lift.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_lift.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_lift.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_lift.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_lift.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_lift.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_lift.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_lift.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_lift.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_lift.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_lift.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_lift.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_lift.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_lift.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_lift.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_lift.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_lift.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_lift.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_lift.tau_ctrl(6);

}

inline void apply_motion_lift(
    motion_lift_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_lift.num_joints; ++i) {
        robot.arm1_solver_lift.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_lift.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_lift.num_joints; ++i) {
        robot.arm2_solver_lift.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_lift.tau_ctrl(i), i);
    }

    {
        const mjModel *model = robot.arm1_solver_home.robot->model;
        int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "kinova1_g_left_driver_joint");
        const int joint_id = mj_name2id(model, mjOBJ_JOINT, "kinova1_g_left_driver_joint");
        for (int i = 0; actuator_id < 0 && joint_id >= 0 && i < model->nu; ++i) {
            if (model->actuator_trntype[i] == mjTRN_JOINT
                && model->actuator_trnid[2 * i] == joint_id) actuator_id = i;
        }
        if (actuator_id >= 0) {
            robot.arm1_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_lt1_close_gripper;
        }
    }

    {
        const mjModel *model = robot.arm2_solver_home.robot->model;
        int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "kinova2_g_left_driver_joint");
        const int joint_id = mj_name2id(model, mjOBJ_JOINT, "kinova2_g_left_driver_joint");
        for (int i = 0; actuator_id < 0 && joint_id >= 0 && i < model->nu; ++i) {
            if (model->actuator_trntype[i] == mjTRN_JOINT
                && model->actuator_trnid[2 * i] == joint_id) actuator_id = i;
        }
        if (actuator_id >= 0) {
            robot.arm2_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_lt2_close_gripper;
        }
    }

}
