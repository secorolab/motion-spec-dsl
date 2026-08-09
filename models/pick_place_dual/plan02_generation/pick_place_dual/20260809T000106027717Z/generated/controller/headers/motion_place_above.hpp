/// Motion: place-above
/// Move both TCPs laterally to the pre-place position above their place locations
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_place_above_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_place_above_solver_state arm1_solver_place_above;
    arm2_solver_place_above_solver_state arm2_solver_place_above;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pla1_reach_x;
    motion_spec::runtime::PIDControl ctrl_pla1_reach_y;
    motion_spec::runtime::PIDControl ctrl_pla1_reach_z;
    motion_spec::runtime::PIDControl ctrl_pla1_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pla1_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pla1_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pla1_support_z;

    motion_spec::runtime::PIDControl ctrl_pla2_reach_x;
    motion_spec::runtime::PIDControl ctrl_pla2_reach_y;
    motion_spec::runtime::PIDControl ctrl_pla2_reach_z;
    motion_spec::runtime::PIDControl ctrl_pla2_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pla2_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pla2_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_pla2_support_z;

    bool mon_place_above_ready_previous = false;
    bool mon_place_above_ready_event_triggered = false;

    bool mon_place_above_grasp_lost_previous = false;
    bool mon_place_above_grasp_lost_event_triggered = false;

};

inline void reset_motion_place_above(motion_place_above_state &state) {
    state = motion_place_above_state{};
}

inline void init_motion_place_above(motion_place_above_state &state, const robot_io &robot) {
    if (!state.arm1_solver_place_above.initialized) {
        state.arm1_solver_place_above.num_joints = robot.arm1_solver_place_above.chain->getNrOfJoints();
        state.arm1_solver_place_above.num_segments = robot.arm1_solver_place_above.chain->getNrOfSegments();
        state.arm1_solver_place_above.q = KDL::JntArray(state.arm1_solver_place_above.num_joints);
        state.arm1_solver_place_above.qd = KDL::JntArray(state.arm1_solver_place_above.num_joints);
        state.arm1_solver_place_above.qdd = KDL::JntArray(state.arm1_solver_place_above.num_joints);
        state.arm1_solver_place_above.tau_ff = KDL::JntArray(state.arm1_solver_place_above.num_joints);
        state.arm1_solver_place_above.tau_ctrl = KDL::JntArray(state.arm1_solver_place_above.num_joints);
        state.arm1_solver_place_above.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_place_above.num_spatial_directions = 6;
        state.arm1_solver_place_above.spatial_directions = KDL::Jacobian(state.arm1_solver_place_above.num_spatial_directions);
        state.arm1_solver_place_above.acceleration_energy = KDL::JntArray(state.arm1_solver_place_above.num_spatial_directions);
        state.arm1_solver_place_above.f_ext = KDL::Wrenches(state.arm1_solver_place_above.num_segments);
        state.arm1_solver_place_above.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm1_solver_place_above.chain, state.arm1_solver_place_above.root_acc, state.arm1_solver_place_above.num_spatial_directions);
        state.arm1_solver_place_above.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_place_above.chain, state.arm1_solver_place_above.root_acc, state.arm1_solver_place_above.num_spatial_directions);
        state.arm1_solver_place_above.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_place_above.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_place_above.initialized = true;
    }
    if (!state.arm2_solver_place_above.initialized) {
        state.arm2_solver_place_above.num_joints = robot.arm2_solver_place_above.chain->getNrOfJoints();
        state.arm2_solver_place_above.num_segments = robot.arm2_solver_place_above.chain->getNrOfSegments();
        state.arm2_solver_place_above.q = KDL::JntArray(state.arm2_solver_place_above.num_joints);
        state.arm2_solver_place_above.qd = KDL::JntArray(state.arm2_solver_place_above.num_joints);
        state.arm2_solver_place_above.qdd = KDL::JntArray(state.arm2_solver_place_above.num_joints);
        state.arm2_solver_place_above.tau_ff = KDL::JntArray(state.arm2_solver_place_above.num_joints);
        state.arm2_solver_place_above.tau_ctrl = KDL::JntArray(state.arm2_solver_place_above.num_joints);
        state.arm2_solver_place_above.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_place_above.num_spatial_directions = 6;
        state.arm2_solver_place_above.spatial_directions = KDL::Jacobian(state.arm2_solver_place_above.num_spatial_directions);
        state.arm2_solver_place_above.acceleration_energy = KDL::JntArray(state.arm2_solver_place_above.num_spatial_directions);
        state.arm2_solver_place_above.f_ext = KDL::Wrenches(state.arm2_solver_place_above.num_segments);
        state.arm2_solver_place_above.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm2_solver_place_above.chain, state.arm2_solver_place_above.root_acc, state.arm2_solver_place_above.num_spatial_directions);
        state.arm2_solver_place_above.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_place_above.chain, state.arm2_solver_place_above.root_acc, state.arm2_solver_place_above.num_spatial_directions);
        state.arm2_solver_place_above.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_place_above.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_place_above.initialized = true;
    }
}

inline void update_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_place_above(state, robot);

    mj_kdl::update(robot.arm1_solver_place_above.robot);
    for (int i = 0; i < state.arm1_solver_place_above.num_joints; ++i) {
        state.arm1_solver_place_above.q(i) = robot.arm1_solver_place_above.robot->jnt_pos_msr[i];
        state.arm1_solver_place_above.qd(i) = robot.arm1_solver_place_above.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_place_above(state.arm1_solver_place_above.q, state.arm1_solver_place_above.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_place_above.robot->model,
                robot.arm1_solver_place_above.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_place_above;
        mj_kdl::get_body_frame(
                robot.arm1_solver_place_above.robot->model,
                robot.arm1_solver_place_above.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_place_above);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_place_above.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_place_above.chain);
        fk.JntToCart(
            state.arm1_solver_place_above.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_place_above.chain);
        fk.JntToCart(
            state.arm1_solver_place_above.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_place_above.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_place_above,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_place_above.robot->model,
                robot.arm1_solver_place_above.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_place_above.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_place_above.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_place_above.robot);
    for (int i = 0; i < state.arm2_solver_place_above.num_joints; ++i) {
        state.arm2_solver_place_above.q(i) = robot.arm2_solver_place_above.robot->jnt_pos_msr[i];
        state.arm2_solver_place_above.qd(i) = robot.arm2_solver_place_above.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_place_above(state.arm2_solver_place_above.q, state.arm2_solver_place_above.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_place_above.robot->model,
                robot.arm2_solver_place_above.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_place_above;
        mj_kdl::get_body_frame(
                robot.arm2_solver_place_above.robot->model,
                robot.arm2_solver_place_above.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_place_above);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_place_above.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_place_above.chain);
        fk.JntToCart(
            state.arm2_solver_place_above.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_place_above.chain);
        fk.JntToCart(
            state.arm2_solver_place_above.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_place_above.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_place_above,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_place_above.robot->model,
                robot.arm2_solver_place_above.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_place_above.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_place_above.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.place_above_support1_z_add_out = shared.pose_elbow1_base.p[2] + shared.place_above_support_lift;
        shared.place_above_support1_z = shared.place_above_support1_z_add_out;

        shared.place_above_support2_z_add_out = shared.pose_elbow2_base.p[2] + shared.place_above_support_lift;
        shared.place_above_support2_z = shared.place_above_support2_z_add_out;
        state.snapshot_taken = true;
    }
    shared.goal1_pose_above = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    shared.goal2_pose_above = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
}

inline bool can_start_motion_place_above(
    shared_data &shared
) {
    // eval_place_above_when_lifted1
    shared.eval_place_above_when_lifted1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee1_base.p[2], shared.lifted_z);
    // eval_place_above_when_lifted2
    shared.eval_place_above_when_lifted2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee2_base.p[2], shared.lifted_z);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted1_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted2_err, shared.default_tolerance_Distance));
}

inline void monitor_when_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.goal1_pose_above = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    shared.goal2_pose_above = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    // eval_place_above_when_lifted1
    shared.eval_place_above_when_lifted1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee1_base.p[2], shared.lifted_z);
    // eval_place_above_when_lifted2
    shared.eval_place_above_when_lifted2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee2_base.p[2], shared.lifted_z);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted1_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_above_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(8);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PLACE_ABOVE_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PLACE_ABOVE_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_place_above(
    motion_place_above_state &state,
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
    // eval_place_above_until_grasp_lost1
    shared.eval_place_above_until_grasp_lost1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost1_distance, shared.lost_dist);
    // eval_place_above_until_grasp_lost2
    shared.eval_place_above_until_grasp_lost2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost2_distance, shared.lost_dist);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_place_above_until_grasp_lost1_err, shared.default_tolerance_Distance) || motion_spec::runtime::constraint_satisfied(shared.eval_place_above_until_grasp_lost2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_above_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(2);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_GRASP_LOST_PLACE_ABOVE);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_GRASP_LOST_PLACE_ABOVE] << std::endl;
        }
    }

}

inline void monitor_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_place_above_when_lifted1
    shared.eval_place_above_when_lifted1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee1_base.p[2], shared.lifted_z);
    // eval_place_above_when_lifted2
    shared.eval_place_above_when_lifted2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_ee2_base.p[2], shared.lifted_z);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted1_err, shared.default_tolerance_Distance) && motion_spec::runtime::constraint_satisfied(shared.eval_place_above_when_lifted2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_above_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(8);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PLACE_ABOVE_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PLACE_ABOVE_READY] << std::endl;
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
    // eval_place_above_until_grasp_lost1
    shared.eval_place_above_until_grasp_lost1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost1_distance, shared.lost_dist);
    // eval_place_above_until_grasp_lost2
    shared.eval_place_above_until_grasp_lost2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.distance_grasp_lost2_distance, shared.lost_dist);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_place_above_until_grasp_lost1_err, shared.default_tolerance_Distance) || motion_spec::runtime::constraint_satisfied(shared.eval_place_above_until_grasp_lost2_err, shared.default_tolerance_Distance));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_place_above_grasp_lost_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(2);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_GRASP_LOST_PLACE_ABOVE);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_GRASP_LOST_PLACE_ABOVE] << std::endl;
        }
    }

}

inline void control_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee1_base = shared.pose_ee1_base;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[0] = shared.place_above_place_x;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[1] = shared.place_above_place_y;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[2] = shared.place_above_place_above_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee1_base = KDL::diff(shared.pose_ee1_base, _pose_axis_target_pose_axis_error_pose_ee1_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.z();

        shared.pose_ee1_base_distance_x_err_place_above = _pose_axis_error_linear_X;
        shared.pose_ee1_base_distance_y_err_place_above = _pose_axis_error_linear_Y;
        shared.pose_ee1_base_distance_z_err_place_above = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee2_base = shared.pose_ee2_base;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[0] = shared.place_above_place_x;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[1] = shared.place_above_place_y;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[2] = shared.place_above_place_above_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee2_base = KDL::diff(shared.pose_ee2_base, _pose_axis_target_pose_axis_error_pose_ee2_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.z();

        shared.pose_ee2_base_distance_x_err_place_above = _pose_axis_error_linear_X;
        shared.pose_ee2_base_distance_y_err_place_above = _pose_axis_error_linear_Y;
        shared.pose_ee2_base_distance_z_err_place_above = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_pla1_align_ori
    shared.pose_diff_ctrl_pla1_align_ori = KDL::diff(shared.pose_ee1_base, shared.goal1_pose_above);
    shared.ctrl_pla1_align_ori_err_ang_x = shared.pose_diff_ctrl_pla1_align_ori.rot[0];
    shared.ctrl_pla1_align_ori_err_ang_y = shared.pose_diff_ctrl_pla1_align_ori.rot[1];
    shared.ctrl_pla1_align_ori_err_ang_z = shared.pose_diff_ctrl_pla1_align_ori.rot[2];
    // eval_pose_diff_ctrl_pla2_align_ori
    shared.pose_diff_ctrl_pla2_align_ori = KDL::diff(shared.pose_ee2_base, shared.goal2_pose_above);
    shared.ctrl_pla2_align_ori_err_ang_x = shared.pose_diff_ctrl_pla2_align_ori.rot[0];
    shared.ctrl_pla2_align_ori_err_ang_y = shared.pose_diff_ctrl_pla2_align_ori.rot[1];
    shared.ctrl_pla2_align_ori_err_ang_z = shared.pose_diff_ctrl_pla2_align_ori.rot[2];
    // eval_place_above_while_support1_elbow_z
    shared.pose_elbow1_base_distance_z_err_place_above = motion_spec::runtime::evaluate_equality_constraint(shared.place_above_support1_z, shared.pose_elbow1_base.p[2]);
    // eval_place_above_while_support2_elbow_z
    shared.pose_elbow2_base_distance_z_err_place_above = motion_spec::runtime::evaluate_equality_constraint(shared.place_above_support2_z, shared.pose_elbow2_base.p[2]);
    // compute_wrench_force_ctrl_pla1_support_z
    shared.wrench_force_ctrl_pla1_support_z = KDL::Wrench(shared.direction_ctrl_pla1_support_z * shared.force_ctrl_pla1_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pla1_support_z);
    // compute_wrench_force_ctrl_pla2_support_z
    shared.wrench_force_ctrl_pla2_support_z = KDL::Wrench(shared.direction_ctrl_pla2_support_z * shared.force_ctrl_pla2_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pla2_support_z);
    // eval_place_above_while_close1_gripper
    shared.gripper1_pos_err_place_above = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper1_pos);
    // eval_place_above_while_close2_gripper
    shared.gripper2_pos_err_place_above = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_closed, shared.gripper2_pos);
    // ctrl_pla2_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_pla2_close_gripper = _control_signal;
    }
    // ctrl_pla2_support_z
    {
        const double _control_signal = state.ctrl_pla2_support_z.control(shared.pose_elbow2_base_distance_z_err_place_above, shared.dt_measured_s, {shared.ctrl_pla2_support_z_stiffness, shared.ctrl_pla2_support_z_damping, shared.ctrl_pla2_support_z_integral_gain});
        shared.force_ctrl_pla2_support_z = _control_signal;
        shared.ctrl_pla2_support_z_error_integral = state.ctrl_pla2_support_z.error_integral();
        shared.ctrl_pla2_support_z_previous_error = state.ctrl_pla2_support_z.previous_error();
        shared.ctrl_pla2_support_z_first_sample = state.ctrl_pla2_support_z.is_first_sample();
    }
    // ctrl_pla2_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_pla2_align_ori_ang_z.control(shared.pose_diff_ctrl_pla2_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pla2_align_ori_ang_z_kp, shared.ctrl_pla2_align_ori_ang_z_ki, shared.ctrl_pla2_align_ori_ang_z_kd, shared.ctrl_pla2_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pla2_align_ori_ang_z = _control_signal;
        shared.ctrl_pla2_align_ori_ang_z_error_integral = state.ctrl_pla2_align_ori_ang_z.error_integral();
        shared.ctrl_pla2_align_ori_ang_z_previous_error = state.ctrl_pla2_align_ori_ang_z.previous_error();
        shared.ctrl_pla2_align_ori_ang_z_first_sample = state.ctrl_pla2_align_ori_ang_z.is_first_sample();
    }
    // ctrl_pla2_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_pla2_align_ori_ang_y.control(shared.pose_diff_ctrl_pla2_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pla2_align_ori_ang_y_kp, shared.ctrl_pla2_align_ori_ang_y_ki, shared.ctrl_pla2_align_ori_ang_y_kd, shared.ctrl_pla2_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pla2_align_ori_ang_y = _control_signal;
        shared.ctrl_pla2_align_ori_ang_y_error_integral = state.ctrl_pla2_align_ori_ang_y.error_integral();
        shared.ctrl_pla2_align_ori_ang_y_previous_error = state.ctrl_pla2_align_ori_ang_y.previous_error();
        shared.ctrl_pla2_align_ori_ang_y_first_sample = state.ctrl_pla2_align_ori_ang_y.is_first_sample();
    }
    // ctrl_pla2_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_pla2_align_ori_ang_x.control(shared.pose_diff_ctrl_pla2_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pla2_align_ori_ang_x_kp, shared.ctrl_pla2_align_ori_ang_x_ki, shared.ctrl_pla2_align_ori_ang_x_kd, shared.ctrl_pla2_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pla2_align_ori_ang_x = _control_signal;
        shared.ctrl_pla2_align_ori_ang_x_error_integral = state.ctrl_pla2_align_ori_ang_x.error_integral();
        shared.ctrl_pla2_align_ori_ang_x_previous_error = state.ctrl_pla2_align_ori_ang_x.previous_error();
        shared.ctrl_pla2_align_ori_ang_x_first_sample = state.ctrl_pla2_align_ori_ang_x.is_first_sample();
    }
    // ctrl_pla2_reach_z
    {
        const double _control_signal = state.ctrl_pla2_reach_z.control(shared.pose_ee2_base_distance_z_err_place_above, shared.dt_measured_s, {shared.ctrl_pla2_reach_z_kp, shared.ctrl_pla2_reach_z_ki, shared.ctrl_pla2_reach_z_kd, shared.ctrl_pla2_reach_z_decay_rate});
        shared.eacc_pose_ee2_base_distance_z_place_above = _control_signal;
        shared.ctrl_pla2_reach_z_error_integral = state.ctrl_pla2_reach_z.error_integral();
        shared.ctrl_pla2_reach_z_previous_error = state.ctrl_pla2_reach_z.previous_error();
        shared.ctrl_pla2_reach_z_first_sample = state.ctrl_pla2_reach_z.is_first_sample();
    }
    // ctrl_pla2_reach_y
    {
        const double _control_signal = state.ctrl_pla2_reach_y.control(shared.pose_ee2_base_distance_y_err_place_above, shared.dt_measured_s, {shared.ctrl_pla2_reach_y_kp, shared.ctrl_pla2_reach_y_ki, shared.ctrl_pla2_reach_y_kd, shared.ctrl_pla2_reach_y_decay_rate});
        shared.eacc_pose_ee2_base_distance_y_place_above = _control_signal;
        shared.ctrl_pla2_reach_y_error_integral = state.ctrl_pla2_reach_y.error_integral();
        shared.ctrl_pla2_reach_y_previous_error = state.ctrl_pla2_reach_y.previous_error();
        shared.ctrl_pla2_reach_y_first_sample = state.ctrl_pla2_reach_y.is_first_sample();
    }
    // ctrl_pla2_reach_x
    {
        const double _control_signal = state.ctrl_pla2_reach_x.control(shared.pose_ee2_base_distance_x_err_place_above, shared.dt_measured_s, {shared.ctrl_pla2_reach_x_kp, shared.ctrl_pla2_reach_x_ki, shared.ctrl_pla2_reach_x_kd, shared.ctrl_pla2_reach_x_decay_rate});
        shared.eacc_pose_ee2_base_distance_x_place_above = _control_signal;
        shared.ctrl_pla2_reach_x_error_integral = state.ctrl_pla2_reach_x.error_integral();
        shared.ctrl_pla2_reach_x_previous_error = state.ctrl_pla2_reach_x.previous_error();
        shared.ctrl_pla2_reach_x_first_sample = state.ctrl_pla2_reach_x.is_first_sample();
    }
    // ctrl_pla1_close_gripper
    {
        const double _control_signal = shared.gripper_closed;
        shared.cmd_ctrl_pla1_close_gripper = _control_signal;
    }
    // ctrl_pla1_support_z
    {
        const double _control_signal = state.ctrl_pla1_support_z.control(shared.pose_elbow1_base_distance_z_err_place_above, shared.dt_measured_s, {shared.ctrl_pla1_support_z_stiffness, shared.ctrl_pla1_support_z_damping, shared.ctrl_pla1_support_z_integral_gain});
        shared.force_ctrl_pla1_support_z = _control_signal;
        shared.ctrl_pla1_support_z_error_integral = state.ctrl_pla1_support_z.error_integral();
        shared.ctrl_pla1_support_z_previous_error = state.ctrl_pla1_support_z.previous_error();
        shared.ctrl_pla1_support_z_first_sample = state.ctrl_pla1_support_z.is_first_sample();
    }
    // ctrl_pla1_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_pla1_align_ori_ang_z.control(shared.pose_diff_ctrl_pla1_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_pla1_align_ori_ang_z_kp, shared.ctrl_pla1_align_ori_ang_z_ki, shared.ctrl_pla1_align_ori_ang_z_kd, shared.ctrl_pla1_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pla1_align_ori_ang_z = _control_signal;
        shared.ctrl_pla1_align_ori_ang_z_error_integral = state.ctrl_pla1_align_ori_ang_z.error_integral();
        shared.ctrl_pla1_align_ori_ang_z_previous_error = state.ctrl_pla1_align_ori_ang_z.previous_error();
        shared.ctrl_pla1_align_ori_ang_z_first_sample = state.ctrl_pla1_align_ori_ang_z.is_first_sample();
    }
    // ctrl_pla1_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_pla1_align_ori_ang_y.control(shared.pose_diff_ctrl_pla1_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_pla1_align_ori_ang_y_kp, shared.ctrl_pla1_align_ori_ang_y_ki, shared.ctrl_pla1_align_ori_ang_y_kd, shared.ctrl_pla1_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pla1_align_ori_ang_y = _control_signal;
        shared.ctrl_pla1_align_ori_ang_y_error_integral = state.ctrl_pla1_align_ori_ang_y.error_integral();
        shared.ctrl_pla1_align_ori_ang_y_previous_error = state.ctrl_pla1_align_ori_ang_y.previous_error();
        shared.ctrl_pla1_align_ori_ang_y_first_sample = state.ctrl_pla1_align_ori_ang_y.is_first_sample();
    }
    // ctrl_pla1_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_pla1_align_ori_ang_x.control(shared.pose_diff_ctrl_pla1_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_pla1_align_ori_ang_x_kp, shared.ctrl_pla1_align_ori_ang_x_ki, shared.ctrl_pla1_align_ori_ang_x_kd, shared.ctrl_pla1_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pla1_align_ori_ang_x = _control_signal;
        shared.ctrl_pla1_align_ori_ang_x_error_integral = state.ctrl_pla1_align_ori_ang_x.error_integral();
        shared.ctrl_pla1_align_ori_ang_x_previous_error = state.ctrl_pla1_align_ori_ang_x.previous_error();
        shared.ctrl_pla1_align_ori_ang_x_first_sample = state.ctrl_pla1_align_ori_ang_x.is_first_sample();
    }
    // ctrl_pla1_reach_z
    {
        const double _control_signal = state.ctrl_pla1_reach_z.control(shared.pose_ee1_base_distance_z_err_place_above, shared.dt_measured_s, {shared.ctrl_pla1_reach_z_kp, shared.ctrl_pla1_reach_z_ki, shared.ctrl_pla1_reach_z_kd, shared.ctrl_pla1_reach_z_decay_rate});
        shared.eacc_pose_ee1_base_distance_z_place_above = _control_signal;
        shared.ctrl_pla1_reach_z_error_integral = state.ctrl_pla1_reach_z.error_integral();
        shared.ctrl_pla1_reach_z_previous_error = state.ctrl_pla1_reach_z.previous_error();
        shared.ctrl_pla1_reach_z_first_sample = state.ctrl_pla1_reach_z.is_first_sample();
    }
    // ctrl_pla1_reach_y
    {
        const double _control_signal = state.ctrl_pla1_reach_y.control(shared.pose_ee1_base_distance_y_err_place_above, shared.dt_measured_s, {shared.ctrl_pla1_reach_y_kp, shared.ctrl_pla1_reach_y_ki, shared.ctrl_pla1_reach_y_kd, shared.ctrl_pla1_reach_y_decay_rate});
        shared.eacc_pose_ee1_base_distance_y_place_above = _control_signal;
        shared.ctrl_pla1_reach_y_error_integral = state.ctrl_pla1_reach_y.error_integral();
        shared.ctrl_pla1_reach_y_previous_error = state.ctrl_pla1_reach_y.previous_error();
        shared.ctrl_pla1_reach_y_first_sample = state.ctrl_pla1_reach_y.is_first_sample();
    }
    // ctrl_pla1_reach_x
    {
        const double _control_signal = state.ctrl_pla1_reach_x.control(shared.pose_ee1_base_distance_x_err_place_above, shared.dt_measured_s, {shared.ctrl_pla1_reach_x_kp, shared.ctrl_pla1_reach_x_ki, shared.ctrl_pla1_reach_x_kd, shared.ctrl_pla1_reach_x_decay_rate});
        shared.eacc_pose_ee1_base_distance_x_place_above = _control_signal;
        shared.ctrl_pla1_reach_x_error_integral = state.ctrl_pla1_reach_x.error_integral();
        shared.ctrl_pla1_reach_x_previous_error = state.ctrl_pla1_reach_x.previous_error();
        shared.ctrl_pla1_reach_x_first_sample = state.ctrl_pla1_reach_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_place_above.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_0(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_0.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_0 =
            alpha_frame_arm1_solver_place_above_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_place_above_0[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_place_above_0[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_place_above_0[2];
    }

    state.arm1_solver_place_above.acceleration_energy(0) = shared.eacc_pose_ee1_base_distance_x_place_above;

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_1(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_1.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_1 =
            alpha_frame_arm1_solver_place_above_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_place_above_1[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_place_above_1[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_place_above_1[2];
    }

    state.arm1_solver_place_above.acceleration_energy(1) = shared.eacc_pose_ee1_base_distance_y_place_above;

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_2(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_2.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_2 =
            alpha_frame_arm1_solver_place_above_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_place_above_2[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_place_above_2[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_place_above_2[2];
    }

    state.arm1_solver_place_above.acceleration_energy(2) = shared.eacc_pose_ee1_base_distance_z_place_above;

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_3(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_3.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_3 =
            alpha_frame_arm1_solver_place_above_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_place_above_3[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_place_above_3[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_place_above_3[2];
    }

    state.arm1_solver_place_above.acceleration_energy(3) = shared.eacc_ctrl_pla1_align_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_4(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_4.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_4 =
            alpha_frame_arm1_solver_place_above_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_place_above_4[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_place_above_4[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_place_above_4[2];
    }

    state.arm1_solver_place_above.acceleration_energy(4) = shared.eacc_ctrl_pla1_align_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_place_above_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_place_above_5(*robot.arm1_solver_place_above.chain);
        alpha_fk_arm1_solver_place_above_5.JntToCart(
            state.arm1_solver_place_above.q,
            alpha_frame_arm1_solver_place_above_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_place_above_5 =
            alpha_frame_arm1_solver_place_above_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_place_above_5[0];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_place_above_5[1];
        state.arm1_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_place_above_5[2];
    }

    state.arm1_solver_place_above.acceleration_energy(5) = shared.eacc_ctrl_pla1_align_ori_ang_z;

    KDL::SetToZero(state.arm1_solver_place_above.tau_ff);

    for (int i = 0; i < state.arm1_solver_place_above.num_segments; ++i) {
        KDL::SetToZero(state.arm1_solver_place_above.f_ext[i]);
    }

    state.arm1_solver_place_above.f_ext[motion_spec::runtime::find_segment_index(*robot.arm1_solver_place_above.chain, "half_arm_2_link", "kinova1_base_link") - 1] += shared.wrench_force_ctrl_pla1_support_z;

    KDL::Wrenches f_ext_zero_arm1_solver_place_above(state.arm1_solver_place_above.num_segments);
    for (int i = 0; i < state.arm1_solver_place_above.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_place_above[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_place_above(state.arm1_solver_place_above.num_joints);
    state.arm1_solver_place_above.achd_acc->CartToJnt(
        state.arm1_solver_place_above.q,
        state.arm1_solver_place_above.qd,
        state.arm1_solver_place_above.qdd,
        state.arm1_solver_place_above.spatial_directions,
        state.arm1_solver_place_above.acceleration_energy,
        state.arm1_solver_place_above.f_ext,
        state.arm1_solver_place_above.tau_ff,
        tau_ctrl_acc_arm1_solver_place_above);
    state.arm1_solver_place_above.rnea->CartToJnt(
        state.arm1_solver_place_above.q,
        state.arm1_solver_place_above.qd,
        state.arm1_solver_place_above.qdd,
        f_ext_zero_arm1_solver_place_above,
        state.arm1_solver_place_above.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_place_above.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_place_above.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_place_above.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_place_above.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_place_above.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_place_above.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_place_above.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_place_above.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_place_above.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_place_above.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_place_above.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_place_above.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_place_above.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_place_above.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_place_above.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_place_above.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_place_above.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_place_above.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_place_above.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_place_above.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_place_above.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_place_above.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_place_above.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_place_above.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_place_above.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_place_above.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_place_above.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_place_above.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_place_above.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_0(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_0.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_0 =
            alpha_frame_arm2_solver_place_above_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_place_above_0[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_place_above_0[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_place_above_0[2];
    }

    state.arm2_solver_place_above.acceleration_energy(0) = shared.eacc_pose_ee2_base_distance_x_place_above;

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_1(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_1.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_1 =
            alpha_frame_arm2_solver_place_above_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_place_above_1[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_place_above_1[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_place_above_1[2];
    }

    state.arm2_solver_place_above.acceleration_energy(1) = shared.eacc_pose_ee2_base_distance_y_place_above;

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_2(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_2.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_2 =
            alpha_frame_arm2_solver_place_above_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_place_above_2[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_place_above_2[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_place_above_2[2];
    }

    state.arm2_solver_place_above.acceleration_energy(2) = shared.eacc_pose_ee2_base_distance_z_place_above;

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_3(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_3.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_3 =
            alpha_frame_arm2_solver_place_above_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_place_above_3[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_place_above_3[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_place_above_3[2];
    }

    state.arm2_solver_place_above.acceleration_energy(3) = shared.eacc_ctrl_pla2_align_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_4(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_4.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_4 =
            alpha_frame_arm2_solver_place_above_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_place_above_4[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_place_above_4[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_place_above_4[2];
    }

    state.arm2_solver_place_above.acceleration_energy(4) = shared.eacc_ctrl_pla2_align_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_place_above_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_place_above_5(*robot.arm2_solver_place_above.chain);
        alpha_fk_arm2_solver_place_above_5.JntToCart(
            state.arm2_solver_place_above.q,
            alpha_frame_arm2_solver_place_above_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_place_above_5 =
            alpha_frame_arm2_solver_place_above_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_place_above_5[0];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_place_above_5[1];
        state.arm2_solver_place_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_place_above_5[2];
    }

    state.arm2_solver_place_above.acceleration_energy(5) = shared.eacc_ctrl_pla2_align_ori_ang_z;

    KDL::SetToZero(state.arm2_solver_place_above.tau_ff);

    for (int i = 0; i < state.arm2_solver_place_above.num_segments; ++i) {
        KDL::SetToZero(state.arm2_solver_place_above.f_ext[i]);
    }

    state.arm2_solver_place_above.f_ext[motion_spec::runtime::find_segment_index(*robot.arm2_solver_place_above.chain, "half_arm_2_link", "kinova2_base_link") - 1] += shared.wrench_force_ctrl_pla2_support_z;

    KDL::Wrenches f_ext_zero_arm2_solver_place_above(state.arm2_solver_place_above.num_segments);
    for (int i = 0; i < state.arm2_solver_place_above.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_place_above[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_place_above(state.arm2_solver_place_above.num_joints);
    state.arm2_solver_place_above.achd_acc->CartToJnt(
        state.arm2_solver_place_above.q,
        state.arm2_solver_place_above.qd,
        state.arm2_solver_place_above.qdd,
        state.arm2_solver_place_above.spatial_directions,
        state.arm2_solver_place_above.acceleration_energy,
        state.arm2_solver_place_above.f_ext,
        state.arm2_solver_place_above.tau_ff,
        tau_ctrl_acc_arm2_solver_place_above);
    state.arm2_solver_place_above.rnea->CartToJnt(
        state.arm2_solver_place_above.q,
        state.arm2_solver_place_above.qd,
        state.arm2_solver_place_above.qdd,
        f_ext_zero_arm2_solver_place_above,
        state.arm2_solver_place_above.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_place_above.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_place_above.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_place_above.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_place_above.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_place_above.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_place_above.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_place_above.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_place_above.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_place_above.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_place_above.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_place_above.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_place_above.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_place_above.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_place_above.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_place_above.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_place_above.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_place_above.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_place_above.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_place_above.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_place_above.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_place_above.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_place_above.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_place_above.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_place_above.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_place_above.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_place_above.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_place_above.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_place_above.tau_ctrl(6);

}

inline void apply_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_place_above.num_joints; ++i) {
        robot.arm1_solver_place_above.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_place_above.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_place_above.num_joints; ++i) {
        robot.arm2_solver_place_above.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_place_above.tau_ctrl(i), i);
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
            robot.arm1_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_pla1_close_gripper;
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
            robot.arm2_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_pla2_close_gripper;
        }
    }

}
