/// Motion: retreat
/// Move both TCPs straight up to the pre-grasp height above the place locations
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_retreat_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_retreat_solver_state arm1_solver_retreat;
    arm2_solver_retreat_solver_state arm2_solver_retreat;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_ret1_reach_x;
    motion_spec::runtime::PIDControl ctrl_ret1_reach_y;
    motion_spec::runtime::PIDControl ctrl_ret1_reach_z;
    motion_spec::runtime::PIDControl ctrl_ret1_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_ret1_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_ret1_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_ret1_support_z;

    motion_spec::runtime::PIDControl ctrl_ret2_reach_x;
    motion_spec::runtime::PIDControl ctrl_ret2_reach_y;
    motion_spec::runtime::PIDControl ctrl_ret2_reach_z;
    motion_spec::runtime::PIDControl ctrl_ret2_align_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_ret2_align_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_ret2_align_ori_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_ret2_support_z;

    bool mon_retreat_ready_previous = false;
    bool mon_retreat_ready_event_triggered = false;

    bool mon_retreat_settled_previous = false;
    bool mon_retreat_settled_event_triggered = false;

};

inline void reset_motion_retreat(motion_retreat_state &state) {
    state = motion_retreat_state{};
}

inline void init_motion_retreat(motion_retreat_state &state, const robot_io &robot) {
    if (!state.arm1_solver_retreat.initialized) {
        state.arm1_solver_retreat.num_joints = robot.arm1_solver_retreat.chain->getNrOfJoints();
        state.arm1_solver_retreat.num_segments = robot.arm1_solver_retreat.chain->getNrOfSegments();
        state.arm1_solver_retreat.q = KDL::JntArray(state.arm1_solver_retreat.num_joints);
        state.arm1_solver_retreat.qd = KDL::JntArray(state.arm1_solver_retreat.num_joints);
        state.arm1_solver_retreat.qdd = KDL::JntArray(state.arm1_solver_retreat.num_joints);
        state.arm1_solver_retreat.tau_ff = KDL::JntArray(state.arm1_solver_retreat.num_joints);
        state.arm1_solver_retreat.tau_ctrl = KDL::JntArray(state.arm1_solver_retreat.num_joints);
        state.arm1_solver_retreat.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_retreat.num_spatial_directions = 6;
        state.arm1_solver_retreat.spatial_directions = KDL::Jacobian(state.arm1_solver_retreat.num_spatial_directions);
        state.arm1_solver_retreat.acceleration_energy = KDL::JntArray(state.arm1_solver_retreat.num_spatial_directions);
        state.arm1_solver_retreat.f_ext = KDL::Wrenches(state.arm1_solver_retreat.num_segments);
        state.arm1_solver_retreat.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm1_solver_retreat.chain, state.arm1_solver_retreat.root_acc, state.arm1_solver_retreat.num_spatial_directions);
        state.arm1_solver_retreat.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_retreat.chain, state.arm1_solver_retreat.root_acc, state.arm1_solver_retreat.num_spatial_directions);
        state.arm1_solver_retreat.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_retreat.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_retreat.initialized = true;
    }
    if (!state.arm2_solver_retreat.initialized) {
        state.arm2_solver_retreat.num_joints = robot.arm2_solver_retreat.chain->getNrOfJoints();
        state.arm2_solver_retreat.num_segments = robot.arm2_solver_retreat.chain->getNrOfSegments();
        state.arm2_solver_retreat.q = KDL::JntArray(state.arm2_solver_retreat.num_joints);
        state.arm2_solver_retreat.qd = KDL::JntArray(state.arm2_solver_retreat.num_joints);
        state.arm2_solver_retreat.qdd = KDL::JntArray(state.arm2_solver_retreat.num_joints);
        state.arm2_solver_retreat.tau_ff = KDL::JntArray(state.arm2_solver_retreat.num_joints);
        state.arm2_solver_retreat.tau_ctrl = KDL::JntArray(state.arm2_solver_retreat.num_joints);
        state.arm2_solver_retreat.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_retreat.num_spatial_directions = 6;
        state.arm2_solver_retreat.spatial_directions = KDL::Jacobian(state.arm2_solver_retreat.num_spatial_directions);
        state.arm2_solver_retreat.acceleration_energy = KDL::JntArray(state.arm2_solver_retreat.num_spatial_directions);
        state.arm2_solver_retreat.f_ext = KDL::Wrenches(state.arm2_solver_retreat.num_segments);
        state.arm2_solver_retreat.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm2_solver_retreat.chain, state.arm2_solver_retreat.root_acc, state.arm2_solver_retreat.num_spatial_directions);
        state.arm2_solver_retreat.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_retreat.chain, state.arm2_solver_retreat.root_acc, state.arm2_solver_retreat.num_spatial_directions);
        state.arm2_solver_retreat.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_retreat.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_retreat.initialized = true;
    }
}

inline void update_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_retreat(state, robot);

    mj_kdl::update(robot.arm1_solver_retreat.robot);
    for (int i = 0; i < state.arm1_solver_retreat.num_joints; ++i) {
        state.arm1_solver_retreat.q(i) = robot.arm1_solver_retreat.robot->jnt_pos_msr[i];
        state.arm1_solver_retreat.qd(i) = robot.arm1_solver_retreat.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_retreat(state.arm1_solver_retreat.q, state.arm1_solver_retreat.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_retreat.robot->model,
                robot.arm1_solver_retreat.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_retreat;
        mj_kdl::get_body_frame(
                robot.arm1_solver_retreat.robot->model,
                robot.arm1_solver_retreat.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_retreat);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_retreat.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_retreat.chain);
        fk.JntToCart(
            state.arm1_solver_retreat.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_retreat.chain);
        fk.JntToCart(
            state.arm1_solver_retreat.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_retreat.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_retreat,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_retreat.robot->model,
                robot.arm1_solver_retreat.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_retreat.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_retreat.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_retreat.robot);
    for (int i = 0; i < state.arm2_solver_retreat.num_joints; ++i) {
        state.arm2_solver_retreat.q(i) = robot.arm2_solver_retreat.robot->jnt_pos_msr[i];
        state.arm2_solver_retreat.qd(i) = robot.arm2_solver_retreat.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_retreat(state.arm2_solver_retreat.q, state.arm2_solver_retreat.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_retreat.robot->model,
                robot.arm2_solver_retreat.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_retreat;
        mj_kdl::get_body_frame(
                robot.arm2_solver_retreat.robot->model,
                robot.arm2_solver_retreat.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_retreat);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_retreat.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_retreat.chain);
        fk.JntToCart(
            state.arm2_solver_retreat.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_retreat.chain);
        fk.JntToCart(
            state.arm2_solver_retreat.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_retreat.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_retreat,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_retreat.robot->model,
                robot.arm2_solver_retreat.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_retreat.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_retreat.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.retreat_support1_z_add_out = shared.pose_elbow1_base.p[2] + shared.retreat_support_lift;
        shared.retreat_support1_z = shared.retreat_support1_z_add_out;

        shared.retreat_support2_z_add_out = shared.pose_elbow2_base.p[2] + shared.retreat_support_lift;
        shared.retreat_support2_z = shared.retreat_support2_z_add_out;
        state.snapshot_taken = true;
    }
    shared.goal1_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    shared.goal2_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
}

inline bool can_start_motion_retreat(
    shared_data &shared
) {
    // eval_retreat_when_released1
    shared.eval_retreat_when_released1_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_retreat_when_released2
    shared.eval_retreat_when_released2_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released1_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released2_err, shared.satisfied_band_rot));
}

inline void monitor_when_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.goal1_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    shared.goal2_pose_retreat = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(0.4, 0.24, 0.22));
    // eval_retreat_when_released1
    shared.eval_retreat_when_released1_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_retreat_when_released2
    shared.eval_retreat_when_released2_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released1_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released2_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(11);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_RETREAT_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_RETREAT_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_retreat_until_at_retreat1_settled
    shared.eval_retreat_until_at_retreat1_settled_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose_retreat.p, shared.pose_ee1_base.p);
    // eval_retreat_until_at_retreat1_settled_rot
    shared.eval_retreat_until_at_retreat1_settled_rot_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose_retreat.M, shared.pose_ee1_base.M);
    // eval_retreat_until_at_retreat2_settled
    shared.eval_retreat_until_at_retreat2_settled_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose_retreat.p, shared.pose_ee2_base.p);
    // eval_retreat_until_at_retreat2_settled_rot
    shared.eval_retreat_until_at_retreat2_settled_rot_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose_retreat.M, shared.pose_ee2_base.M);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat1_settled_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat1_settled_rot_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat2_settled_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat2_settled_rot_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_settled_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_RETREAT_SETTLED);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_RETREAT_SETTLED] << std::endl;
        }
    }

}

inline void monitor_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_retreat_when_released1
    shared.eval_retreat_when_released1_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_retreat_when_released2
    shared.eval_retreat_when_released2_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released1_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_when_released2_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(11);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_RETREAT_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_RETREAT_READY] << std::endl;
        }
    }

    // eval_retreat_until_at_retreat1_settled
    shared.eval_retreat_until_at_retreat1_settled_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose_retreat.p, shared.pose_ee1_base.p);
    // eval_retreat_until_at_retreat1_settled_rot
    shared.eval_retreat_until_at_retreat1_settled_rot_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal1_pose_retreat.M, shared.pose_ee1_base.M);
    // eval_retreat_until_at_retreat2_settled
    shared.eval_retreat_until_at_retreat2_settled_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose_retreat.p, shared.pose_ee2_base.p);
    // eval_retreat_until_at_retreat2_settled_rot
    shared.eval_retreat_until_at_retreat2_settled_rot_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal2_pose_retreat.M, shared.pose_ee2_base.M);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat1_settled_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat1_settled_rot_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat2_settled_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_retreat_until_at_retreat2_settled_rot_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_retreat_settled_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(12);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_RETREAT_SETTLED);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_RETREAT_SETTLED] << std::endl;
        }
    }

}

inline void control_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee1_base = shared.pose_ee1_base;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[0] = shared.retreat_place_x;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[1] = shared.retreat_place_y;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[2] = shared.retreat_place_above_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee1_base = KDL::diff(shared.pose_ee1_base, _pose_axis_target_pose_axis_error_pose_ee1_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.z();

        shared.pose_ee1_base_distance_x_err_retreat = _pose_axis_error_linear_X;
        shared.pose_ee1_base_distance_y_err_retreat = _pose_axis_error_linear_Y;
        shared.pose_ee1_base_distance_z_err_retreat = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee2_base = shared.pose_ee2_base;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[0] = shared.retreat_place_x;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[1] = shared.retreat_place_y;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[2] = shared.retreat_place_above_z;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee2_base = KDL::diff(shared.pose_ee2_base, _pose_axis_target_pose_axis_error_pose_ee2_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.z();

        shared.pose_ee2_base_distance_x_err_retreat = _pose_axis_error_linear_X;
        shared.pose_ee2_base_distance_y_err_retreat = _pose_axis_error_linear_Y;
        shared.pose_ee2_base_distance_z_err_retreat = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_ret1_align_ori
    shared.pose_diff_ctrl_ret1_align_ori = KDL::diff(shared.pose_ee1_base, shared.goal1_pose_retreat);
    shared.ctrl_ret1_align_ori_err_ang_x = shared.pose_diff_ctrl_ret1_align_ori.rot[0];
    shared.ctrl_ret1_align_ori_err_ang_y = shared.pose_diff_ctrl_ret1_align_ori.rot[1];
    shared.ctrl_ret1_align_ori_err_ang_z = shared.pose_diff_ctrl_ret1_align_ori.rot[2];
    // eval_pose_diff_ctrl_ret2_align_ori
    shared.pose_diff_ctrl_ret2_align_ori = KDL::diff(shared.pose_ee2_base, shared.goal2_pose_retreat);
    shared.ctrl_ret2_align_ori_err_ang_x = shared.pose_diff_ctrl_ret2_align_ori.rot[0];
    shared.ctrl_ret2_align_ori_err_ang_y = shared.pose_diff_ctrl_ret2_align_ori.rot[1];
    shared.ctrl_ret2_align_ori_err_ang_z = shared.pose_diff_ctrl_ret2_align_ori.rot[2];
    // eval_retreat_while_support1_elbow_z
    shared.pose_elbow1_base_distance_z_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.retreat_support1_z, shared.pose_elbow1_base.p[2]);
    // eval_retreat_while_support2_elbow_z
    shared.pose_elbow2_base_distance_z_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.retreat_support2_z, shared.pose_elbow2_base.p[2]);
    // compute_wrench_force_ctrl_ret1_support_z
    shared.wrench_force_ctrl_ret1_support_z = KDL::Wrench(shared.direction_ctrl_ret1_support_z * shared.force_ctrl_ret1_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_ret1_support_z);
    // compute_wrench_force_ctrl_ret2_support_z
    shared.wrench_force_ctrl_ret2_support_z = KDL::Wrench(shared.direction_ctrl_ret2_support_z * shared.force_ctrl_ret2_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_ret2_support_z);
    // eval_retreat_while_open1_gripper
    shared.gripper1_pos_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_retreat_while_open2_gripper
    shared.gripper2_pos_err_retreat = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);
    // ctrl_ret2_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_ret2_open_gripper = _control_signal;
    }
    // ctrl_ret2_support_z
    {
        const double _control_signal = state.ctrl_ret2_support_z.control(shared.pose_elbow2_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret2_support_z_stiffness, shared.ctrl_ret2_support_z_damping, shared.ctrl_ret2_support_z_integral_gain});
        shared.force_ctrl_ret2_support_z = _control_signal;
        shared.ctrl_ret2_support_z_error_integral = state.ctrl_ret2_support_z.error_integral();
        shared.ctrl_ret2_support_z_previous_error = state.ctrl_ret2_support_z.previous_error();
        shared.ctrl_ret2_support_z_first_sample = state.ctrl_ret2_support_z.is_first_sample();
    }
    // ctrl_ret2_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_ret2_align_ori_ang_z.control(shared.pose_diff_ctrl_ret2_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_ret2_align_ori_ang_z_kp, shared.ctrl_ret2_align_ori_ang_z_ki, shared.ctrl_ret2_align_ori_ang_z_kd, shared.ctrl_ret2_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_ret2_align_ori_ang_z = _control_signal;
        shared.ctrl_ret2_align_ori_ang_z_error_integral = state.ctrl_ret2_align_ori_ang_z.error_integral();
        shared.ctrl_ret2_align_ori_ang_z_previous_error = state.ctrl_ret2_align_ori_ang_z.previous_error();
        shared.ctrl_ret2_align_ori_ang_z_first_sample = state.ctrl_ret2_align_ori_ang_z.is_first_sample();
    }
    // ctrl_ret2_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_ret2_align_ori_ang_y.control(shared.pose_diff_ctrl_ret2_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_ret2_align_ori_ang_y_kp, shared.ctrl_ret2_align_ori_ang_y_ki, shared.ctrl_ret2_align_ori_ang_y_kd, shared.ctrl_ret2_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_ret2_align_ori_ang_y = _control_signal;
        shared.ctrl_ret2_align_ori_ang_y_error_integral = state.ctrl_ret2_align_ori_ang_y.error_integral();
        shared.ctrl_ret2_align_ori_ang_y_previous_error = state.ctrl_ret2_align_ori_ang_y.previous_error();
        shared.ctrl_ret2_align_ori_ang_y_first_sample = state.ctrl_ret2_align_ori_ang_y.is_first_sample();
    }
    // ctrl_ret2_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_ret2_align_ori_ang_x.control(shared.pose_diff_ctrl_ret2_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_ret2_align_ori_ang_x_kp, shared.ctrl_ret2_align_ori_ang_x_ki, shared.ctrl_ret2_align_ori_ang_x_kd, shared.ctrl_ret2_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_ret2_align_ori_ang_x = _control_signal;
        shared.ctrl_ret2_align_ori_ang_x_error_integral = state.ctrl_ret2_align_ori_ang_x.error_integral();
        shared.ctrl_ret2_align_ori_ang_x_previous_error = state.ctrl_ret2_align_ori_ang_x.previous_error();
        shared.ctrl_ret2_align_ori_ang_x_first_sample = state.ctrl_ret2_align_ori_ang_x.is_first_sample();
    }
    // ctrl_ret2_reach_z
    {
        const double _control_signal = state.ctrl_ret2_reach_z.control(shared.pose_ee2_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret2_reach_z_kp, shared.ctrl_ret2_reach_z_ki, shared.ctrl_ret2_reach_z_kd, shared.ctrl_ret2_reach_z_decay_rate});
        shared.eacc_pose_ee2_base_distance_z_retreat = _control_signal;
        shared.ctrl_ret2_reach_z_error_integral = state.ctrl_ret2_reach_z.error_integral();
        shared.ctrl_ret2_reach_z_previous_error = state.ctrl_ret2_reach_z.previous_error();
        shared.ctrl_ret2_reach_z_first_sample = state.ctrl_ret2_reach_z.is_first_sample();
    }
    // ctrl_ret2_reach_y
    {
        const double _control_signal = state.ctrl_ret2_reach_y.control(shared.pose_ee2_base_distance_y_err_retreat, shared.dt_measured_s, {shared.ctrl_ret2_reach_y_kp, shared.ctrl_ret2_reach_y_ki, shared.ctrl_ret2_reach_y_kd, shared.ctrl_ret2_reach_y_decay_rate});
        shared.eacc_pose_ee2_base_distance_y_retreat = _control_signal;
        shared.ctrl_ret2_reach_y_error_integral = state.ctrl_ret2_reach_y.error_integral();
        shared.ctrl_ret2_reach_y_previous_error = state.ctrl_ret2_reach_y.previous_error();
        shared.ctrl_ret2_reach_y_first_sample = state.ctrl_ret2_reach_y.is_first_sample();
    }
    // ctrl_ret2_reach_x
    {
        const double _control_signal = state.ctrl_ret2_reach_x.control(shared.pose_ee2_base_distance_x_err_retreat, shared.dt_measured_s, {shared.ctrl_ret2_reach_x_kp, shared.ctrl_ret2_reach_x_ki, shared.ctrl_ret2_reach_x_kd, shared.ctrl_ret2_reach_x_decay_rate});
        shared.eacc_pose_ee2_base_distance_x_retreat = _control_signal;
        shared.ctrl_ret2_reach_x_error_integral = state.ctrl_ret2_reach_x.error_integral();
        shared.ctrl_ret2_reach_x_previous_error = state.ctrl_ret2_reach_x.previous_error();
        shared.ctrl_ret2_reach_x_first_sample = state.ctrl_ret2_reach_x.is_first_sample();
    }
    // ctrl_ret1_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_ret1_open_gripper = _control_signal;
    }
    // ctrl_ret1_support_z
    {
        const double _control_signal = state.ctrl_ret1_support_z.control(shared.pose_elbow1_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret1_support_z_stiffness, shared.ctrl_ret1_support_z_damping, shared.ctrl_ret1_support_z_integral_gain});
        shared.force_ctrl_ret1_support_z = _control_signal;
        shared.ctrl_ret1_support_z_error_integral = state.ctrl_ret1_support_z.error_integral();
        shared.ctrl_ret1_support_z_previous_error = state.ctrl_ret1_support_z.previous_error();
        shared.ctrl_ret1_support_z_first_sample = state.ctrl_ret1_support_z.is_first_sample();
    }
    // ctrl_ret1_align_ori_ang_z
    {
        const double _control_signal = state.ctrl_ret1_align_ori_ang_z.control(shared.pose_diff_ctrl_ret1_align_ori.rot[2], shared.dt_measured_s, {shared.ctrl_ret1_align_ori_ang_z_kp, shared.ctrl_ret1_align_ori_ang_z_ki, shared.ctrl_ret1_align_ori_ang_z_kd, shared.ctrl_ret1_align_ori_ang_z_decay_rate});
        shared.eacc_ctrl_ret1_align_ori_ang_z = _control_signal;
        shared.ctrl_ret1_align_ori_ang_z_error_integral = state.ctrl_ret1_align_ori_ang_z.error_integral();
        shared.ctrl_ret1_align_ori_ang_z_previous_error = state.ctrl_ret1_align_ori_ang_z.previous_error();
        shared.ctrl_ret1_align_ori_ang_z_first_sample = state.ctrl_ret1_align_ori_ang_z.is_first_sample();
    }
    // ctrl_ret1_align_ori_ang_y
    {
        const double _control_signal = state.ctrl_ret1_align_ori_ang_y.control(shared.pose_diff_ctrl_ret1_align_ori.rot[1], shared.dt_measured_s, {shared.ctrl_ret1_align_ori_ang_y_kp, shared.ctrl_ret1_align_ori_ang_y_ki, shared.ctrl_ret1_align_ori_ang_y_kd, shared.ctrl_ret1_align_ori_ang_y_decay_rate});
        shared.eacc_ctrl_ret1_align_ori_ang_y = _control_signal;
        shared.ctrl_ret1_align_ori_ang_y_error_integral = state.ctrl_ret1_align_ori_ang_y.error_integral();
        shared.ctrl_ret1_align_ori_ang_y_previous_error = state.ctrl_ret1_align_ori_ang_y.previous_error();
        shared.ctrl_ret1_align_ori_ang_y_first_sample = state.ctrl_ret1_align_ori_ang_y.is_first_sample();
    }
    // ctrl_ret1_align_ori_ang_x
    {
        const double _control_signal = state.ctrl_ret1_align_ori_ang_x.control(shared.pose_diff_ctrl_ret1_align_ori.rot[0], shared.dt_measured_s, {shared.ctrl_ret1_align_ori_ang_x_kp, shared.ctrl_ret1_align_ori_ang_x_ki, shared.ctrl_ret1_align_ori_ang_x_kd, shared.ctrl_ret1_align_ori_ang_x_decay_rate});
        shared.eacc_ctrl_ret1_align_ori_ang_x = _control_signal;
        shared.ctrl_ret1_align_ori_ang_x_error_integral = state.ctrl_ret1_align_ori_ang_x.error_integral();
        shared.ctrl_ret1_align_ori_ang_x_previous_error = state.ctrl_ret1_align_ori_ang_x.previous_error();
        shared.ctrl_ret1_align_ori_ang_x_first_sample = state.ctrl_ret1_align_ori_ang_x.is_first_sample();
    }
    // ctrl_ret1_reach_z
    {
        const double _control_signal = state.ctrl_ret1_reach_z.control(shared.pose_ee1_base_distance_z_err_retreat, shared.dt_measured_s, {shared.ctrl_ret1_reach_z_kp, shared.ctrl_ret1_reach_z_ki, shared.ctrl_ret1_reach_z_kd, shared.ctrl_ret1_reach_z_decay_rate});
        shared.eacc_pose_ee1_base_distance_z_retreat = _control_signal;
        shared.ctrl_ret1_reach_z_error_integral = state.ctrl_ret1_reach_z.error_integral();
        shared.ctrl_ret1_reach_z_previous_error = state.ctrl_ret1_reach_z.previous_error();
        shared.ctrl_ret1_reach_z_first_sample = state.ctrl_ret1_reach_z.is_first_sample();
    }
    // ctrl_ret1_reach_y
    {
        const double _control_signal = state.ctrl_ret1_reach_y.control(shared.pose_ee1_base_distance_y_err_retreat, shared.dt_measured_s, {shared.ctrl_ret1_reach_y_kp, shared.ctrl_ret1_reach_y_ki, shared.ctrl_ret1_reach_y_kd, shared.ctrl_ret1_reach_y_decay_rate});
        shared.eacc_pose_ee1_base_distance_y_retreat = _control_signal;
        shared.ctrl_ret1_reach_y_error_integral = state.ctrl_ret1_reach_y.error_integral();
        shared.ctrl_ret1_reach_y_previous_error = state.ctrl_ret1_reach_y.previous_error();
        shared.ctrl_ret1_reach_y_first_sample = state.ctrl_ret1_reach_y.is_first_sample();
    }
    // ctrl_ret1_reach_x
    {
        const double _control_signal = state.ctrl_ret1_reach_x.control(shared.pose_ee1_base_distance_x_err_retreat, shared.dt_measured_s, {shared.ctrl_ret1_reach_x_kp, shared.ctrl_ret1_reach_x_ki, shared.ctrl_ret1_reach_x_kd, shared.ctrl_ret1_reach_x_decay_rate});
        shared.eacc_pose_ee1_base_distance_x_retreat = _control_signal;
        shared.ctrl_ret1_reach_x_error_integral = state.ctrl_ret1_reach_x.error_integral();
        shared.ctrl_ret1_reach_x_previous_error = state.ctrl_ret1_reach_x.previous_error();
        shared.ctrl_ret1_reach_x_first_sample = state.ctrl_ret1_reach_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_retreat.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_0(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_0.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_0 =
            alpha_frame_arm1_solver_retreat_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_retreat_0[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_retreat_0[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_retreat_0[2];
    }

    state.arm1_solver_retreat.acceleration_energy(0) = shared.eacc_pose_ee1_base_distance_x_retreat;

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_1(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_1.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_1 =
            alpha_frame_arm1_solver_retreat_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_retreat_1[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_retreat_1[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_retreat_1[2];
    }

    state.arm1_solver_retreat.acceleration_energy(1) = shared.eacc_pose_ee1_base_distance_y_retreat;

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_2(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_2.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_2 =
            alpha_frame_arm1_solver_retreat_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_retreat_2[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_retreat_2[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_retreat_2[2];
    }

    state.arm1_solver_retreat.acceleration_energy(2) = shared.eacc_pose_ee1_base_distance_z_retreat;

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_3(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_3.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_3 =
            alpha_frame_arm1_solver_retreat_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_retreat_3[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_retreat_3[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_retreat_3[2];
    }

    state.arm1_solver_retreat.acceleration_energy(3) = shared.eacc_ctrl_ret1_align_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_4(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_4.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_4 =
            alpha_frame_arm1_solver_retreat_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_retreat_4[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_retreat_4[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_retreat_4[2];
    }

    state.arm1_solver_retreat.acceleration_energy(4) = shared.eacc_ctrl_ret1_align_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_retreat_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_retreat_5(*robot.arm1_solver_retreat.chain);
        alpha_fk_arm1_solver_retreat_5.JntToCart(
            state.arm1_solver_retreat.q,
            alpha_frame_arm1_solver_retreat_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_retreat_5 =
            alpha_frame_arm1_solver_retreat_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_retreat_5[0];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_retreat_5[1];
        state.arm1_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_retreat_5[2];
    }

    state.arm1_solver_retreat.acceleration_energy(5) = shared.eacc_ctrl_ret1_align_ori_ang_z;

    KDL::SetToZero(state.arm1_solver_retreat.tau_ff);

    for (int i = 0; i < state.arm1_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(state.arm1_solver_retreat.f_ext[i]);
    }

    state.arm1_solver_retreat.f_ext[motion_spec::runtime::find_segment_index(*robot.arm1_solver_retreat.chain, "half_arm_2_link", "kinova1_base_link") - 1] += shared.wrench_force_ctrl_ret1_support_z;

    KDL::Wrenches f_ext_zero_arm1_solver_retreat(state.arm1_solver_retreat.num_segments);
    for (int i = 0; i < state.arm1_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_retreat[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_retreat(state.arm1_solver_retreat.num_joints);
    state.arm1_solver_retreat.achd_acc->CartToJnt(
        state.arm1_solver_retreat.q,
        state.arm1_solver_retreat.qd,
        state.arm1_solver_retreat.qdd,
        state.arm1_solver_retreat.spatial_directions,
        state.arm1_solver_retreat.acceleration_energy,
        state.arm1_solver_retreat.f_ext,
        state.arm1_solver_retreat.tau_ff,
        tau_ctrl_acc_arm1_solver_retreat);
    state.arm1_solver_retreat.rnea->CartToJnt(
        state.arm1_solver_retreat.q,
        state.arm1_solver_retreat.qd,
        state.arm1_solver_retreat.qdd,
        f_ext_zero_arm1_solver_retreat,
        state.arm1_solver_retreat.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_retreat.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_retreat.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_retreat.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_retreat.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_retreat.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_retreat.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_retreat.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_retreat.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_retreat.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_retreat.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_retreat.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_retreat.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_retreat.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_retreat.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_retreat.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_retreat.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_retreat.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_retreat.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_retreat.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_retreat.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_retreat.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_retreat.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_retreat.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_retreat.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_retreat.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_retreat.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_retreat.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_retreat.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_retreat.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_0(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_0.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_0 =
            alpha_frame_arm2_solver_retreat_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_retreat_0[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_retreat_0[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_retreat_0[2];
    }

    state.arm2_solver_retreat.acceleration_energy(0) = shared.eacc_pose_ee2_base_distance_x_retreat;

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_1(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_1.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_1 =
            alpha_frame_arm2_solver_retreat_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_retreat_1[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_retreat_1[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_retreat_1[2];
    }

    state.arm2_solver_retreat.acceleration_energy(1) = shared.eacc_pose_ee2_base_distance_y_retreat;

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_2(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_2.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_2 =
            alpha_frame_arm2_solver_retreat_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_retreat_2[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_retreat_2[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_retreat_2[2];
    }

    state.arm2_solver_retreat.acceleration_energy(2) = shared.eacc_pose_ee2_base_distance_z_retreat;

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_3(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_3.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_3 =
            alpha_frame_arm2_solver_retreat_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_retreat_3[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_retreat_3[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_retreat_3[2];
    }

    state.arm2_solver_retreat.acceleration_energy(3) = shared.eacc_ctrl_ret2_align_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_4(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_4.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_4 =
            alpha_frame_arm2_solver_retreat_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_retreat_4[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_retreat_4[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_retreat_4[2];
    }

    state.arm2_solver_retreat.acceleration_energy(4) = shared.eacc_ctrl_ret2_align_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_retreat_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_retreat_5(*robot.arm2_solver_retreat.chain);
        alpha_fk_arm2_solver_retreat_5.JntToCart(
            state.arm2_solver_retreat.q,
            alpha_frame_arm2_solver_retreat_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_retreat_5 =
            alpha_frame_arm2_solver_retreat_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_retreat_5[0];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_retreat_5[1];
        state.arm2_solver_retreat.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_retreat_5[2];
    }

    state.arm2_solver_retreat.acceleration_energy(5) = shared.eacc_ctrl_ret2_align_ori_ang_z;

    KDL::SetToZero(state.arm2_solver_retreat.tau_ff);

    for (int i = 0; i < state.arm2_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(state.arm2_solver_retreat.f_ext[i]);
    }

    state.arm2_solver_retreat.f_ext[motion_spec::runtime::find_segment_index(*robot.arm2_solver_retreat.chain, "half_arm_2_link", "kinova2_base_link") - 1] += shared.wrench_force_ctrl_ret2_support_z;

    KDL::Wrenches f_ext_zero_arm2_solver_retreat(state.arm2_solver_retreat.num_segments);
    for (int i = 0; i < state.arm2_solver_retreat.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_retreat[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_retreat(state.arm2_solver_retreat.num_joints);
    state.arm2_solver_retreat.achd_acc->CartToJnt(
        state.arm2_solver_retreat.q,
        state.arm2_solver_retreat.qd,
        state.arm2_solver_retreat.qdd,
        state.arm2_solver_retreat.spatial_directions,
        state.arm2_solver_retreat.acceleration_energy,
        state.arm2_solver_retreat.f_ext,
        state.arm2_solver_retreat.tau_ff,
        tau_ctrl_acc_arm2_solver_retreat);
    state.arm2_solver_retreat.rnea->CartToJnt(
        state.arm2_solver_retreat.q,
        state.arm2_solver_retreat.qd,
        state.arm2_solver_retreat.qdd,
        f_ext_zero_arm2_solver_retreat,
        state.arm2_solver_retreat.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_retreat.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_retreat.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_retreat.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_retreat.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_retreat.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_retreat.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_retreat.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_retreat.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_retreat.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_retreat.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_retreat.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_retreat.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_retreat.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_retreat.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_retreat.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_retreat.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_retreat.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_retreat.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_retreat.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_retreat.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_retreat.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_retreat.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_retreat.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_retreat.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_retreat.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_retreat.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_retreat.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_retreat.tau_ctrl(6);

}

inline void apply_motion_retreat(
    motion_retreat_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_retreat.num_joints; ++i) {
        robot.arm1_solver_retreat.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_retreat.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_retreat.num_joints; ++i) {
        robot.arm2_solver_retreat.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_retreat.tau_ctrl(i), i);
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
            robot.arm1_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_ret1_open_gripper;
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
            robot.arm2_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_ret2_open_gripper;
        }
    }

}
