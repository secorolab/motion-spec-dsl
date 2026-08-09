/// Motion: pick-above
/// Move both TCPs to the pre-grasp position above their cubes along paths
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_pick_above_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_pick_above_solver_state arm1_solver_pick_above;
    arm2_solver_pick_above_solver_state arm2_solver_pick_above;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_tan;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_lat_lin_normal_a;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_lat_lin_normal_b;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pa1_follow_ori_ang_z;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_tan;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_lat_lin_normal_a;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_lat_lin_normal_b;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_ori_ang_x;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_ori_ang_y;
    motion_spec::runtime::PIDControl ctrl_pa2_follow_ori_ang_z;
    bool mon_pick_above_ready_previous = false;
    bool mon_pick_above_ready_event_triggered = false;

    bool mon_pa1_advancing_pa1_advancing = false;
    bool mon_pa2_advancing_pa2_advancing = false;
};

inline void reset_motion_pick_above(motion_pick_above_state &state) {
    state = motion_pick_above_state{};
}

inline void init_motion_pick_above(motion_pick_above_state &state, const robot_io &robot) {
    if (!state.arm1_solver_pick_above.initialized) {
        state.arm1_solver_pick_above.num_joints = robot.arm1_solver_pick_above.chain->getNrOfJoints();
        state.arm1_solver_pick_above.num_segments = robot.arm1_solver_pick_above.chain->getNrOfSegments();
        state.arm1_solver_pick_above.q = KDL::JntArray(state.arm1_solver_pick_above.num_joints);
        state.arm1_solver_pick_above.qd = KDL::JntArray(state.arm1_solver_pick_above.num_joints);
        state.arm1_solver_pick_above.qdd = KDL::JntArray(state.arm1_solver_pick_above.num_joints);
        state.arm1_solver_pick_above.tau_ff = KDL::JntArray(state.arm1_solver_pick_above.num_joints);
        state.arm1_solver_pick_above.tau_ctrl = KDL::JntArray(state.arm1_solver_pick_above.num_joints);
        state.arm1_solver_pick_above.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_pick_above.num_spatial_directions = 6;
        state.arm1_solver_pick_above.spatial_directions = KDL::Jacobian(state.arm1_solver_pick_above.num_spatial_directions);
        state.arm1_solver_pick_above.acceleration_energy = KDL::JntArray(state.arm1_solver_pick_above.num_spatial_directions);
        state.arm1_solver_pick_above.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_pick_above.chain, state.arm1_solver_pick_above.root_acc, state.arm1_solver_pick_above.num_spatial_directions);
        state.arm1_solver_pick_above.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_pick_above.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_pick_above.initialized = true;
    }
    if (!state.arm2_solver_pick_above.initialized) {
        state.arm2_solver_pick_above.num_joints = robot.arm2_solver_pick_above.chain->getNrOfJoints();
        state.arm2_solver_pick_above.num_segments = robot.arm2_solver_pick_above.chain->getNrOfSegments();
        state.arm2_solver_pick_above.q = KDL::JntArray(state.arm2_solver_pick_above.num_joints);
        state.arm2_solver_pick_above.qd = KDL::JntArray(state.arm2_solver_pick_above.num_joints);
        state.arm2_solver_pick_above.qdd = KDL::JntArray(state.arm2_solver_pick_above.num_joints);
        state.arm2_solver_pick_above.tau_ff = KDL::JntArray(state.arm2_solver_pick_above.num_joints);
        state.arm2_solver_pick_above.tau_ctrl = KDL::JntArray(state.arm2_solver_pick_above.num_joints);
        state.arm2_solver_pick_above.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_pick_above.num_spatial_directions = 6;
        state.arm2_solver_pick_above.spatial_directions = KDL::Jacobian(state.arm2_solver_pick_above.num_spatial_directions);
        state.arm2_solver_pick_above.acceleration_energy = KDL::JntArray(state.arm2_solver_pick_above.num_spatial_directions);
        state.arm2_solver_pick_above.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_pick_above.chain, state.arm2_solver_pick_above.root_acc, state.arm2_solver_pick_above.num_spatial_directions);
        state.arm2_solver_pick_above.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_pick_above.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_pick_above.initialized = true;
    }
}

inline void update_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_pick_above(state, robot);

    mj_kdl::update(robot.arm1_solver_pick_above.robot);
    for (int i = 0; i < state.arm1_solver_pick_above.num_joints; ++i) {
        state.arm1_solver_pick_above.q(i) = robot.arm1_solver_pick_above.robot->jnt_pos_msr[i];
        state.arm1_solver_pick_above.qd(i) = robot.arm1_solver_pick_above.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_pick_above(state.arm1_solver_pick_above.q, state.arm1_solver_pick_above.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_pick_above.robot->model,
                robot.arm1_solver_pick_above.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_pick_above;
        mj_kdl::get_body_frame(
                robot.arm1_solver_pick_above.robot->model,
                robot.arm1_solver_pick_above.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_pick_above);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_pick_above.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_pick_above.chain);
        fk.JntToCart(
            state.arm1_solver_pick_above.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_pick_above.chain);
        fk.JntToCart(
            state.arm1_solver_pick_above.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_pick_above.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_pick_above,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_pick_above.robot->model,
                robot.arm1_solver_pick_above.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_pick_above.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_pick_above.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_pick_above.robot);
    for (int i = 0; i < state.arm2_solver_pick_above.num_joints; ++i) {
        state.arm2_solver_pick_above.q(i) = robot.arm2_solver_pick_above.robot->jnt_pos_msr[i];
        state.arm2_solver_pick_above.qd(i) = robot.arm2_solver_pick_above.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_pick_above(state.arm2_solver_pick_above.q, state.arm2_solver_pick_above.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_pick_above.robot->model,
                robot.arm2_solver_pick_above.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_pick_above;
        mj_kdl::get_body_frame(
                robot.arm2_solver_pick_above.robot->model,
                robot.arm2_solver_pick_above.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_pick_above);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_pick_above.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_pick_above.chain);
        fk.JntToCart(
            state.arm2_solver_pick_above.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_pick_above.chain);
        fk.JntToCart(
            state.arm2_solver_pick_above.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_pick_above.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_pick_above,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_pick_above.robot->model,
                robot.arm2_solver_pick_above.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_pick_above.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_pick_above.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.start1_cube_x = shared.pose_cube1_base.p[0];
        shared.start1_cube_y = shared.pose_cube1_base.p[1];
        shared.start1_pose = shared.pose_ee1_base;
        shared.start2_cube_x = shared.pose_cube2_base.p[0];
        shared.start2_cube_y = shared.pose_cube2_base.p[1];
        shared.start2_pose = shared.pose_ee2_base;
        state.snapshot_taken = true;
    }
    shared.goal1_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start1_cube_x, shared.start1_cube_y, 0.26));
    shared.goal2_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start2_cube_x, shared.start2_cube_y, 0.26));
}

inline bool can_start_motion_pick_above(
    shared_data &shared
) {
    // eval_pick_above_when_gripper1_ready
    shared.eval_pick_above_when_gripper1_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_pick_above_when_gripper2_ready
    shared.eval_pick_above_when_gripper2_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper1_ready_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper2_ready_err, shared.satisfied_band_rot));
}

inline void monitor_when_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    shared.goal1_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start1_cube_x, shared.start1_cube_y, 0.26));
    shared.goal2_pose = KDL::Frame(
        motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))),
        KDL::Vector(shared.start2_cube_x, shared.start2_cube_y, 0.26));
    // eval_pick_above_when_gripper1_ready
    shared.eval_pick_above_when_gripper1_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_pick_above_when_gripper2_ready
    shared.eval_pick_above_when_gripper2_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper1_ready_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper2_ready_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_above_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(6);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PICK_ABOVE_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PICK_ABOVE_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_pick_above() {
}

inline void monitor_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_pick_above_when_gripper1_ready
    shared.eval_pick_above_when_gripper1_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_pick_above_when_gripper2_ready
    shared.eval_pick_above_when_gripper2_ready_err = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper1_ready_err, shared.satisfied_band_rot) && motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_when_gripper2_ready_err, shared.satisfied_band_rot));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_pick_above_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(6);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_PICK_ABOVE_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_PICK_ABOVE_READY] << std::endl;
        }
    }

}

inline void control_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // projection_arm1_approach_path
    {
        const KDL::Frame _goal = KDL::Frame(motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))), KDL::Vector(shared.start1_cube_x, shared.start1_cube_y, 0.26));
        shared.goal1_pose = _goal;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start1_pose, KDL::diff(shared.start1_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.arm1_approach_path_s = motion_spec::runtime::path_project(
            _path_eval, shared.pose_ee1_base.p, shared.arm1_approach_path_s);
    }
    // frame_arm1_approach_path
    {
        const KDL::Frame _goal = shared.goal1_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start1_pose, KDL::diff(shared.start1_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        motion_spec::runtime::path_frame(
            _path_eval, shared.arm1_approach_path_s,
            shared.arm1_approach_path_tangent, shared.arm1_approach_path_normal_a, shared.arm1_approach_path_normal_b);
    }
    // along_arm1_approach_path
    shared.arm1_approach_path_along_speed = KDL::dot(shared.twist_ee1_base.vel, shared.arm1_approach_path_tangent);
    // projection_arm2_approach_path
    {
        const KDL::Frame _goal = KDL::Frame(motion_spec::runtime::quat_rotation(motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('z', 1.5708), motion_spec::runtime::quat_mul(motion_spec::runtime::quat_axis('y', 0.0), motion_spec::runtime::quat_axis('x', 3.14159)))), KDL::Vector(shared.start2_cube_x, shared.start2_cube_y, 0.26));
        shared.goal2_pose = _goal;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start2_pose, KDL::diff(shared.start2_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.arm2_approach_path_s = motion_spec::runtime::path_project(
            _path_eval, shared.pose_ee2_base.p, shared.arm2_approach_path_s);
    }
    // frame_arm2_approach_path
    {
        const KDL::Frame _goal = shared.goal2_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start2_pose, KDL::diff(shared.start2_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        motion_spec::runtime::path_frame(
            _path_eval, shared.arm2_approach_path_s,
            shared.arm2_approach_path_tangent, shared.arm2_approach_path_normal_a, shared.arm2_approach_path_normal_b);
    }
    // along_arm2_approach_path
    shared.arm2_approach_path_along_speed = KDL::dot(shared.twist_ee2_base.vel, shared.arm2_approach_path_tangent);
    // evaluator_arm1_approach_path
    {
        const KDL::Frame _goal = shared.goal1_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start1_pose, KDL::diff(shared.start1_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.pick_above_arm1_approach_path_reference = _path_eval(shared.arm1_approach_path_s);
    }
    // eval_pose_diff_ctrl_pa1_follow_lat
    shared.pose_diff_ctrl_pa1_follow_lat = KDL::diff(shared.pose_ee1_base, shared.pick_above_arm1_approach_path_reference);
    shared.ctrl_pa1_follow_lat_err_lin_normal_a = KDL::dot(shared.pose_diff_ctrl_pa1_follow_lat.vel, shared.arm1_approach_path_normal_a);
    shared.ctrl_pa1_follow_lat_err_lin_normal_b = KDL::dot(shared.pose_diff_ctrl_pa1_follow_lat.vel, shared.arm1_approach_path_normal_b);
    // eval_pose_diff_ctrl_pa1_follow_ori
    shared.pose_diff_ctrl_pa1_follow_ori = KDL::diff(shared.pose_ee1_base, shared.pick_above_arm1_approach_path_reference);
    shared.ctrl_pa1_follow_ori_err_ang_x = shared.pose_diff_ctrl_pa1_follow_ori.rot[0];
    shared.ctrl_pa1_follow_ori_err_ang_y = shared.pose_diff_ctrl_pa1_follow_ori.rot[1];
    shared.ctrl_pa1_follow_ori_err_ang_z = shared.pose_diff_ctrl_pa1_follow_ori.rot[2];
    // evaluator_arm2_approach_path
    {
        const KDL::Frame _goal = shared.goal2_pose;
        const auto _path_eval = [&](double _s) -> KDL::Frame {
            return KDL::addDelta(shared.start2_pose, KDL::diff(shared.start2_pose, _goal),
                                 motion_spec::runtime::clamp01(_s));
        };
        shared.pick_above_arm2_approach_path_reference = _path_eval(shared.arm2_approach_path_s);
    }
    // eval_pose_diff_ctrl_pa2_follow_lat
    shared.pose_diff_ctrl_pa2_follow_lat = KDL::diff(shared.pose_ee2_base, shared.pick_above_arm2_approach_path_reference);
    shared.ctrl_pa2_follow_lat_err_lin_normal_a = KDL::dot(shared.pose_diff_ctrl_pa2_follow_lat.vel, shared.arm2_approach_path_normal_a);
    shared.ctrl_pa2_follow_lat_err_lin_normal_b = KDL::dot(shared.pose_diff_ctrl_pa2_follow_lat.vel, shared.arm2_approach_path_normal_b);
    // eval_pose_diff_ctrl_pa2_follow_ori
    shared.pose_diff_ctrl_pa2_follow_ori = KDL::diff(shared.pose_ee2_base, shared.pick_above_arm2_approach_path_reference);
    shared.ctrl_pa2_follow_ori_err_ang_x = shared.pose_diff_ctrl_pa2_follow_ori.rot[0];
    shared.ctrl_pa2_follow_ori_err_ang_y = shared.pose_diff_ctrl_pa2_follow_ori.rot[1];
    shared.ctrl_pa2_follow_ori_err_ang_z = shared.pose_diff_ctrl_pa2_follow_ori.rot[2];
    // eval_pick_above_while_follow1_tan
    shared.arm1_approach_path_along_speed_err_pick_above = motion_spec::runtime::evaluate_equality_constraint(shared.approach_speed, shared.arm1_approach_path_along_speed);
    // eval_pick_above_while_follow2_tan
    shared.arm2_approach_path_along_speed_err_pick_above = motion_spec::runtime::evaluate_equality_constraint(shared.approach_speed, shared.arm2_approach_path_along_speed);
    // eval_pick_above_while_advance1
    shared.eval_pick_above_while_advance1_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.arm1_approach_path_along_speed, shared.min_approach_speed);
    // eval_pick_above_while_advance2
    shared.eval_pick_above_while_advance2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.arm2_approach_path_along_speed, shared.min_approach_speed);
    // ctrl_pa2_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pa2_follow_ori_ang_z.control(shared.pose_diff_ctrl_pa2_follow_ori.rot[2], -(shared.twist_ee2_base.rot[2]), shared.dt_measured_s, {shared.ctrl_pa2_follow_ori_ang_z_kp, shared.ctrl_pa2_follow_ori_ang_z_ki, shared.ctrl_pa2_follow_ori_ang_z_kd, shared.ctrl_pa2_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pa2_follow_ori_ang_z = _control_signal;
        shared.ctrl_pa2_follow_ori_ang_z_error_integral = state.ctrl_pa2_follow_ori_ang_z.error_integral();
        shared.ctrl_pa2_follow_ori_ang_z_previous_error = state.ctrl_pa2_follow_ori_ang_z.previous_error();
        shared.ctrl_pa2_follow_ori_ang_z_first_sample = state.ctrl_pa2_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pa2_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pa2_follow_ori_ang_y.control(shared.pose_diff_ctrl_pa2_follow_ori.rot[1], -(shared.twist_ee2_base.rot[1]), shared.dt_measured_s, {shared.ctrl_pa2_follow_ori_ang_y_kp, shared.ctrl_pa2_follow_ori_ang_y_ki, shared.ctrl_pa2_follow_ori_ang_y_kd, shared.ctrl_pa2_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pa2_follow_ori_ang_y = _control_signal;
        shared.ctrl_pa2_follow_ori_ang_y_error_integral = state.ctrl_pa2_follow_ori_ang_y.error_integral();
        shared.ctrl_pa2_follow_ori_ang_y_previous_error = state.ctrl_pa2_follow_ori_ang_y.previous_error();
        shared.ctrl_pa2_follow_ori_ang_y_first_sample = state.ctrl_pa2_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pa2_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pa2_follow_ori_ang_x.control(shared.pose_diff_ctrl_pa2_follow_ori.rot[0], -(shared.twist_ee2_base.rot[0]), shared.dt_measured_s, {shared.ctrl_pa2_follow_ori_ang_x_kp, shared.ctrl_pa2_follow_ori_ang_x_ki, shared.ctrl_pa2_follow_ori_ang_x_kd, shared.ctrl_pa2_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pa2_follow_ori_ang_x = _control_signal;
        shared.ctrl_pa2_follow_ori_ang_x_error_integral = state.ctrl_pa2_follow_ori_ang_x.error_integral();
        shared.ctrl_pa2_follow_ori_ang_x_previous_error = state.ctrl_pa2_follow_ori_ang_x.previous_error();
        shared.ctrl_pa2_follow_ori_ang_x_first_sample = state.ctrl_pa2_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pa2_follow_lat_lin_normal_b
    {
        const double _control_signal = state.ctrl_pa2_follow_lat_lin_normal_b.control(KDL::dot(shared.pose_diff_ctrl_pa2_follow_lat.vel, shared.arm2_approach_path_normal_b), -(KDL::dot(shared.twist_ee2_base.vel, shared.arm2_approach_path_normal_b)), shared.dt_measured_s, {shared.ctrl_pa2_follow_lat_lin_normal_b_kp, shared.ctrl_pa2_follow_lat_lin_normal_b_ki, shared.ctrl_pa2_follow_lat_lin_normal_b_kd, shared.ctrl_pa2_follow_lat_lin_normal_b_decay_rate});
        shared.eacc_ctrl_pa2_follow_lat_lin_normal_b = _control_signal;
        shared.ctrl_pa2_follow_lat_lin_normal_b_error_integral = state.ctrl_pa2_follow_lat_lin_normal_b.error_integral();
        shared.ctrl_pa2_follow_lat_lin_normal_b_previous_error = state.ctrl_pa2_follow_lat_lin_normal_b.previous_error();
        shared.ctrl_pa2_follow_lat_lin_normal_b_first_sample = state.ctrl_pa2_follow_lat_lin_normal_b.is_first_sample();
    }
    // ctrl_pa2_follow_lat_lin_normal_a
    {
        const double _control_signal = state.ctrl_pa2_follow_lat_lin_normal_a.control(KDL::dot(shared.pose_diff_ctrl_pa2_follow_lat.vel, shared.arm2_approach_path_normal_a), -(KDL::dot(shared.twist_ee2_base.vel, shared.arm2_approach_path_normal_a)), shared.dt_measured_s, {shared.ctrl_pa2_follow_lat_lin_normal_a_kp, shared.ctrl_pa2_follow_lat_lin_normal_a_ki, shared.ctrl_pa2_follow_lat_lin_normal_a_kd, shared.ctrl_pa2_follow_lat_lin_normal_a_decay_rate});
        shared.eacc_ctrl_pa2_follow_lat_lin_normal_a = _control_signal;
        shared.ctrl_pa2_follow_lat_lin_normal_a_error_integral = state.ctrl_pa2_follow_lat_lin_normal_a.error_integral();
        shared.ctrl_pa2_follow_lat_lin_normal_a_previous_error = state.ctrl_pa2_follow_lat_lin_normal_a.previous_error();
        shared.ctrl_pa2_follow_lat_lin_normal_a_first_sample = state.ctrl_pa2_follow_lat_lin_normal_a.is_first_sample();
    }
    // ctrl_pa2_follow_tan
    {
        const double _control_signal = state.ctrl_pa2_follow_tan.control(shared.arm2_approach_path_along_speed_err_pick_above, shared.dt_measured_s, {shared.ctrl_pa2_follow_tan_kp, shared.ctrl_pa2_follow_tan_ki, shared.ctrl_pa2_follow_tan_kd, shared.ctrl_pa2_follow_tan_decay_rate});
        shared.eacc_arm2_approach_path_along_speed_pick_above = _control_signal;
        shared.ctrl_pa2_follow_tan_error_integral = state.ctrl_pa2_follow_tan.error_integral();
        shared.ctrl_pa2_follow_tan_previous_error = state.ctrl_pa2_follow_tan.previous_error();
        shared.ctrl_pa2_follow_tan_first_sample = state.ctrl_pa2_follow_tan.is_first_sample();
    }
    // ctrl_pa1_follow_ori_ang_z
    {
        const double _control_signal = state.ctrl_pa1_follow_ori_ang_z.control(shared.pose_diff_ctrl_pa1_follow_ori.rot[2], -(shared.twist_ee1_base.rot[2]), shared.dt_measured_s, {shared.ctrl_pa1_follow_ori_ang_z_kp, shared.ctrl_pa1_follow_ori_ang_z_ki, shared.ctrl_pa1_follow_ori_ang_z_kd, shared.ctrl_pa1_follow_ori_ang_z_decay_rate});
        shared.eacc_ctrl_pa1_follow_ori_ang_z = _control_signal;
        shared.ctrl_pa1_follow_ori_ang_z_error_integral = state.ctrl_pa1_follow_ori_ang_z.error_integral();
        shared.ctrl_pa1_follow_ori_ang_z_previous_error = state.ctrl_pa1_follow_ori_ang_z.previous_error();
        shared.ctrl_pa1_follow_ori_ang_z_first_sample = state.ctrl_pa1_follow_ori_ang_z.is_first_sample();
    }
    // ctrl_pa1_follow_ori_ang_y
    {
        const double _control_signal = state.ctrl_pa1_follow_ori_ang_y.control(shared.pose_diff_ctrl_pa1_follow_ori.rot[1], -(shared.twist_ee1_base.rot[1]), shared.dt_measured_s, {shared.ctrl_pa1_follow_ori_ang_y_kp, shared.ctrl_pa1_follow_ori_ang_y_ki, shared.ctrl_pa1_follow_ori_ang_y_kd, shared.ctrl_pa1_follow_ori_ang_y_decay_rate});
        shared.eacc_ctrl_pa1_follow_ori_ang_y = _control_signal;
        shared.ctrl_pa1_follow_ori_ang_y_error_integral = state.ctrl_pa1_follow_ori_ang_y.error_integral();
        shared.ctrl_pa1_follow_ori_ang_y_previous_error = state.ctrl_pa1_follow_ori_ang_y.previous_error();
        shared.ctrl_pa1_follow_ori_ang_y_first_sample = state.ctrl_pa1_follow_ori_ang_y.is_first_sample();
    }
    // ctrl_pa1_follow_ori_ang_x
    {
        const double _control_signal = state.ctrl_pa1_follow_ori_ang_x.control(shared.pose_diff_ctrl_pa1_follow_ori.rot[0], -(shared.twist_ee1_base.rot[0]), shared.dt_measured_s, {shared.ctrl_pa1_follow_ori_ang_x_kp, shared.ctrl_pa1_follow_ori_ang_x_ki, shared.ctrl_pa1_follow_ori_ang_x_kd, shared.ctrl_pa1_follow_ori_ang_x_decay_rate});
        shared.eacc_ctrl_pa1_follow_ori_ang_x = _control_signal;
        shared.ctrl_pa1_follow_ori_ang_x_error_integral = state.ctrl_pa1_follow_ori_ang_x.error_integral();
        shared.ctrl_pa1_follow_ori_ang_x_previous_error = state.ctrl_pa1_follow_ori_ang_x.previous_error();
        shared.ctrl_pa1_follow_ori_ang_x_first_sample = state.ctrl_pa1_follow_ori_ang_x.is_first_sample();
    }
    // ctrl_pa1_follow_lat_lin_normal_b
    {
        const double _control_signal = state.ctrl_pa1_follow_lat_lin_normal_b.control(KDL::dot(shared.pose_diff_ctrl_pa1_follow_lat.vel, shared.arm1_approach_path_normal_b), -(KDL::dot(shared.twist_ee1_base.vel, shared.arm1_approach_path_normal_b)), shared.dt_measured_s, {shared.ctrl_pa1_follow_lat_lin_normal_b_kp, shared.ctrl_pa1_follow_lat_lin_normal_b_ki, shared.ctrl_pa1_follow_lat_lin_normal_b_kd, shared.ctrl_pa1_follow_lat_lin_normal_b_decay_rate});
        shared.eacc_ctrl_pa1_follow_lat_lin_normal_b = _control_signal;
        shared.ctrl_pa1_follow_lat_lin_normal_b_error_integral = state.ctrl_pa1_follow_lat_lin_normal_b.error_integral();
        shared.ctrl_pa1_follow_lat_lin_normal_b_previous_error = state.ctrl_pa1_follow_lat_lin_normal_b.previous_error();
        shared.ctrl_pa1_follow_lat_lin_normal_b_first_sample = state.ctrl_pa1_follow_lat_lin_normal_b.is_first_sample();
    }
    // ctrl_pa1_follow_lat_lin_normal_a
    {
        const double _control_signal = state.ctrl_pa1_follow_lat_lin_normal_a.control(KDL::dot(shared.pose_diff_ctrl_pa1_follow_lat.vel, shared.arm1_approach_path_normal_a), -(KDL::dot(shared.twist_ee1_base.vel, shared.arm1_approach_path_normal_a)), shared.dt_measured_s, {shared.ctrl_pa1_follow_lat_lin_normal_a_kp, shared.ctrl_pa1_follow_lat_lin_normal_a_ki, shared.ctrl_pa1_follow_lat_lin_normal_a_kd, shared.ctrl_pa1_follow_lat_lin_normal_a_decay_rate});
        shared.eacc_ctrl_pa1_follow_lat_lin_normal_a = _control_signal;
        shared.ctrl_pa1_follow_lat_lin_normal_a_error_integral = state.ctrl_pa1_follow_lat_lin_normal_a.error_integral();
        shared.ctrl_pa1_follow_lat_lin_normal_a_previous_error = state.ctrl_pa1_follow_lat_lin_normal_a.previous_error();
        shared.ctrl_pa1_follow_lat_lin_normal_a_first_sample = state.ctrl_pa1_follow_lat_lin_normal_a.is_first_sample();
    }
    // ctrl_pa1_follow_tan
    {
        const double _control_signal = state.ctrl_pa1_follow_tan.control(shared.arm1_approach_path_along_speed_err_pick_above, shared.dt_measured_s, {shared.ctrl_pa1_follow_tan_kp, shared.ctrl_pa1_follow_tan_ki, shared.ctrl_pa1_follow_tan_kd, shared.ctrl_pa1_follow_tan_decay_rate});
        shared.eacc_arm1_approach_path_along_speed_pick_above = _control_signal;
        shared.ctrl_pa1_follow_tan_error_integral = state.ctrl_pa1_follow_tan.error_integral();
        shared.ctrl_pa1_follow_tan_previous_error = state.ctrl_pa1_follow_tan.previous_error();
        shared.ctrl_pa1_follow_tan_first_sample = state.ctrl_pa1_follow_tan.is_first_sample();
    }

    motion_spec::runtime::set_flag(state.mon_pa1_advancing_pa1_advancing, motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_while_advance1_err, 0.0));

    motion_spec::runtime::set_flag(state.mon_pa2_advancing_pa2_advancing, motion_spec::runtime::constraint_satisfied(shared.eval_pick_above_while_advance2_err, 0.0));

    KDL::SetToZero(state.arm1_solver_pick_above.spatial_directions);

    state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = shared.arm1_approach_path_tangent.x();
    state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = shared.arm1_approach_path_tangent.y();
    state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = shared.arm1_approach_path_tangent.z();

    state.arm1_solver_pick_above.acceleration_energy(0) = shared.eacc_arm1_approach_path_along_speed_pick_above;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_above_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_above_1(*robot.arm1_solver_pick_above.chain);
        alpha_fk_arm1_solver_pick_above_1.JntToCart(
            state.arm1_solver_pick_above.q,
            alpha_frame_arm1_solver_pick_above_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_above_1 =
            alpha_frame_arm1_solver_pick_above_1.M * shared.arm1_approach_path_normal_a;
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_pick_above_1[0];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_pick_above_1[1];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_pick_above_1[2];
    }

    state.arm1_solver_pick_above.acceleration_energy(1) = shared.eacc_ctrl_pa1_follow_lat_lin_normal_a;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_above_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_above_2(*robot.arm1_solver_pick_above.chain);
        alpha_fk_arm1_solver_pick_above_2.JntToCart(
            state.arm1_solver_pick_above.q,
            alpha_frame_arm1_solver_pick_above_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_above_2 =
            alpha_frame_arm1_solver_pick_above_2.M * shared.arm1_approach_path_normal_b;
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_pick_above_2[0];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_pick_above_2[1];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_pick_above_2[2];
    }

    state.arm1_solver_pick_above.acceleration_energy(2) = shared.eacc_ctrl_pa1_follow_lat_lin_normal_b;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_above_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_above_3(*robot.arm1_solver_pick_above.chain);
        alpha_fk_arm1_solver_pick_above_3.JntToCart(
            state.arm1_solver_pick_above.q,
            alpha_frame_arm1_solver_pick_above_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_above_3 =
            alpha_frame_arm1_solver_pick_above_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_pick_above_3[0];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_pick_above_3[1];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_pick_above_3[2];
    }

    state.arm1_solver_pick_above.acceleration_energy(3) = shared.eacc_ctrl_pa1_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_above_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_above_4(*robot.arm1_solver_pick_above.chain);
        alpha_fk_arm1_solver_pick_above_4.JntToCart(
            state.arm1_solver_pick_above.q,
            alpha_frame_arm1_solver_pick_above_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_above_4 =
            alpha_frame_arm1_solver_pick_above_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_pick_above_4[0];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_pick_above_4[1];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_pick_above_4[2];
    }

    state.arm1_solver_pick_above.acceleration_energy(4) = shared.eacc_ctrl_pa1_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_pick_above_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_pick_above_5(*robot.arm1_solver_pick_above.chain);
        alpha_fk_arm1_solver_pick_above_5.JntToCart(
            state.arm1_solver_pick_above.q,
            alpha_frame_arm1_solver_pick_above_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_pick_above.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_pick_above_5 =
            alpha_frame_arm1_solver_pick_above_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_pick_above_5[0];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_pick_above_5[1];
        state.arm1_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_pick_above_5[2];
    }

    state.arm1_solver_pick_above.acceleration_energy(5) = shared.eacc_ctrl_pa1_follow_ori_ang_z;

    KDL::SetToZero(state.arm1_solver_pick_above.tau_ff);

    KDL::Wrenches f_ext_zero_arm1_solver_pick_above(state.arm1_solver_pick_above.num_segments);
    for (int i = 0; i < state.arm1_solver_pick_above.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_pick_above[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_pick_above(state.arm1_solver_pick_above.num_joints);
    state.arm1_solver_pick_above.achd_acc->CartToJnt(
        state.arm1_solver_pick_above.q,
        state.arm1_solver_pick_above.qd,
        state.arm1_solver_pick_above.qdd,
        state.arm1_solver_pick_above.spatial_directions,
        state.arm1_solver_pick_above.acceleration_energy,
        f_ext_zero_arm1_solver_pick_above,
        state.arm1_solver_pick_above.tau_ff,
        tau_ctrl_acc_arm1_solver_pick_above);
    state.arm1_solver_pick_above.rnea->CartToJnt(
        state.arm1_solver_pick_above.q,
        state.arm1_solver_pick_above.qd,
        state.arm1_solver_pick_above.qdd,
        f_ext_zero_arm1_solver_pick_above,
        state.arm1_solver_pick_above.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_pick_above.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_pick_above.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_pick_above.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_pick_above.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_pick_above.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_pick_above.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_pick_above.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_pick_above.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_pick_above.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_pick_above.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_pick_above.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_pick_above.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_pick_above.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_pick_above.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_pick_above.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_pick_above.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_pick_above.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_pick_above.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_pick_above.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_pick_above.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_pick_above.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_pick_above.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_pick_above.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_pick_above.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_pick_above.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_pick_above.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_pick_above.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_pick_above.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_pick_above.spatial_directions);

    state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = shared.arm2_approach_path_tangent.x();
    state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = shared.arm2_approach_path_tangent.y();
    state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = shared.arm2_approach_path_tangent.z();

    state.arm2_solver_pick_above.acceleration_energy(0) = shared.eacc_arm2_approach_path_along_speed_pick_above;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_above_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_above_1(*robot.arm2_solver_pick_above.chain);
        alpha_fk_arm2_solver_pick_above_1.JntToCart(
            state.arm2_solver_pick_above.q,
            alpha_frame_arm2_solver_pick_above_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_above_1 =
            alpha_frame_arm2_solver_pick_above_1.M * shared.arm2_approach_path_normal_a;
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_pick_above_1[0];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_pick_above_1[1];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_pick_above_1[2];
    }

    state.arm2_solver_pick_above.acceleration_energy(1) = shared.eacc_ctrl_pa2_follow_lat_lin_normal_a;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_above_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_above_2(*robot.arm2_solver_pick_above.chain);
        alpha_fk_arm2_solver_pick_above_2.JntToCart(
            state.arm2_solver_pick_above.q,
            alpha_frame_arm2_solver_pick_above_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_above_2 =
            alpha_frame_arm2_solver_pick_above_2.M * shared.arm2_approach_path_normal_b;
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_pick_above_2[0];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_pick_above_2[1];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_pick_above_2[2];
    }

    state.arm2_solver_pick_above.acceleration_energy(2) = shared.eacc_ctrl_pa2_follow_lat_lin_normal_b;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_above_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_above_3(*robot.arm2_solver_pick_above.chain);
        alpha_fk_arm2_solver_pick_above_3.JntToCart(
            state.arm2_solver_pick_above.q,
            alpha_frame_arm2_solver_pick_above_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_above_3 =
            alpha_frame_arm2_solver_pick_above_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_pick_above_3[0];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_pick_above_3[1];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_pick_above_3[2];
    }

    state.arm2_solver_pick_above.acceleration_energy(3) = shared.eacc_ctrl_pa2_follow_ori_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_above_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_above_4(*robot.arm2_solver_pick_above.chain);
        alpha_fk_arm2_solver_pick_above_4.JntToCart(
            state.arm2_solver_pick_above.q,
            alpha_frame_arm2_solver_pick_above_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_above_4 =
            alpha_frame_arm2_solver_pick_above_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_pick_above_4[0];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_pick_above_4[1];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_pick_above_4[2];
    }

    state.arm2_solver_pick_above.acceleration_energy(4) = shared.eacc_ctrl_pa2_follow_ori_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_pick_above_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_pick_above_5(*robot.arm2_solver_pick_above.chain);
        alpha_fk_arm2_solver_pick_above_5.JntToCart(
            state.arm2_solver_pick_above.q,
            alpha_frame_arm2_solver_pick_above_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_pick_above.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_pick_above_5 =
            alpha_frame_arm2_solver_pick_above_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_pick_above_5[0];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_pick_above_5[1];
        state.arm2_solver_pick_above.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_pick_above_5[2];
    }

    state.arm2_solver_pick_above.acceleration_energy(5) = shared.eacc_ctrl_pa2_follow_ori_ang_z;

    KDL::SetToZero(state.arm2_solver_pick_above.tau_ff);

    KDL::Wrenches f_ext_zero_arm2_solver_pick_above(state.arm2_solver_pick_above.num_segments);
    for (int i = 0; i < state.arm2_solver_pick_above.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_pick_above[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_pick_above(state.arm2_solver_pick_above.num_joints);
    state.arm2_solver_pick_above.achd_acc->CartToJnt(
        state.arm2_solver_pick_above.q,
        state.arm2_solver_pick_above.qd,
        state.arm2_solver_pick_above.qdd,
        state.arm2_solver_pick_above.spatial_directions,
        state.arm2_solver_pick_above.acceleration_energy,
        f_ext_zero_arm2_solver_pick_above,
        state.arm2_solver_pick_above.tau_ff,
        tau_ctrl_acc_arm2_solver_pick_above);
    state.arm2_solver_pick_above.rnea->CartToJnt(
        state.arm2_solver_pick_above.q,
        state.arm2_solver_pick_above.qd,
        state.arm2_solver_pick_above.qdd,
        f_ext_zero_arm2_solver_pick_above,
        state.arm2_solver_pick_above.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_pick_above.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_pick_above.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_pick_above.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_pick_above.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_pick_above.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_pick_above.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_pick_above.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_pick_above.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_pick_above.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_pick_above.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_pick_above.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_pick_above.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_pick_above.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_pick_above.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_pick_above.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_pick_above.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_pick_above.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_pick_above.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_pick_above.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_pick_above.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_pick_above.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_pick_above.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_pick_above.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_pick_above.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_pick_above.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_pick_above.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_pick_above.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_pick_above.tau_ctrl(6);

}

inline void apply_motion_pick_above(
    motion_pick_above_state &state,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_pick_above.num_joints; ++i) {
        robot.arm1_solver_pick_above.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_pick_above.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_pick_above.num_joints; ++i) {
        robot.arm2_solver_pick_above.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_pick_above.tau_ctrl(i), i);
    }

}
