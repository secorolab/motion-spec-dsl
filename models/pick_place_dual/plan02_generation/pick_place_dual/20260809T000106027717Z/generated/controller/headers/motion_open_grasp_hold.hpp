/// Motion: open-grasp-hold
/// Hold both TCPs at place pose while the grippers open
#pragma once

#include "controllers.hpp"
#include "shared_state.hpp"

struct motion_open_grasp_hold_state {
    bool active = false;
    int active_steps = 0;
    arm1_solver_open_grasp_hold_solver_state arm1_solver_open_grasp_hold;
    arm2_solver_open_grasp_hold_solver_state arm2_solver_open_grasp_hold;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_og1_hold_x;
    motion_spec::runtime::PIDControl ctrl_og1_hold_y;
    motion_spec::runtime::PIDControl ctrl_og1_hold_z;
    motion_spec::runtime::PIDControl ctrl_og1_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_og1_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_og1_hold_orientation_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_og1_support_z;

    motion_spec::runtime::PIDControl ctrl_og2_hold_x;
    motion_spec::runtime::PIDControl ctrl_og2_hold_y;
    motion_spec::runtime::PIDControl ctrl_og2_hold_z;
    motion_spec::runtime::PIDControl ctrl_og2_hold_orientation_ang_x;
    motion_spec::runtime::PIDControl ctrl_og2_hold_orientation_ang_y;
    motion_spec::runtime::PIDControl ctrl_og2_hold_orientation_ang_z;
    motion_spec::runtime::ImpedanceControl ctrl_og2_support_z;

    bool mon_open_ready_previous = false;
    bool mon_open_ready_event_triggered = false;

};

inline void reset_motion_open_grasp_hold(motion_open_grasp_hold_state &state) {
    state = motion_open_grasp_hold_state{};
}

inline void init_motion_open_grasp_hold(motion_open_grasp_hold_state &state, const robot_io &robot) {
    if (!state.arm1_solver_open_grasp_hold.initialized) {
        state.arm1_solver_open_grasp_hold.num_joints = robot.arm1_solver_open_grasp_hold.chain->getNrOfJoints();
        state.arm1_solver_open_grasp_hold.num_segments = robot.arm1_solver_open_grasp_hold.chain->getNrOfSegments();
        state.arm1_solver_open_grasp_hold.q = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_joints);
        state.arm1_solver_open_grasp_hold.qd = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_joints);
        state.arm1_solver_open_grasp_hold.qdd = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_joints);
        state.arm1_solver_open_grasp_hold.tau_ff = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_joints);
        state.arm1_solver_open_grasp_hold.tau_ctrl = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_joints);
        state.arm1_solver_open_grasp_hold.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm1_solver_open_grasp_hold.num_spatial_directions = 6;
        state.arm1_solver_open_grasp_hold.spatial_directions = KDL::Jacobian(state.arm1_solver_open_grasp_hold.num_spatial_directions);
        state.arm1_solver_open_grasp_hold.acceleration_energy = KDL::JntArray(state.arm1_solver_open_grasp_hold.num_spatial_directions);
        state.arm1_solver_open_grasp_hold.f_ext = KDL::Wrenches(state.arm1_solver_open_grasp_hold.num_segments);
        state.arm1_solver_open_grasp_hold.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm1_solver_open_grasp_hold.chain, state.arm1_solver_open_grasp_hold.root_acc, state.arm1_solver_open_grasp_hold.num_spatial_directions);
        state.arm1_solver_open_grasp_hold.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm1_solver_open_grasp_hold.chain, state.arm1_solver_open_grasp_hold.root_acc, state.arm1_solver_open_grasp_hold.num_spatial_directions);
        state.arm1_solver_open_grasp_hold.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm1_solver_open_grasp_hold.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm1_solver_open_grasp_hold.initialized = true;
    }
    if (!state.arm2_solver_open_grasp_hold.initialized) {
        state.arm2_solver_open_grasp_hold.num_joints = robot.arm2_solver_open_grasp_hold.chain->getNrOfJoints();
        state.arm2_solver_open_grasp_hold.num_segments = robot.arm2_solver_open_grasp_hold.chain->getNrOfSegments();
        state.arm2_solver_open_grasp_hold.q = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_joints);
        state.arm2_solver_open_grasp_hold.qd = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_joints);
        state.arm2_solver_open_grasp_hold.qdd = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_joints);
        state.arm2_solver_open_grasp_hold.tau_ff = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_joints);
        state.arm2_solver_open_grasp_hold.tau_ctrl = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_joints);
        state.arm2_solver_open_grasp_hold.root_acc.vel = KDL::Vector(0.0, 0.0, 9.81);
        state.arm2_solver_open_grasp_hold.num_spatial_directions = 6;
        state.arm2_solver_open_grasp_hold.spatial_directions = KDL::Jacobian(state.arm2_solver_open_grasp_hold.num_spatial_directions);
        state.arm2_solver_open_grasp_hold.acceleration_energy = KDL::JntArray(state.arm2_solver_open_grasp_hold.num_spatial_directions);
        state.arm2_solver_open_grasp_hold.f_ext = KDL::Wrenches(state.arm2_solver_open_grasp_hold.num_segments);
        state.arm2_solver_open_grasp_hold.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm2_solver_open_grasp_hold.chain, state.arm2_solver_open_grasp_hold.root_acc, state.arm2_solver_open_grasp_hold.num_spatial_directions);
        state.arm2_solver_open_grasp_hold.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm2_solver_open_grasp_hold.chain, state.arm2_solver_open_grasp_hold.root_acc, state.arm2_solver_open_grasp_hold.num_spatial_directions);
        state.arm2_solver_open_grasp_hold.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm2_solver_open_grasp_hold.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm2_solver_open_grasp_hold.initialized = true;
    }
}

inline void update_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_open_grasp_hold(state, robot);

    mj_kdl::update(robot.arm1_solver_open_grasp_hold.robot);
    for (int i = 0; i < state.arm1_solver_open_grasp_hold.num_joints; ++i) {
        state.arm1_solver_open_grasp_hold.q(i) = robot.arm1_solver_open_grasp_hold.robot->jnt_pos_msr[i];
        state.arm1_solver_open_grasp_hold.qd(i) = robot.arm1_solver_open_grasp_hold.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm1_solver_open_grasp_hold(state.arm1_solver_open_grasp_hold.q, state.arm1_solver_open_grasp_hold.qd);
    {
        KDL::Frame _body_frame_pose_cube1_base;
        if (!mj_kdl::get_body_frame(
                robot.arm1_solver_open_grasp_hold.robot->model,
                robot.arm1_solver_open_grasp_hold.robot->data,
                "cube",
                &_body_frame_pose_cube1_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm1_solver_open_grasp_hold;
        mj_kdl::get_body_frame(
                robot.arm1_solver_open_grasp_hold.robot->model,
                robot.arm1_solver_open_grasp_hold.robot->data,
                "kinova1_base_link",
                &_base_world_frame_arm1_solver_open_grasp_hold);
        shared.pose_cube1_base = _base_world_frame_arm1_solver_open_grasp_hold.Inverse() * _body_frame_pose_cube1_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            shared.pose_ee1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "g_pinch", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm1_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            shared.pose_elbow1_base,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "half_arm_2_link", "kinova1_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm1_solver_open_grasp_hold.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm1_solver_open_grasp_hold,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "g_pinch", "kinova1_base_link"));
        shared.twist_ee1_base = tmp.deriv();
    }

    {
        double _joint_position_gripper1_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm1_solver_open_grasp_hold.robot->model,
                robot.arm1_solver_open_grasp_hold.robot->data,
                "kinova1_g_left_driver_joint",
                &_joint_position_gripper1_pos)) {
            shared.gripper1_pos = _joint_position_gripper1_pos;
        } else {
            shared.gripper1_pos = state.arm1_solver_open_grasp_hold.q(motion_spec::runtime::find_joint_index(*robot.arm1_solver_open_grasp_hold.chain, "kinova1_g_left_driver_joint"));
        }
    }

    mj_kdl::update(robot.arm2_solver_open_grasp_hold.robot);
    for (int i = 0; i < state.arm2_solver_open_grasp_hold.num_joints; ++i) {
        state.arm2_solver_open_grasp_hold.q(i) = robot.arm2_solver_open_grasp_hold.robot->jnt_pos_msr[i];
        state.arm2_solver_open_grasp_hold.qd(i) = robot.arm2_solver_open_grasp_hold.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm2_solver_open_grasp_hold(state.arm2_solver_open_grasp_hold.q, state.arm2_solver_open_grasp_hold.qd);
    {
        KDL::Frame _body_frame_pose_cube2_base;
        if (!mj_kdl::get_body_frame(
                robot.arm2_solver_open_grasp_hold.robot->model,
                robot.arm2_solver_open_grasp_hold.robot->data,
                "cube2",
                &_body_frame_pose_cube2_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube2");
        }
        KDL::Frame _base_world_frame_arm2_solver_open_grasp_hold;
        mj_kdl::get_body_frame(
                robot.arm2_solver_open_grasp_hold.robot->model,
                robot.arm2_solver_open_grasp_hold.robot->data,
                "kinova2_base_link",
                &_base_world_frame_arm2_solver_open_grasp_hold);
        shared.pose_cube2_base = _base_world_frame_arm2_solver_open_grasp_hold.Inverse() * _body_frame_pose_cube2_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            shared.pose_ee2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "g_pinch", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm2_solver_open_grasp_hold.chain);
        fk.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            shared.pose_elbow2_base,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "half_arm_2_link", "kinova2_base_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm2_solver_open_grasp_hold.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm2_solver_open_grasp_hold,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "g_pinch", "kinova2_base_link"));
        shared.twist_ee2_base = tmp.deriv();
    }

    {
        double _joint_position_gripper2_pos = 0.0;
        if (mj_kdl::get_joint_position(
                robot.arm2_solver_open_grasp_hold.robot->model,
                robot.arm2_solver_open_grasp_hold.robot->data,
                "kinova2_g_left_driver_joint",
                &_joint_position_gripper2_pos)) {
            shared.gripper2_pos = _joint_position_gripper2_pos;
        } else {
            shared.gripper2_pos = state.arm2_solver_open_grasp_hold.q(motion_spec::runtime::find_joint_index(*robot.arm2_solver_open_grasp_hold.chain, "kinova2_g_left_driver_joint"));
        }
    }

    if (!state.snapshot_taken) {
        shared.open_grasp_hold_hold1_orientation_pose = shared.pose_ee1_base;
        shared.open_grasp_hold_hold2_orientation_pose = shared.pose_ee2_base;

        shared.open_grasp_hold_support1_z_add_out = shared.pose_elbow1_base.p[2] + shared.open_grasp_hold_support_lift;
        shared.open_grasp_hold_support1_z = shared.open_grasp_hold_support1_z_add_out;

        shared.open_grasp_hold_support2_z_add_out = shared.pose_elbow2_base.p[2] + shared.open_grasp_hold_support_lift;
        shared.open_grasp_hold_support2_z = shared.open_grasp_hold_support2_z_add_out;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_open_grasp_hold(
    shared_data &shared
) {
    // eval_open_grasp_hold_when_at1_place
    shared.eval_open_grasp_hold_when_at1_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee1_base.p[2]);
    // eval_open_grasp_hold_when_at2_place
    shared.eval_open_grasp_hold_when_at2_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee2_base.p[2]);

    return (motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at1_place_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at2_place_err, shared.satisfied_band));
}

inline void monitor_when_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_open_grasp_hold_when_at1_place
    shared.eval_open_grasp_hold_when_at1_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee1_base.p[2]);
    // eval_open_grasp_hold_when_at2_place
    shared.eval_open_grasp_hold_when_at2_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee2_base.p[2]);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at1_place_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at2_place_err, shared.satisfied_band));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_open_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_OPEN_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_OPEN_READY] << std::endl;
        }
    }

}

inline void monitor_until_motion_open_grasp_hold() {
}

inline void monitor_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot,
    motion_spec_event_buffer &events
) {
    // eval_open_grasp_hold_when_at1_place
    shared.eval_open_grasp_hold_when_at1_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee1_base.p[2]);
    // eval_open_grasp_hold_when_at2_place
    shared.eval_open_grasp_hold_when_at2_place_err = motion_spec::runtime::evaluate_equality_constraint(shared.release_z, shared.pose_ee2_base.p[2]);

    {
        const bool active = (motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at1_place_err, shared.satisfied_band) && motion_spec::runtime::constraint_satisfied(shared.eval_open_grasp_hold_when_at2_place_err, shared.satisfied_band));
        const bool detected = motion_spec::runtime::rising_edge(state.mon_open_ready_previous, active);
        // coord2b is the sequencer: fire the FSM event on the rising edge while in this state.
        if (detected) {
            events.record(5);
        }
        if (detected && robot.fsm_events) {
            produce_event(robot.fsm_events, pick_place_dual_fsm::E_OPEN_READY);
            std::cerr << "[fsm] event   " << pick_place_dual_fsm::EVENT_URIS[pick_place_dual_fsm::E_OPEN_READY] << std::endl;
        }
    }

}

inline void control_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee1_base = shared.pose_ee1_base;
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[0] = shared.open_grasp_hold_hold1_orientation_pose.p[0];
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[1] = shared.open_grasp_hold_hold1_orientation_pose.p[1];
        _pose_axis_target_pose_axis_error_pose_ee1_base.p[2] = shared.open_grasp_hold_hold1_orientation_pose.p[2];
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee1_base = KDL::diff(shared.pose_ee1_base, _pose_axis_target_pose_axis_error_pose_ee1_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee1_base.vel.z();

        shared.pose_ee1_base_distance_x_err_open_grasp_hold = _pose_axis_error_linear_X;
        shared.pose_ee1_base_distance_y_err_open_grasp_hold = _pose_axis_error_linear_Y;
        shared.pose_ee1_base_distance_z_err_open_grasp_hold = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee2_base = shared.pose_ee2_base;
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[0] = shared.open_grasp_hold_hold2_orientation_pose.p[0];
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[1] = shared.open_grasp_hold_hold2_orientation_pose.p[1];
        _pose_axis_target_pose_axis_error_pose_ee2_base.p[2] = shared.open_grasp_hold_hold2_orientation_pose.p[2];
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee2_base = KDL::diff(shared.pose_ee2_base, _pose_axis_target_pose_axis_error_pose_ee2_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee2_base.vel.z();

        shared.pose_ee2_base_distance_x_err_open_grasp_hold = _pose_axis_error_linear_X;
        shared.pose_ee2_base_distance_y_err_open_grasp_hold = _pose_axis_error_linear_Y;
        shared.pose_ee2_base_distance_z_err_open_grasp_hold = _pose_axis_error_linear_Z;
    }
    // eval_pose_diff_ctrl_og1_hold_orientation
    shared.pose_diff_ctrl_og1_hold_orientation = KDL::diff(shared.pose_ee1_base, shared.open_grasp_hold_hold1_orientation_pose);
    shared.ctrl_og1_hold_orientation_err_ang_x = shared.pose_diff_ctrl_og1_hold_orientation.rot[0];
    shared.ctrl_og1_hold_orientation_err_ang_y = shared.pose_diff_ctrl_og1_hold_orientation.rot[1];
    shared.ctrl_og1_hold_orientation_err_ang_z = shared.pose_diff_ctrl_og1_hold_orientation.rot[2];
    // eval_pose_diff_ctrl_og2_hold_orientation
    shared.pose_diff_ctrl_og2_hold_orientation = KDL::diff(shared.pose_ee2_base, shared.open_grasp_hold_hold2_orientation_pose);
    shared.ctrl_og2_hold_orientation_err_ang_x = shared.pose_diff_ctrl_og2_hold_orientation.rot[0];
    shared.ctrl_og2_hold_orientation_err_ang_y = shared.pose_diff_ctrl_og2_hold_orientation.rot[1];
    shared.ctrl_og2_hold_orientation_err_ang_z = shared.pose_diff_ctrl_og2_hold_orientation.rot[2];
    // eval_open_grasp_hold_while_support1_elbow_z
    shared.pose_elbow1_base_distance_z_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.open_grasp_hold_support1_z, shared.pose_elbow1_base.p[2]);
    // eval_open_grasp_hold_while_support2_elbow_z
    shared.pose_elbow2_base_distance_z_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.open_grasp_hold_support2_z, shared.pose_elbow2_base.p[2]);
    // compute_wrench_force_ctrl_og1_support_z
    shared.wrench_force_ctrl_og1_support_z = KDL::Wrench(shared.direction_ctrl_og1_support_z * shared.force_ctrl_og1_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_og1_support_z);
    // compute_wrench_force_ctrl_og2_support_z
    shared.wrench_force_ctrl_og2_support_z = KDL::Wrench(shared.direction_ctrl_og2_support_z * shared.force_ctrl_og2_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_og2_support_z);
    // eval_open_grasp_hold_while_open1_gripper
    shared.gripper1_pos_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper1_pos);
    // eval_open_grasp_hold_while_open2_gripper
    shared.gripper2_pos_err_open_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.gripper_open, shared.gripper2_pos);
    // ctrl_og2_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_og2_open_gripper = _control_signal;
    }
    // ctrl_og2_support_z
    {
        const double _control_signal = state.ctrl_og2_support_z.control(shared.pose_elbow2_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og2_support_z_stiffness, shared.ctrl_og2_support_z_damping, shared.ctrl_og2_support_z_integral_gain});
        shared.force_ctrl_og2_support_z = _control_signal;
        shared.ctrl_og2_support_z_error_integral = state.ctrl_og2_support_z.error_integral();
        shared.ctrl_og2_support_z_previous_error = state.ctrl_og2_support_z.previous_error();
        shared.ctrl_og2_support_z_first_sample = state.ctrl_og2_support_z.is_first_sample();
    }
    // ctrl_og2_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_og2_hold_orientation_ang_z.control(shared.pose_diff_ctrl_og2_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_og2_hold_orientation_ang_z_kp, shared.ctrl_og2_hold_orientation_ang_z_ki, shared.ctrl_og2_hold_orientation_ang_z_kd, shared.ctrl_og2_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_og2_hold_orientation_ang_z = _control_signal;
        shared.ctrl_og2_hold_orientation_ang_z_error_integral = state.ctrl_og2_hold_orientation_ang_z.error_integral();
        shared.ctrl_og2_hold_orientation_ang_z_previous_error = state.ctrl_og2_hold_orientation_ang_z.previous_error();
        shared.ctrl_og2_hold_orientation_ang_z_first_sample = state.ctrl_og2_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_og2_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_og2_hold_orientation_ang_y.control(shared.pose_diff_ctrl_og2_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_og2_hold_orientation_ang_y_kp, shared.ctrl_og2_hold_orientation_ang_y_ki, shared.ctrl_og2_hold_orientation_ang_y_kd, shared.ctrl_og2_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_og2_hold_orientation_ang_y = _control_signal;
        shared.ctrl_og2_hold_orientation_ang_y_error_integral = state.ctrl_og2_hold_orientation_ang_y.error_integral();
        shared.ctrl_og2_hold_orientation_ang_y_previous_error = state.ctrl_og2_hold_orientation_ang_y.previous_error();
        shared.ctrl_og2_hold_orientation_ang_y_first_sample = state.ctrl_og2_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_og2_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_og2_hold_orientation_ang_x.control(shared.pose_diff_ctrl_og2_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_og2_hold_orientation_ang_x_kp, shared.ctrl_og2_hold_orientation_ang_x_ki, shared.ctrl_og2_hold_orientation_ang_x_kd, shared.ctrl_og2_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_og2_hold_orientation_ang_x = _control_signal;
        shared.ctrl_og2_hold_orientation_ang_x_error_integral = state.ctrl_og2_hold_orientation_ang_x.error_integral();
        shared.ctrl_og2_hold_orientation_ang_x_previous_error = state.ctrl_og2_hold_orientation_ang_x.previous_error();
        shared.ctrl_og2_hold_orientation_ang_x_first_sample = state.ctrl_og2_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_og2_hold_z
    {
        const double _control_signal = state.ctrl_og2_hold_z.control(shared.pose_ee2_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og2_hold_z_kp, shared.ctrl_og2_hold_z_ki, shared.ctrl_og2_hold_z_kd, shared.ctrl_og2_hold_z_decay_rate});
        shared.eacc_pose_ee2_base_distance_z_open_grasp_hold = _control_signal;
        shared.ctrl_og2_hold_z_error_integral = state.ctrl_og2_hold_z.error_integral();
        shared.ctrl_og2_hold_z_previous_error = state.ctrl_og2_hold_z.previous_error();
        shared.ctrl_og2_hold_z_first_sample = state.ctrl_og2_hold_z.is_first_sample();
    }
    // ctrl_og2_hold_y
    {
        const double _control_signal = state.ctrl_og2_hold_y.control(shared.pose_ee2_base_distance_y_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og2_hold_y_kp, shared.ctrl_og2_hold_y_ki, shared.ctrl_og2_hold_y_kd, shared.ctrl_og2_hold_y_decay_rate});
        shared.eacc_pose_ee2_base_distance_y_open_grasp_hold = _control_signal;
        shared.ctrl_og2_hold_y_error_integral = state.ctrl_og2_hold_y.error_integral();
        shared.ctrl_og2_hold_y_previous_error = state.ctrl_og2_hold_y.previous_error();
        shared.ctrl_og2_hold_y_first_sample = state.ctrl_og2_hold_y.is_first_sample();
    }
    // ctrl_og2_hold_x
    {
        const double _control_signal = state.ctrl_og2_hold_x.control(shared.pose_ee2_base_distance_x_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og2_hold_x_kp, shared.ctrl_og2_hold_x_ki, shared.ctrl_og2_hold_x_kd, shared.ctrl_og2_hold_x_decay_rate});
        shared.eacc_pose_ee2_base_distance_x_open_grasp_hold = _control_signal;
        shared.ctrl_og2_hold_x_error_integral = state.ctrl_og2_hold_x.error_integral();
        shared.ctrl_og2_hold_x_previous_error = state.ctrl_og2_hold_x.previous_error();
        shared.ctrl_og2_hold_x_first_sample = state.ctrl_og2_hold_x.is_first_sample();
    }
    // ctrl_og1_open_gripper
    {
        const double _control_signal = shared.gripper_open;
        shared.cmd_ctrl_og1_open_gripper = _control_signal;
    }
    // ctrl_og1_support_z
    {
        const double _control_signal = state.ctrl_og1_support_z.control(shared.pose_elbow1_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og1_support_z_stiffness, shared.ctrl_og1_support_z_damping, shared.ctrl_og1_support_z_integral_gain});
        shared.force_ctrl_og1_support_z = _control_signal;
        shared.ctrl_og1_support_z_error_integral = state.ctrl_og1_support_z.error_integral();
        shared.ctrl_og1_support_z_previous_error = state.ctrl_og1_support_z.previous_error();
        shared.ctrl_og1_support_z_first_sample = state.ctrl_og1_support_z.is_first_sample();
    }
    // ctrl_og1_hold_orientation_ang_z
    {
        const double _control_signal = state.ctrl_og1_hold_orientation_ang_z.control(shared.pose_diff_ctrl_og1_hold_orientation.rot[2], shared.dt_measured_s, {shared.ctrl_og1_hold_orientation_ang_z_kp, shared.ctrl_og1_hold_orientation_ang_z_ki, shared.ctrl_og1_hold_orientation_ang_z_kd, shared.ctrl_og1_hold_orientation_ang_z_decay_rate});
        shared.eacc_ctrl_og1_hold_orientation_ang_z = _control_signal;
        shared.ctrl_og1_hold_orientation_ang_z_error_integral = state.ctrl_og1_hold_orientation_ang_z.error_integral();
        shared.ctrl_og1_hold_orientation_ang_z_previous_error = state.ctrl_og1_hold_orientation_ang_z.previous_error();
        shared.ctrl_og1_hold_orientation_ang_z_first_sample = state.ctrl_og1_hold_orientation_ang_z.is_first_sample();
    }
    // ctrl_og1_hold_orientation_ang_y
    {
        const double _control_signal = state.ctrl_og1_hold_orientation_ang_y.control(shared.pose_diff_ctrl_og1_hold_orientation.rot[1], shared.dt_measured_s, {shared.ctrl_og1_hold_orientation_ang_y_kp, shared.ctrl_og1_hold_orientation_ang_y_ki, shared.ctrl_og1_hold_orientation_ang_y_kd, shared.ctrl_og1_hold_orientation_ang_y_decay_rate});
        shared.eacc_ctrl_og1_hold_orientation_ang_y = _control_signal;
        shared.ctrl_og1_hold_orientation_ang_y_error_integral = state.ctrl_og1_hold_orientation_ang_y.error_integral();
        shared.ctrl_og1_hold_orientation_ang_y_previous_error = state.ctrl_og1_hold_orientation_ang_y.previous_error();
        shared.ctrl_og1_hold_orientation_ang_y_first_sample = state.ctrl_og1_hold_orientation_ang_y.is_first_sample();
    }
    // ctrl_og1_hold_orientation_ang_x
    {
        const double _control_signal = state.ctrl_og1_hold_orientation_ang_x.control(shared.pose_diff_ctrl_og1_hold_orientation.rot[0], shared.dt_measured_s, {shared.ctrl_og1_hold_orientation_ang_x_kp, shared.ctrl_og1_hold_orientation_ang_x_ki, shared.ctrl_og1_hold_orientation_ang_x_kd, shared.ctrl_og1_hold_orientation_ang_x_decay_rate});
        shared.eacc_ctrl_og1_hold_orientation_ang_x = _control_signal;
        shared.ctrl_og1_hold_orientation_ang_x_error_integral = state.ctrl_og1_hold_orientation_ang_x.error_integral();
        shared.ctrl_og1_hold_orientation_ang_x_previous_error = state.ctrl_og1_hold_orientation_ang_x.previous_error();
        shared.ctrl_og1_hold_orientation_ang_x_first_sample = state.ctrl_og1_hold_orientation_ang_x.is_first_sample();
    }
    // ctrl_og1_hold_z
    {
        const double _control_signal = state.ctrl_og1_hold_z.control(shared.pose_ee1_base_distance_z_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og1_hold_z_kp, shared.ctrl_og1_hold_z_ki, shared.ctrl_og1_hold_z_kd, shared.ctrl_og1_hold_z_decay_rate});
        shared.eacc_pose_ee1_base_distance_z_open_grasp_hold = _control_signal;
        shared.ctrl_og1_hold_z_error_integral = state.ctrl_og1_hold_z.error_integral();
        shared.ctrl_og1_hold_z_previous_error = state.ctrl_og1_hold_z.previous_error();
        shared.ctrl_og1_hold_z_first_sample = state.ctrl_og1_hold_z.is_first_sample();
    }
    // ctrl_og1_hold_y
    {
        const double _control_signal = state.ctrl_og1_hold_y.control(shared.pose_ee1_base_distance_y_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og1_hold_y_kp, shared.ctrl_og1_hold_y_ki, shared.ctrl_og1_hold_y_kd, shared.ctrl_og1_hold_y_decay_rate});
        shared.eacc_pose_ee1_base_distance_y_open_grasp_hold = _control_signal;
        shared.ctrl_og1_hold_y_error_integral = state.ctrl_og1_hold_y.error_integral();
        shared.ctrl_og1_hold_y_previous_error = state.ctrl_og1_hold_y.previous_error();
        shared.ctrl_og1_hold_y_first_sample = state.ctrl_og1_hold_y.is_first_sample();
    }
    // ctrl_og1_hold_x
    {
        const double _control_signal = state.ctrl_og1_hold_x.control(shared.pose_ee1_base_distance_x_err_open_grasp_hold, shared.dt_measured_s, {shared.ctrl_og1_hold_x_kp, shared.ctrl_og1_hold_x_ki, shared.ctrl_og1_hold_x_kd, shared.ctrl_og1_hold_x_decay_rate});
        shared.eacc_pose_ee1_base_distance_x_open_grasp_hold = _control_signal;
        shared.ctrl_og1_hold_x_error_integral = state.ctrl_og1_hold_x.error_integral();
        shared.ctrl_og1_hold_x_previous_error = state.ctrl_og1_hold_x.previous_error();
        shared.ctrl_og1_hold_x_first_sample = state.ctrl_og1_hold_x.is_first_sample();
    }

    KDL::SetToZero(state.arm1_solver_open_grasp_hold.spatial_directions);

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_0(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_0.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_0,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_0 =
            alpha_frame_arm1_solver_open_grasp_hold_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm1_solver_open_grasp_hold_0[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm1_solver_open_grasp_hold_0[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm1_solver_open_grasp_hold_0[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(0) = shared.eacc_pose_ee1_base_distance_x_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_1(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_1.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_1,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_1 =
            alpha_frame_arm1_solver_open_grasp_hold_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm1_solver_open_grasp_hold_1[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm1_solver_open_grasp_hold_1[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm1_solver_open_grasp_hold_1[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(1) = shared.eacc_pose_ee1_base_distance_y_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_2(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_2.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_2,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_2 =
            alpha_frame_arm1_solver_open_grasp_hold_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm1_solver_open_grasp_hold_2[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm1_solver_open_grasp_hold_2[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm1_solver_open_grasp_hold_2[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(2) = shared.eacc_pose_ee1_base_distance_z_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_3(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_3.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_3,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_3 =
            alpha_frame_arm1_solver_open_grasp_hold_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm1_solver_open_grasp_hold_3[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm1_solver_open_grasp_hold_3[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm1_solver_open_grasp_hold_3[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(3) = shared.eacc_ctrl_og1_hold_orientation_ang_x;

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_4(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_4.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_4,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_4 =
            alpha_frame_arm1_solver_open_grasp_hold_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm1_solver_open_grasp_hold_4[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm1_solver_open_grasp_hold_4[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm1_solver_open_grasp_hold_4[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(4) = shared.eacc_ctrl_og1_hold_orientation_ang_y;

    {
        KDL::Frame alpha_frame_arm1_solver_open_grasp_hold_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm1_solver_open_grasp_hold_5(*robot.arm1_solver_open_grasp_hold.chain);
        alpha_fk_arm1_solver_open_grasp_hold_5.JntToCart(
            state.arm1_solver_open_grasp_hold.q,
            alpha_frame_arm1_solver_open_grasp_hold_5,
            motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "base_link", "kinova1_base_link"));
        const KDL::Vector alpha_axis_arm1_solver_open_grasp_hold_5 =
            alpha_frame_arm1_solver_open_grasp_hold_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm1_solver_open_grasp_hold_5[0];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm1_solver_open_grasp_hold_5[1];
        state.arm1_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm1_solver_open_grasp_hold_5[2];
    }

    state.arm1_solver_open_grasp_hold.acceleration_energy(5) = shared.eacc_ctrl_og1_hold_orientation_ang_z;

    KDL::SetToZero(state.arm1_solver_open_grasp_hold.tau_ff);

    for (int i = 0; i < state.arm1_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(state.arm1_solver_open_grasp_hold.f_ext[i]);
    }

    state.arm1_solver_open_grasp_hold.f_ext[motion_spec::runtime::find_segment_index(*robot.arm1_solver_open_grasp_hold.chain, "half_arm_2_link", "kinova1_base_link") - 1] += shared.wrench_force_ctrl_og1_support_z;

    KDL::Wrenches f_ext_zero_arm1_solver_open_grasp_hold(state.arm1_solver_open_grasp_hold.num_segments);
    for (int i = 0; i < state.arm1_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm1_solver_open_grasp_hold[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm1_solver_open_grasp_hold(state.arm1_solver_open_grasp_hold.num_joints);
    state.arm1_solver_open_grasp_hold.achd_acc->CartToJnt(
        state.arm1_solver_open_grasp_hold.q,
        state.arm1_solver_open_grasp_hold.qd,
        state.arm1_solver_open_grasp_hold.qdd,
        state.arm1_solver_open_grasp_hold.spatial_directions,
        state.arm1_solver_open_grasp_hold.acceleration_energy,
        state.arm1_solver_open_grasp_hold.f_ext,
        state.arm1_solver_open_grasp_hold.tau_ff,
        tau_ctrl_acc_arm1_solver_open_grasp_hold);
    state.arm1_solver_open_grasp_hold.rnea->CartToJnt(
        state.arm1_solver_open_grasp_hold.q,
        state.arm1_solver_open_grasp_hold.qd,
        state.arm1_solver_open_grasp_hold.qdd,
        f_ext_zero_arm1_solver_open_grasp_hold,
        state.arm1_solver_open_grasp_hold.tau_ctrl);
    shared.arm1_solver_home_q_kinova1_joint_1 = state.arm1_solver_open_grasp_hold.q(0);
    shared.arm1_solver_home_q_kinova1_joint_2 = state.arm1_solver_open_grasp_hold.q(1);
    shared.arm1_solver_home_q_kinova1_joint_3 = state.arm1_solver_open_grasp_hold.q(2);
    shared.arm1_solver_home_q_kinova1_joint_4 = state.arm1_solver_open_grasp_hold.q(3);
    shared.arm1_solver_home_q_kinova1_joint_5 = state.arm1_solver_open_grasp_hold.q(4);
    shared.arm1_solver_home_q_kinova1_joint_6 = state.arm1_solver_open_grasp_hold.q(5);
    shared.arm1_solver_home_q_kinova1_joint_7 = state.arm1_solver_open_grasp_hold.q(6);
    shared.arm1_solver_home_qd_kinova1_joint_1 = state.arm1_solver_open_grasp_hold.qd(0);
    shared.arm1_solver_home_qd_kinova1_joint_2 = state.arm1_solver_open_grasp_hold.qd(1);
    shared.arm1_solver_home_qd_kinova1_joint_3 = state.arm1_solver_open_grasp_hold.qd(2);
    shared.arm1_solver_home_qd_kinova1_joint_4 = state.arm1_solver_open_grasp_hold.qd(3);
    shared.arm1_solver_home_qd_kinova1_joint_5 = state.arm1_solver_open_grasp_hold.qd(4);
    shared.arm1_solver_home_qd_kinova1_joint_6 = state.arm1_solver_open_grasp_hold.qd(5);
    shared.arm1_solver_home_qd_kinova1_joint_7 = state.arm1_solver_open_grasp_hold.qd(6);
    shared.arm1_solver_home_qdd_kinova1_joint_1 = state.arm1_solver_open_grasp_hold.qdd(0);
    shared.arm1_solver_home_qdd_kinova1_joint_2 = state.arm1_solver_open_grasp_hold.qdd(1);
    shared.arm1_solver_home_qdd_kinova1_joint_3 = state.arm1_solver_open_grasp_hold.qdd(2);
    shared.arm1_solver_home_qdd_kinova1_joint_4 = state.arm1_solver_open_grasp_hold.qdd(3);
    shared.arm1_solver_home_qdd_kinova1_joint_5 = state.arm1_solver_open_grasp_hold.qdd(4);
    shared.arm1_solver_home_qdd_kinova1_joint_6 = state.arm1_solver_open_grasp_hold.qdd(5);
    shared.arm1_solver_home_qdd_kinova1_joint_7 = state.arm1_solver_open_grasp_hold.qdd(6);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_1 = state.arm1_solver_open_grasp_hold.tau_ctrl(0);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_2 = state.arm1_solver_open_grasp_hold.tau_ctrl(1);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_3 = state.arm1_solver_open_grasp_hold.tau_ctrl(2);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_4 = state.arm1_solver_open_grasp_hold.tau_ctrl(3);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_5 = state.arm1_solver_open_grasp_hold.tau_ctrl(4);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_6 = state.arm1_solver_open_grasp_hold.tau_ctrl(5);
    shared.arm1_solver_home_tau_ctrl_kinova1_joint_7 = state.arm1_solver_open_grasp_hold.tau_ctrl(6);

    KDL::SetToZero(state.arm2_solver_open_grasp_hold.spatial_directions);

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_0(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_0.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_0,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_0 =
            alpha_frame_arm2_solver_open_grasp_hold_0.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm2_solver_open_grasp_hold_0[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm2_solver_open_grasp_hold_0[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm2_solver_open_grasp_hold_0[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(0) = shared.eacc_pose_ee2_base_distance_x_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_1(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_1.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_1,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_1 =
            alpha_frame_arm2_solver_open_grasp_hold_1.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm2_solver_open_grasp_hold_1[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm2_solver_open_grasp_hold_1[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm2_solver_open_grasp_hold_1[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(1) = shared.eacc_pose_ee2_base_distance_y_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_2(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_2.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_2,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_2 =
            alpha_frame_arm2_solver_open_grasp_hold_2.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm2_solver_open_grasp_hold_2[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm2_solver_open_grasp_hold_2[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm2_solver_open_grasp_hold_2[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(2) = shared.eacc_pose_ee2_base_distance_z_open_grasp_hold;

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_3(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_3.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_3,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_3 =
            alpha_frame_arm2_solver_open_grasp_hold_3.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = alpha_axis_arm2_solver_open_grasp_hold_3[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 3) = alpha_axis_arm2_solver_open_grasp_hold_3[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = alpha_axis_arm2_solver_open_grasp_hold_3[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(3) = shared.eacc_ctrl_og2_hold_orientation_ang_x;

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_4(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_4.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_4,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_4 =
            alpha_frame_arm2_solver_open_grasp_hold_4.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_arm2_solver_open_grasp_hold_4[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_arm2_solver_open_grasp_hold_4[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_arm2_solver_open_grasp_hold_4[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(4) = shared.eacc_ctrl_og2_hold_orientation_ang_y;

    {
        KDL::Frame alpha_frame_arm2_solver_open_grasp_hold_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm2_solver_open_grasp_hold_5(*robot.arm2_solver_open_grasp_hold.chain);
        alpha_fk_arm2_solver_open_grasp_hold_5.JntToCart(
            state.arm2_solver_open_grasp_hold.q,
            alpha_frame_arm2_solver_open_grasp_hold_5,
            motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "base_link", "kinova2_base_link"));
        const KDL::Vector alpha_axis_arm2_solver_open_grasp_hold_5 =
            alpha_frame_arm2_solver_open_grasp_hold_5.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_arm2_solver_open_grasp_hold_5[0];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_arm2_solver_open_grasp_hold_5[1];
        state.arm2_solver_open_grasp_hold.spatial_directions(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_arm2_solver_open_grasp_hold_5[2];
    }

    state.arm2_solver_open_grasp_hold.acceleration_energy(5) = shared.eacc_ctrl_og2_hold_orientation_ang_z;

    KDL::SetToZero(state.arm2_solver_open_grasp_hold.tau_ff);

    for (int i = 0; i < state.arm2_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(state.arm2_solver_open_grasp_hold.f_ext[i]);
    }

    state.arm2_solver_open_grasp_hold.f_ext[motion_spec::runtime::find_segment_index(*robot.arm2_solver_open_grasp_hold.chain, "half_arm_2_link", "kinova2_base_link") - 1] += shared.wrench_force_ctrl_og2_support_z;

    KDL::Wrenches f_ext_zero_arm2_solver_open_grasp_hold(state.arm2_solver_open_grasp_hold.num_segments);
    for (int i = 0; i < state.arm2_solver_open_grasp_hold.num_segments; ++i) {
        KDL::SetToZero(f_ext_zero_arm2_solver_open_grasp_hold[i]);
    }
    KDL::JntArray tau_ctrl_acc_arm2_solver_open_grasp_hold(state.arm2_solver_open_grasp_hold.num_joints);
    state.arm2_solver_open_grasp_hold.achd_acc->CartToJnt(
        state.arm2_solver_open_grasp_hold.q,
        state.arm2_solver_open_grasp_hold.qd,
        state.arm2_solver_open_grasp_hold.qdd,
        state.arm2_solver_open_grasp_hold.spatial_directions,
        state.arm2_solver_open_grasp_hold.acceleration_energy,
        state.arm2_solver_open_grasp_hold.f_ext,
        state.arm2_solver_open_grasp_hold.tau_ff,
        tau_ctrl_acc_arm2_solver_open_grasp_hold);
    state.arm2_solver_open_grasp_hold.rnea->CartToJnt(
        state.arm2_solver_open_grasp_hold.q,
        state.arm2_solver_open_grasp_hold.qd,
        state.arm2_solver_open_grasp_hold.qdd,
        f_ext_zero_arm2_solver_open_grasp_hold,
        state.arm2_solver_open_grasp_hold.tau_ctrl);
    shared.arm2_solver_home_q_kinova2_joint_1 = state.arm2_solver_open_grasp_hold.q(0);
    shared.arm2_solver_home_q_kinova2_joint_2 = state.arm2_solver_open_grasp_hold.q(1);
    shared.arm2_solver_home_q_kinova2_joint_3 = state.arm2_solver_open_grasp_hold.q(2);
    shared.arm2_solver_home_q_kinova2_joint_4 = state.arm2_solver_open_grasp_hold.q(3);
    shared.arm2_solver_home_q_kinova2_joint_5 = state.arm2_solver_open_grasp_hold.q(4);
    shared.arm2_solver_home_q_kinova2_joint_6 = state.arm2_solver_open_grasp_hold.q(5);
    shared.arm2_solver_home_q_kinova2_joint_7 = state.arm2_solver_open_grasp_hold.q(6);
    shared.arm2_solver_home_qd_kinova2_joint_1 = state.arm2_solver_open_grasp_hold.qd(0);
    shared.arm2_solver_home_qd_kinova2_joint_2 = state.arm2_solver_open_grasp_hold.qd(1);
    shared.arm2_solver_home_qd_kinova2_joint_3 = state.arm2_solver_open_grasp_hold.qd(2);
    shared.arm2_solver_home_qd_kinova2_joint_4 = state.arm2_solver_open_grasp_hold.qd(3);
    shared.arm2_solver_home_qd_kinova2_joint_5 = state.arm2_solver_open_grasp_hold.qd(4);
    shared.arm2_solver_home_qd_kinova2_joint_6 = state.arm2_solver_open_grasp_hold.qd(5);
    shared.arm2_solver_home_qd_kinova2_joint_7 = state.arm2_solver_open_grasp_hold.qd(6);
    shared.arm2_solver_home_qdd_kinova2_joint_1 = state.arm2_solver_open_grasp_hold.qdd(0);
    shared.arm2_solver_home_qdd_kinova2_joint_2 = state.arm2_solver_open_grasp_hold.qdd(1);
    shared.arm2_solver_home_qdd_kinova2_joint_3 = state.arm2_solver_open_grasp_hold.qdd(2);
    shared.arm2_solver_home_qdd_kinova2_joint_4 = state.arm2_solver_open_grasp_hold.qdd(3);
    shared.arm2_solver_home_qdd_kinova2_joint_5 = state.arm2_solver_open_grasp_hold.qdd(4);
    shared.arm2_solver_home_qdd_kinova2_joint_6 = state.arm2_solver_open_grasp_hold.qdd(5);
    shared.arm2_solver_home_qdd_kinova2_joint_7 = state.arm2_solver_open_grasp_hold.qdd(6);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_1 = state.arm2_solver_open_grasp_hold.tau_ctrl(0);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_2 = state.arm2_solver_open_grasp_hold.tau_ctrl(1);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_3 = state.arm2_solver_open_grasp_hold.tau_ctrl(2);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_4 = state.arm2_solver_open_grasp_hold.tau_ctrl(3);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_5 = state.arm2_solver_open_grasp_hold.tau_ctrl(4);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_6 = state.arm2_solver_open_grasp_hold.tau_ctrl(5);
    shared.arm2_solver_home_tau_ctrl_kinova2_joint_7 = state.arm2_solver_open_grasp_hold.tau_ctrl(6);

}

inline void apply_motion_open_grasp_hold(
    motion_open_grasp_hold_state &state,
    shared_data &shared,
    const robot_io &robot
) {

    for (int i = 0; i < state.arm1_solver_open_grasp_hold.num_joints; ++i) {
        robot.arm1_solver_open_grasp_hold.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm1_solver_open_grasp_hold.tau_ctrl(i), i);
    }

    for (int i = 0; i < state.arm2_solver_open_grasp_hold.num_joints; ++i) {
        robot.arm2_solver_open_grasp_hold.robot->jnt_trq_cmd[i] = motion_spec::runtime::finite_or_stop(state.arm2_solver_open_grasp_hold.tau_ctrl(i), i);
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
            robot.arm1_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_og1_open_gripper;
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
            robot.arm2_solver_home.robot->data->ctrl[actuator_id] = shared.cmd_ctrl_og2_open_gripper;
        }
    }

}
