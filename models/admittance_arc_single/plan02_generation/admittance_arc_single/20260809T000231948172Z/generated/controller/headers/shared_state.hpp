#pragma once

#include "frame_layout.h"
#include "runtime.hpp"
#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "admittance_arc_single_fsm.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <array>
#include <cstdint>
#include <mutex>
#include <thread>
#include "bdd_ros2_interfaces/msg/trinary_stamped.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
namespace motion_spec::ros {

// A hex UUID, with or without its dashes. An empty string is the zero UUID: a run that
// belongs to no scenario says so, rather than failing to start.
inline bool parse_uuid(const std::string &text, std::array<std::uint8_t, 16> &out) {
    std::string digits;
    for (const char c : text) {
        if (c != '-') digits.push_back(c);
    }
    if (digits.empty()) {
        out.fill(0);
        return true;
    }
    if (digits.size() != 32) return false;
    for (std::size_t i = 0; i < 16; ++i) {
        const std::string byte = digits.substr(i * 2, 2);
        if (byte.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) return false;
        out[i] = static_cast<std::uint8_t>(std::stoul(byte, nullptr, 16));
    }
    return true;
}

// Publish every Nth control cycle. A rate at or above the loop rate, or none at all, means
// every cycle -- the loop is the fastest this can go.
inline int publish_divider(double rate_hz) {
    if (rate_hz <= 0.0) return 1;
    const double every = 1.0 / (rate_hz * motion_spec::runtime::kControlPeriodS);
    return every <= 1.0 ? 1 : static_cast<int>(every + 0.5);
}

}  // namespace motion_spec::ros
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainhdsolver_vereshchagin_fixed_joint.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>
#include "headers/admittance_arc_single.kdl.hpp"
#include <kdl/chainhdsolver_vereshchagin_fext_fixed_joint.hpp>

inline constexpr int KINOVA_NUM_JOINTS = 7;

struct manipulator_robot {
    mj_kdl::Robot *robot = nullptr;
    KDL::Chain *chain = nullptr;
};

struct arm_solver_home_solver_state {
    bool initialized = false;
    int num_spatial_directions = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian spatial_directions;
    KDL::JntArray acceleration_energy;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> achd_acc;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
};

struct arm_solver_touchdown_solver_state {
    bool initialized = false;
    int num_spatial_directions = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian spatial_directions;
    KDL::JntArray acceleration_energy;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> achd_acc;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
};

struct arm_solver_forward_solver_state {
    bool initialized = false;
    int num_spatial_directions = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian spatial_directions;
    KDL::JntArray acceleration_energy;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> achd_acc;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
};

struct rne_arm_solver_arc_motion_solver_state {
    bool initialized = false;
    int num_spatial_directions = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian spatial_directions;
    KDL::JntArray cartesian_acceleration;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
    std::unique_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver;

};

struct rne_arm_solver_compliance_solver_state {
    bool initialized = false;
    int num_spatial_directions = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian spatial_directions;
    KDL::JntArray cartesian_acceleration;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
    std::unique_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver;

};

struct robot_io {
    struct events *fsm_events = nullptr;
    rclcpp::Node::SharedPtr ros_node = nullptr;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor = nullptr;
    std::thread ros_spin_thread;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ros_param_handle = nullptr;
    std::mutex ros_context_id_mutex;
    std::array<std::uint8_t, 16> ros_context_id{};
    std::shared_ptr<realtime_tools::RealtimePublisher<bdd_ros2_interfaces::msg::TrinaryStamped>> mon_forward_done_pub = nullptr;
    bdd_ros2_interfaces::msg::TrinaryStamped mon_forward_done_pub_msg{};
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> joint_states_pub = nullptr;
    sensor_msgs::msg::JointState joint_states_msg{};
    int joint_states_divider = 1;

    manipulator_robot arm_solver_home;
    manipulator_robot arm_solver_touchdown;
    manipulator_robot arm_solver_forward;
    manipulator_robot rne_arm_solver_arc_motion;
    manipulator_robot rne_arm_solver_compliance;
    int ft_wrist_ft_bias_samples = 0;
    KDL::Wrench *ext_force = nullptr;
};

struct motion_spec_event_buffer {
    static constexpr int size =
        motion_spec::introspection::kNumTriggers > 0
            ? static_cast<int>(motion_spec::introspection::kNumTriggers)
            : 1;
    int count = 0;
    int buffer[size] = {};
    void record(int event_idx) {
        if (event_idx < 0) return;
        buffer[count % size] = event_idx;
        ++count;
    }
};

struct shared_data {
    double acc_arc_path_along_speed_arc_motion = 0.0;
    double acc_ctrl_arc_orientation_ang_x = 0.0;
    double acc_ctrl_arc_orientation_ang_y = 0.0;
    double acc_ctrl_arc_orientation_ang_z = 0.0;
    double acc_ctrl_comply_ori_ang_x = 0.0;
    double acc_ctrl_comply_ori_ang_y = 0.0;
    double acc_ctrl_comply_ori_ang_z = 0.0;
    double acc_ctrl_follow_position_lin_normal_a = 0.0;
    double acc_ctrl_follow_position_lin_normal_b = 0.0;
    double acc_twist_ee_base_linear_x_compliance = 0.0;
    double acc_twist_ee_base_linear_y_compliance = 0.0;
    double acc_twist_ee_base_linear_z_compliance = 0.0;
    double admit_comply_x_ctrl_comply_x_damping = 30.0;
    double admit_comply_x_ctrl_comply_x_mass = 2.0;
    double admit_comply_x_ctrl_comply_x_max_velocity = 0.3;
    double admit_comply_x_ctrl_comply_x_maximum_velocity = 0.3;
    double admit_comply_x_ctrl_comply_x_stiffness = 0.0;
    double admit_comply_y_ctrl_comply_y_damping = 30.0;
    double admit_comply_y_ctrl_comply_y_mass = 2.0;
    double admit_comply_y_ctrl_comply_y_max_velocity = 0.3;
    double admit_comply_y_ctrl_comply_y_maximum_velocity = 0.3;
    double admit_comply_y_ctrl_comply_y_stiffness = 0.0;
    double admit_comply_z_ctrl_comply_z_damping = 30.0;
    double admit_comply_z_ctrl_comply_z_mass = 2.0;
    double admit_comply_z_ctrl_comply_z_max_velocity = 0.3;
    double admit_comply_z_ctrl_comply_z_maximum_velocity = 0.3;
    double admit_comply_z_ctrl_comply_z_stiffness = 0.0;
    double admittance_arc_exec_timestep = 0.001;
    double arc_height = 0.18;
    KDL::Frame arc_motion_start_pose;
    double arc_path_along_speed = 0.0;
    double arc_path_along_speed_err_arc_motion = 0.0;
    KDL::Vector arc_path_normal_a;
    KDL::Vector arc_path_normal_b;
    double arc_path_s = 0.0;
    KDL::Vector arc_path_tangent;
    double arc_speed = 0.05;
    double arm_solver_home_q_joint_1 = 0.0;
    double arm_solver_home_q_joint_2 = 0.0;
    double arm_solver_home_q_joint_3 = 0.0;
    double arm_solver_home_q_joint_4 = 0.0;
    double arm_solver_home_q_joint_5 = 0.0;
    double arm_solver_home_q_joint_6 = 0.0;
    double arm_solver_home_q_joint_7 = 0.0;
    double arm_solver_home_qd_joint_1 = 0.0;
    double arm_solver_home_qd_joint_2 = 0.0;
    double arm_solver_home_qd_joint_3 = 0.0;
    double arm_solver_home_qd_joint_4 = 0.0;
    double arm_solver_home_qd_joint_5 = 0.0;
    double arm_solver_home_qd_joint_6 = 0.0;
    double arm_solver_home_qd_joint_7 = 0.0;
    double arm_solver_home_qdd_joint_1 = 0.0;
    double arm_solver_home_qdd_joint_2 = 0.0;
    double arm_solver_home_qdd_joint_3 = 0.0;
    double arm_solver_home_qdd_joint_4 = 0.0;
    double arm_solver_home_qdd_joint_5 = 0.0;
    double arm_solver_home_qdd_joint_6 = 0.0;
    double arm_solver_home_qdd_joint_7 = 0.0;
    double arm_solver_home_tau_ctrl_joint_1 = 0.0;
    double arm_solver_home_tau_ctrl_joint_2 = 0.0;
    double arm_solver_home_tau_ctrl_joint_3 = 0.0;
    double arm_solver_home_tau_ctrl_joint_4 = 0.0;
    double arm_solver_home_tau_ctrl_joint_5 = 0.0;
    double arm_solver_home_tau_ctrl_joint_6 = 0.0;
    double arm_solver_home_tau_ctrl_joint_7 = 0.0;
    KDL::Vector base_link_com_in_base_link_position_coord = KDL::Vector(-6.48E-4, -1.66E-4, 0.084487);
    double bracelet_contact_z = 0.07;
    KDL::Vector bracelet_link_com_in_bracelet_link_position_coord = KDL::Vector(2.81E-4, 0.011402, -0.029798);
    double chord_x = -0.05;
    double chord_y = 0.3;
    double chord_y_hi = 0.33;
    double chord_y_lo = 0.27;
    double chord_z = 0.0;
    double clock_time_s = 0.0;
    double comply_int_max = 0.02;
    double comply_x_ctrl_comply_x_admit_ref = 0.0;
    double comply_y_ctrl_comply_y_admit_ref = 0.0;
    double comply_z_ctrl_comply_z_admit_ref = 0.0;
    double contact_z = 0.03;
    double ctrl_arc_orientation_ang_x_decay_rate = 0.0;
    double ctrl_arc_orientation_ang_x_error_integral = 0.0;
    bool ctrl_arc_orientation_ang_x_first_sample = false;
    double ctrl_arc_orientation_ang_x_kd = 160.0;
    double ctrl_arc_orientation_ang_x_ki = 0.0;
    double ctrl_arc_orientation_ang_x_kp = 240.0;
    double ctrl_arc_orientation_ang_x_previous_error = 0.0;
    double ctrl_arc_orientation_ang_y_decay_rate = 0.0;
    double ctrl_arc_orientation_ang_y_error_integral = 0.0;
    bool ctrl_arc_orientation_ang_y_first_sample = false;
    double ctrl_arc_orientation_ang_y_kd = 160.0;
    double ctrl_arc_orientation_ang_y_ki = 0.0;
    double ctrl_arc_orientation_ang_y_kp = 240.0;
    double ctrl_arc_orientation_ang_y_previous_error = 0.0;
    double ctrl_arc_orientation_ang_z_decay_rate = 0.0;
    double ctrl_arc_orientation_ang_z_error_integral = 0.0;
    bool ctrl_arc_orientation_ang_z_first_sample = false;
    double ctrl_arc_orientation_ang_z_kd = 160.0;
    double ctrl_arc_orientation_ang_z_ki = 0.0;
    double ctrl_arc_orientation_ang_z_kp = 240.0;
    double ctrl_arc_orientation_ang_z_previous_error = 0.0;
    double ctrl_arc_orientation_err_ang_x = 0.0;
    double ctrl_arc_orientation_err_ang_y = 0.0;
    double ctrl_arc_orientation_err_ang_z = 0.0;
    double ctrl_arc_orientation_measured_derivative_ang_x = 0.0;
    double ctrl_arc_orientation_measured_derivative_ang_y = 0.0;
    double ctrl_arc_orientation_measured_derivative_ang_z = 0.0;
    double ctrl_comply_ori_ang_x_decay_rate = 0.0;
    double ctrl_comply_ori_ang_x_error_integral = 0.0;
    bool ctrl_comply_ori_ang_x_first_sample = false;
    double ctrl_comply_ori_ang_x_kd = 160.0;
    double ctrl_comply_ori_ang_x_ki = 0.0;
    double ctrl_comply_ori_ang_x_kp = 240.0;
    double ctrl_comply_ori_ang_x_previous_error = 0.0;
    double ctrl_comply_ori_ang_y_decay_rate = 0.0;
    double ctrl_comply_ori_ang_y_error_integral = 0.0;
    bool ctrl_comply_ori_ang_y_first_sample = false;
    double ctrl_comply_ori_ang_y_kd = 160.0;
    double ctrl_comply_ori_ang_y_ki = 0.0;
    double ctrl_comply_ori_ang_y_kp = 240.0;
    double ctrl_comply_ori_ang_y_previous_error = 0.0;
    double ctrl_comply_ori_ang_z_decay_rate = 0.0;
    double ctrl_comply_ori_ang_z_error_integral = 0.0;
    bool ctrl_comply_ori_ang_z_first_sample = false;
    double ctrl_comply_ori_ang_z_kd = 160.0;
    double ctrl_comply_ori_ang_z_ki = 0.0;
    double ctrl_comply_ori_ang_z_kp = 240.0;
    double ctrl_comply_ori_ang_z_previous_error = 0.0;
    double ctrl_comply_ori_err_ang_x = 0.0;
    double ctrl_comply_ori_err_ang_y = 0.0;
    double ctrl_comply_ori_err_ang_z = 0.0;
    double ctrl_comply_ori_measured_derivative_ang_x = 0.0;
    double ctrl_comply_ori_measured_derivative_ang_y = 0.0;
    double ctrl_comply_ori_measured_derivative_ang_z = 0.0;
    double ctrl_comply_x_decay_rate = 0.0;
    double ctrl_comply_x_error_integral = 0.0;
    bool ctrl_comply_x_first_sample = false;
    double ctrl_comply_x_kd = 0.0;
    double ctrl_comply_x_ki = 800.0;
    double ctrl_comply_x_kp = 150.0;
    double ctrl_comply_x_previous_error = 0.0;
    double ctrl_comply_y_decay_rate = 0.0;
    double ctrl_comply_y_error_integral = 0.0;
    bool ctrl_comply_y_first_sample = false;
    double ctrl_comply_y_kd = 0.0;
    double ctrl_comply_y_ki = 800.0;
    double ctrl_comply_y_kp = 150.0;
    double ctrl_comply_y_previous_error = 0.0;
    double ctrl_comply_z_decay_rate = 0.0;
    double ctrl_comply_z_error_integral = 0.0;
    bool ctrl_comply_z_first_sample = false;
    double ctrl_comply_z_kd = 0.0;
    double ctrl_comply_z_ki = 800.0;
    double ctrl_comply_z_kp = 150.0;
    double ctrl_comply_z_previous_error = 0.0;
    double ctrl_follow_position_err_lin_normal_a = 0.0;
    double ctrl_follow_position_err_lin_normal_b = 0.0;
    double ctrl_follow_position_lin_normal_a_decay_rate = 0.0;
    double ctrl_follow_position_lin_normal_a_error_integral = 0.0;
    bool ctrl_follow_position_lin_normal_a_first_sample = false;
    double ctrl_follow_position_lin_normal_a_kd = 40.0;
    double ctrl_follow_position_lin_normal_a_ki = 0.0;
    double ctrl_follow_position_lin_normal_a_kp = 200.0;
    double ctrl_follow_position_lin_normal_a_previous_error = 0.0;
    double ctrl_follow_position_lin_normal_b_decay_rate = 0.0;
    double ctrl_follow_position_lin_normal_b_error_integral = 0.0;
    bool ctrl_follow_position_lin_normal_b_first_sample = false;
    double ctrl_follow_position_lin_normal_b_kd = 40.0;
    double ctrl_follow_position_lin_normal_b_ki = 0.0;
    double ctrl_follow_position_lin_normal_b_kp = 200.0;
    double ctrl_follow_position_lin_normal_b_previous_error = 0.0;
    double ctrl_follow_position_measured_derivative_lin_normal_a = 0.0;
    double ctrl_follow_position_measured_derivative_lin_normal_b = 0.0;
    double ctrl_follow_tangent_decay_rate = 0.0;
    double ctrl_follow_tangent_error_integral = 0.0;
    bool ctrl_follow_tangent_first_sample = false;
    double ctrl_follow_tangent_kd = 0.0;
    double ctrl_follow_tangent_ki = 4.0;
    double ctrl_follow_tangent_kp = 20.0;
    double ctrl_follow_tangent_previous_error = 0.0;
    double ctrl_forward_ori_ang_x_decay_rate = 0.0;
    double ctrl_forward_ori_ang_x_error_integral = 0.0;
    bool ctrl_forward_ori_ang_x_first_sample = false;
    double ctrl_forward_ori_ang_x_kd = 80.0;
    double ctrl_forward_ori_ang_x_ki = 50.0;
    double ctrl_forward_ori_ang_x_kp = 120.0;
    double ctrl_forward_ori_ang_x_previous_error = 0.0;
    double ctrl_forward_ori_ang_y_decay_rate = 0.0;
    double ctrl_forward_ori_ang_y_error_integral = 0.0;
    bool ctrl_forward_ori_ang_y_first_sample = false;
    double ctrl_forward_ori_ang_y_kd = 80.0;
    double ctrl_forward_ori_ang_y_ki = 50.0;
    double ctrl_forward_ori_ang_y_kp = 120.0;
    double ctrl_forward_ori_ang_y_previous_error = 0.0;
    double ctrl_forward_ori_ang_z_decay_rate = 0.0;
    double ctrl_forward_ori_ang_z_error_integral = 0.0;
    bool ctrl_forward_ori_ang_z_first_sample = false;
    double ctrl_forward_ori_ang_z_kd = 80.0;
    double ctrl_forward_ori_ang_z_ki = 50.0;
    double ctrl_forward_ori_ang_z_kp = 120.0;
    double ctrl_forward_ori_ang_z_previous_error = 0.0;
    double ctrl_forward_ori_err_ang_x = 0.0;
    double ctrl_forward_ori_err_ang_y = 0.0;
    double ctrl_forward_ori_err_ang_z = 0.0;
    double ctrl_hold_orientation_ang_x_decay_rate = 0.0;
    double ctrl_hold_orientation_ang_x_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_x_first_sample = false;
    double ctrl_hold_orientation_ang_x_kd = 80.0;
    double ctrl_hold_orientation_ang_x_ki = 50.0;
    double ctrl_hold_orientation_ang_x_kp = 120.0;
    double ctrl_hold_orientation_ang_x_previous_error = 0.0;
    double ctrl_hold_orientation_ang_y_decay_rate = 0.0;
    double ctrl_hold_orientation_ang_y_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_y_first_sample = false;
    double ctrl_hold_orientation_ang_y_kd = 80.0;
    double ctrl_hold_orientation_ang_y_ki = 50.0;
    double ctrl_hold_orientation_ang_y_kp = 120.0;
    double ctrl_hold_orientation_ang_y_previous_error = 0.0;
    double ctrl_hold_orientation_ang_z_decay_rate = 0.0;
    double ctrl_hold_orientation_ang_z_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_z_first_sample = false;
    double ctrl_hold_orientation_ang_z_kd = 80.0;
    double ctrl_hold_orientation_ang_z_ki = 50.0;
    double ctrl_hold_orientation_ang_z_kp = 120.0;
    double ctrl_hold_orientation_ang_z_previous_error = 0.0;
    double ctrl_hold_orientation_err_ang_x = 0.0;
    double ctrl_hold_orientation_err_ang_y = 0.0;
    double ctrl_hold_orientation_err_ang_z = 0.0;
    double ctrl_hold_position_err_lin_x = 0.0;
    double ctrl_hold_position_err_lin_y = 0.0;
    double ctrl_hold_position_err_lin_z = 0.0;
    double ctrl_hold_position_lin_x_decay_rate = 0.0;
    double ctrl_hold_position_lin_x_error_integral = 0.0;
    bool ctrl_hold_position_lin_x_first_sample = false;
    double ctrl_hold_position_lin_x_kd = 40.0;
    double ctrl_hold_position_lin_x_ki = 100.0;
    double ctrl_hold_position_lin_x_kp = 200.0;
    double ctrl_hold_position_lin_x_previous_error = 0.0;
    double ctrl_hold_position_lin_y_decay_rate = 0.0;
    double ctrl_hold_position_lin_y_error_integral = 0.0;
    bool ctrl_hold_position_lin_y_first_sample = false;
    double ctrl_hold_position_lin_y_kd = 40.0;
    double ctrl_hold_position_lin_y_ki = 100.0;
    double ctrl_hold_position_lin_y_kp = 200.0;
    double ctrl_hold_position_lin_y_previous_error = 0.0;
    double ctrl_hold_position_lin_z_decay_rate = 0.0;
    double ctrl_hold_position_lin_z_error_integral = 0.0;
    bool ctrl_hold_position_lin_z_first_sample = false;
    double ctrl_hold_position_lin_z_kd = 40.0;
    double ctrl_hold_position_lin_z_ki = 100.0;
    double ctrl_hold_position_lin_z_kp = 200.0;
    double ctrl_hold_position_lin_z_previous_error = 0.0;
    double ctrl_move_forward_decay_rate = 0.0;
    double ctrl_move_forward_error_integral = 0.0;
    bool ctrl_move_forward_first_sample = false;
    double ctrl_move_forward_kd = 40.0;
    double ctrl_move_forward_ki = 100.0;
    double ctrl_move_forward_kp = 200.0;
    double ctrl_move_forward_previous_error = 0.0;
    double ctrl_touchdown_ori_ang_x_decay_rate = 0.0;
    double ctrl_touchdown_ori_ang_x_error_integral = 0.0;
    bool ctrl_touchdown_ori_ang_x_first_sample = false;
    double ctrl_touchdown_ori_ang_x_kd = 80.0;
    double ctrl_touchdown_ori_ang_x_ki = 50.0;
    double ctrl_touchdown_ori_ang_x_kp = 120.0;
    double ctrl_touchdown_ori_ang_x_previous_error = 0.0;
    double ctrl_touchdown_ori_ang_y_decay_rate = 0.0;
    double ctrl_touchdown_ori_ang_y_error_integral = 0.0;
    bool ctrl_touchdown_ori_ang_y_first_sample = false;
    double ctrl_touchdown_ori_ang_y_kd = 80.0;
    double ctrl_touchdown_ori_ang_y_ki = 50.0;
    double ctrl_touchdown_ori_ang_y_kp = 120.0;
    double ctrl_touchdown_ori_ang_y_previous_error = 0.0;
    double ctrl_touchdown_ori_ang_z_decay_rate = 0.0;
    double ctrl_touchdown_ori_ang_z_error_integral = 0.0;
    bool ctrl_touchdown_ori_ang_z_first_sample = false;
    double ctrl_touchdown_ori_ang_z_kd = 80.0;
    double ctrl_touchdown_ori_ang_z_ki = 50.0;
    double ctrl_touchdown_ori_ang_z_kp = 120.0;
    double ctrl_touchdown_ori_ang_z_previous_error = 0.0;
    double ctrl_touchdown_ori_err_ang_x = 0.0;
    double ctrl_touchdown_ori_err_ang_y = 0.0;
    double ctrl_touchdown_ori_err_ang_z = 0.0;
    double ctrl_touchdown_x_decay_rate = 0.0;
    double ctrl_touchdown_x_error_integral = 0.0;
    bool ctrl_touchdown_x_first_sample = false;
    double ctrl_touchdown_x_kd = 25.0;
    double ctrl_touchdown_x_ki = 20.0;
    double ctrl_touchdown_x_kp = 80.0;
    double ctrl_touchdown_x_previous_error = 0.0;
    double ctrl_touchdown_y_decay_rate = 0.0;
    double ctrl_touchdown_y_error_integral = 0.0;
    bool ctrl_touchdown_y_first_sample = false;
    double ctrl_touchdown_y_kd = 25.0;
    double ctrl_touchdown_y_ki = 20.0;
    double ctrl_touchdown_y_kp = 80.0;
    double ctrl_touchdown_y_previous_error = 0.0;
    double ctrl_touchdown_z_decay_rate = 0.0;
    double ctrl_touchdown_z_error_integral = 0.0;
    bool ctrl_touchdown_z_first_sample = false;
    double ctrl_touchdown_z_kd = 0.0;
    double ctrl_touchdown_z_ki = 0.0;
    double ctrl_touchdown_z_kp = 200.0;
    double ctrl_touchdown_z_previous_error = 0.0;
    double default_tolerance_Distance = 0.01;
    double default_tolerance_LinearVelocity = 0.01;
    double default_tolerance_Orientation = 0.01;
    double default_tolerance_Position = 0.01;
    double descend_vel = -0.05;
    double dt_measured_s = 0.001;
    double eacc_ctrl_forward_ori_ang_x = 0.0;
    double eacc_ctrl_forward_ori_ang_y = 0.0;
    double eacc_ctrl_forward_ori_ang_z = 0.0;
    double eacc_ctrl_hold_orientation_ang_x = 0.0;
    double eacc_ctrl_hold_orientation_ang_y = 0.0;
    double eacc_ctrl_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_hold_position_lin_x = 0.0;
    double eacc_ctrl_hold_position_lin_y = 0.0;
    double eacc_ctrl_hold_position_lin_z = 0.0;
    double eacc_ctrl_touchdown_ori_ang_x = 0.0;
    double eacc_ctrl_touchdown_ori_ang_y = 0.0;
    double eacc_ctrl_touchdown_ori_ang_z = 0.0;
    double eacc_pose_ee_base_distance_x_forward = 0.0;
    double eacc_pose_ee_base_distance_x_touchdown = 0.0;
    double eacc_pose_ee_base_distance_y_touchdown = 0.0;
    double eacc_twist_ee_base_linear_z_touchdown = 0.0;
    KDL::Frame end_pose;
    KDL::Rotation end_pose_orientation_orientation_rel;
    KDL::Vector end_pose_position_position_rel;
    double end_x = 0.0;
    double end_y = 0.0;
    double end_y_hi = 0.0;
    double end_y_lo = 0.0;
    double end_z = 0.0;
    double eval_arc_motion_until_at_target_y_err = 0.0;
    double eval_arc_motion_until_near_table_err = 0.0;
    double eval_arc_motion_while_advance_err = 0.0;
    double eval_arc_motion_while_force_x_err = 0.0;
    double eval_arc_motion_while_force_y_err = 0.0;
    double eval_contact_table_err = 0.0;
    double eval_forward_until_reached_forward_err = 0.0;
    double eval_home_until_settled_x_err = 0.0;
    double eval_home_until_settled_y_err = 0.0;
    double eval_home_until_settled_z_err = 0.0;
    double eval_near_table_err = 0.0;
    double eval_released_x_err = 0.0;
    double eval_released_y_err = 0.0;
    double eval_released_z_err = 0.0;
    double eval_touchdown_until_contact_table_err = 0.0;
    KDL::Wrench ext_force;
    double ext_force_force_x = 0.0;
    double ext_force_force_y = 0.0;
    double ext_force_force_z = 0.0;
    KDL::Wrench ext_force_ft_bias;
    int ext_force_ft_settle = 0;
    double force_threshold = 10.0;
    KDL::Vector forearm_link_com_in_forearm_link_position_coord = KDL::Vector(-1.8E-5, -0.075478, -0.015006);
    double forward_distance = 0.05;
    KDL::Frame forward_start_pose;
    KDL::Rotation forward_start_pose_orientation_rel;
    double forward_start_pose_position_x = 0.0;
    KDL::Vector g_base_com_in_g_base_position_coord = KDL::Vector(0.0, -2.70394E-5, 0.0354675);
    KDL::Vector g_base_in_g_base_mount_position_coord = KDL::Vector(0.0, 0.0, 0.0038);
    KDL::Vector g_base_lumped_com_in_g_base_position_coord = KDL::Vector(0.0, -2.3289840972067775E-5, 0.04247701482094086);
    KDL::Vector g_base_mount_com_in_g_base_mount_position_coord = KDL::Vector(-3.605835958214157E-4, 8.536274800937428E-5, -5.9199141450506514E-5);
    KDL::Vector g_left_driver_com_in_g_left_driver_position_coord = KDL::Vector(0.0, 0.0177547, 0.00107314);
    KDL::Vector g_left_driver_joint_anchor_in_g_base_position_coord = KDL::Vector(0.0, -0.0306011, 0.054904);
    double g_left_driver_joint_limit_effort_lower = -5.0;
    double g_left_driver_joint_limit_effort_upper = 5.0;
    double g_left_driver_joint_limit_position_lower = 0.0;
    double g_left_driver_joint_limit_position_upper = 0.8;
    KDL::Vector g_mount_interface_in_g_base_mount_position_coord = KDL::Vector(0.0, 0.0, -0.007);
    KDL::Vector g_pinch_in_g_base_position_coord = KDL::Vector(0.0, 0.0, 0.145);
    KDL::Vector gravity_value_arm_solver = KDL::Vector(0.0, 0.0, 9.81);
    KDL::Vector gravity_value_rne_arm_solver = KDL::Vector(0.0, 0.0, 9.81);
    KDL::Vector half_arm_1_link_com_in_half_arm_1_link_position_coord = KDL::Vector(-4.4E-5, -0.09958, -0.013278);
    KDL::Vector half_arm_2_link_com_in_half_arm_2_link_position_coord = KDL::Vector(-4.4E-5, -0.006641, -0.117892);
    KDL::Frame hold_orientation;
    KDL::Rotation hold_orientation_orientation_rel;
    KDL::Frame home_pose;
    KDL::Rotation home_pose_orientation_rel;
    KDL::Vector home_pose_position_rel;
    KDL::Frame initial_pose;
    double initial_pose_position_x = 0.0;
    double initial_pose_position_y = 0.0;
    double initial_pose_position_z = 0.0;
    KDL::Vector joint_1_anchor_in_base_link_position_coord = KDL::Vector(0.0, 0.0, 0.15643);
    double joint_1_limit_effort_lower = -105.0;
    double joint_1_limit_effort_upper = 105.0;
    KDL::Vector joint_2_anchor_in_shoulder_link_position_coord = KDL::Vector(0.0, 0.005375, -0.12838);
    double joint_2_limit_effort_lower = -105.0;
    double joint_2_limit_effort_upper = 105.0;
    double joint_2_limit_position_lower = -2.24;
    double joint_2_limit_position_upper = 2.24;
    KDL::Vector joint_3_anchor_in_half_arm_1_link_position_coord = KDL::Vector(0.0, -0.21038, -0.006375);
    double joint_3_limit_effort_lower = -105.0;
    double joint_3_limit_effort_upper = 105.0;
    KDL::Vector joint_4_anchor_in_half_arm_2_link_position_coord = KDL::Vector(0.0, 0.006375, -0.21038);
    double joint_4_limit_effort_lower = -105.0;
    double joint_4_limit_effort_upper = 105.0;
    double joint_4_limit_position_lower = -2.57;
    double joint_4_limit_position_upper = 2.57;
    KDL::Vector joint_5_anchor_in_forearm_link_position_coord = KDL::Vector(0.0, -0.20843, -0.006375);
    double joint_5_limit_effort_lower = -52.0;
    double joint_5_limit_effort_upper = 52.0;
    KDL::Vector joint_6_anchor_in_spherical_wrist_1_link_position_coord = KDL::Vector(0.0, 1.7505E-4, -0.10593);
    double joint_6_limit_effort_lower = -52.0;
    double joint_6_limit_effort_upper = 52.0;
    double joint_6_limit_position_lower = -2.09;
    double joint_6_limit_position_upper = 2.09;
    KDL::Vector joint_7_anchor_in_spherical_wrist_2_link_position_coord = KDL::Vector(0.0, -0.10593, -1.7505E-4);
    double joint_7_limit_effort_lower = -52.0;
    double joint_7_limit_effort_upper = 52.0;
    double min_arc_speed = 0.005;
    double mon_arc_complete_debounce = 0.3;
    double mon_contact_debounce = 0.3;
    double mon_force_x_debounce = 0.3;
    double mon_force_y_debounce = 0.3;
    double mon_home_settled_debounce = 0.3;
    double mon_released_debounce = 0.3;
    double mon_table_contact_debounce = 0.3;
    double neg_comply_int_max = -0.02;
    double neg_force_threshold = -10.0;
    double neg_release_threshold = -3.0;
    KDL::Vector path_normal = KDL::Vector(0.9863939238, 0.1643989873, 0.0);
    KDL::Vector pinch_site_in_bracelet_link_position_coord = KDL::Vector(0.0, 0.0, -0.061525);
    KDL::Frame pose_bracelet_base;
    double pose_bracelet_base_distance_z = 0.0;
    KDL::Twist pose_diff_ctrl_arc_orientation;
    KDL::Twist pose_diff_ctrl_comply_ori;
    KDL::Twist pose_diff_ctrl_follow_position;
    KDL::Twist pose_diff_ctrl_forward_ori;
    KDL::Twist pose_diff_ctrl_hold_orientation;
    KDL::Twist pose_diff_ctrl_hold_position;
    KDL::Twist pose_diff_ctrl_touchdown_ori;
    KDL::Frame pose_ee_base;
    double pose_ee_base_distance_x = 0.0;
    double pose_ee_base_distance_x_err_forward = 0.0;
    double pose_ee_base_distance_x_err_touchdown = 0.0;
    double pose_ee_base_distance_y = 0.0;
    double pose_ee_base_distance_y_err_touchdown = 0.0;
    double pose_ee_base_distance_z = 0.0;
    KDL::Rotation pose_ee_base_orientation_rel;
    KDL::Vector pose_ee_base_position_rel;
    KDL::Frame reference;
    KDL::Rotation reference_orientation_rel;
    KDL::Vector reference_position_rel;
    double release_threshold = 3.0;
    double satisfied_band = 0.01;
    double satisfied_band_rot = 0.01;
    double satisfied_band_vel = 0.01;
    KDL::Vector shoulder_link_com_in_shoulder_link_position_coord = KDL::Vector(-2.3E-5, -0.010364, -0.07336);
    KDL::Vector spherical_wrist_1_link_com_in_spherical_wrist_1_link_position_coord = KDL::Vector(1.0E-6, -0.009432, -0.063883);
    KDL::Vector spherical_wrist_2_link_com_in_spherical_wrist_2_link_position_coord = KDL::Vector(1.0E-6, -0.045483, -0.00965);
    double start_pose_position_y = 0.0;
    KDL::Vector table_com_in_table_top_position_coord = KDL::Vector(0.0, 0.0, -0.05100416904566816);
    KDL::Vector table_top_in_world_position_coord = KDL::Vector(0.0, 0.0, 0.72);
    double target_x = 0.0;
    KDL::Frame touchdown_start_pose;
    KDL::Rotation touchdown_start_pose_orientation_rel;
    double touchdown_start_pose_position_x = 0.0;
    KDL::Twist twist_bracelet_base;
    double twist_bracelet_base_linear_z = 0.0;
    KDL::Twist twist_ee_base;
    double twist_ee_base_linear_x = 0.0;
    double twist_ee_base_linear_x_err_compliance = 0.0;
    double twist_ee_base_linear_y = 0.0;
    double twist_ee_base_linear_y_err_compliance = 0.0;
    double twist_ee_base_linear_z = 0.0;
    double twist_ee_base_linear_z_err_compliance = 0.0;
    double twist_ee_base_linear_z_err_touchdown = 0.0;
    double update_rate = 1000.0;
    KDL::Vector wrist_anchor_in_bracelet_link_position_coord = KDL::Vector(0.0, -0.05639, -0.058475);
    KDL::Vector wrist_ft_site_in_wrist_ft_body_position_coord = KDL::Vector(0.0, 0.0, 0.0);
    double zero_linvel = 0.0;
};
