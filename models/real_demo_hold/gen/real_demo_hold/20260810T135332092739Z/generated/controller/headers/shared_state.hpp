#pragma once

#include "frame_layout.h"
#include "runtime.hpp"
#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "real_demo_hold_fsm.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <array>
#include <cstdint>
#include <mutex>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
namespace motion_spec::ros {

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

#include "headers/real_demo_hold.kdl.hpp"
#include "device_io.hpp"
#include <robif2b/functions/kinova_gen3.h>
#include <kdl/chainhdsolver_vereshchagin_fext_fixed_joint.hpp>

inline constexpr int KINOVA_NUM_JOINTS = 7;

// The gripper closes under its own speed and force loop; the model commands position only. What
// speed and what force come from the robot config -- see motion_spec::config::gripper_pct.

struct kinova_state {
    bool success = false;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    double pos_msr[KINOVA_NUM_JOINTS] = {0.0};
    double vel_msr[KINOVA_NUM_JOINTS] = {0.0};
    double eff_msr[KINOVA_NUM_JOINTS] = {0.0};
    double cur_msr[KINOVA_NUM_JOINTS] = {0.0};
    double pos_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double vel_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double eff_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double cur_cmd[KINOVA_NUM_JOINTS] = {0.0};
    // The base IMU is not optional: the driver writes it on every measurement it publishes,
    // whether or not the model asks for it, and asserts on the buffer being there.
    double imu_ang_vel_msr[3] = {0.0};
    double imu_lin_acc_msr[3] = {0.0};
};

// Gripper over the arm's interconnect: every port is a percentage of travel.
struct kg3_gripper_state {
    bool success = false;
    float pos_msr = 0.0f;
    float vel_msr = 0.0f;
    float cur_msr = 0.0f;
    float pos_cmd = 0.0f;
    float vel_cmd = 0.0f;
    float frc_cmd = 0.0f;
};

struct manipulator_robot {
    kinova_state *state = nullptr;
    robif2b_kinova_gen3_nbx *robot = nullptr;
    KDL::Chain *chain = nullptr;
    // The commanded gripper joint position [rad], as the model authored it. Which device that
    // reaches, and in whose units, is settled at generation by the bound device's rule set.
    double *gripper_cmd = nullptr;
    // The measured gripper joint position [rad]: the mimic joint is not in the chain, so the
    // bound gripper device reports it.
    double *gripper_pos = nullptr;
};

struct arm_solver_hold_pose_solver_state {
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

};

struct robot_io {
    struct events *fsm_events = nullptr;
    rclcpp::Node::SharedPtr ros_node = nullptr;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor = nullptr;
    std::thread ros_spin_thread;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> joint_states_pub = nullptr;
    sensor_msgs::msg::JointState joint_states_msg{};
    int joint_states_divider = 1;

    manipulator_robot arm_solver_hold_pose;
    const ft_sensor_sample *ft_wrist_ft = nullptr;
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
    double arm_solver_hold_pose_q_joint_1 = 0.0;
    double arm_solver_hold_pose_q_joint_2 = 0.0;
    double arm_solver_hold_pose_q_joint_3 = 0.0;
    double arm_solver_hold_pose_q_joint_4 = 0.0;
    double arm_solver_hold_pose_q_joint_5 = 0.0;
    double arm_solver_hold_pose_q_joint_6 = 0.0;
    double arm_solver_hold_pose_q_joint_7 = 0.0;
    double arm_solver_hold_pose_qd_joint_1 = 0.0;
    double arm_solver_hold_pose_qd_joint_2 = 0.0;
    double arm_solver_hold_pose_qd_joint_3 = 0.0;
    double arm_solver_hold_pose_qd_joint_4 = 0.0;
    double arm_solver_hold_pose_qd_joint_5 = 0.0;
    double arm_solver_hold_pose_qd_joint_6 = 0.0;
    double arm_solver_hold_pose_qd_joint_7 = 0.0;
    double arm_solver_hold_pose_qdd_joint_1 = 0.0;
    double arm_solver_hold_pose_qdd_joint_2 = 0.0;
    double arm_solver_hold_pose_qdd_joint_3 = 0.0;
    double arm_solver_hold_pose_qdd_joint_4 = 0.0;
    double arm_solver_hold_pose_qdd_joint_5 = 0.0;
    double arm_solver_hold_pose_qdd_joint_6 = 0.0;
    double arm_solver_hold_pose_qdd_joint_7 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_1 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_2 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_3 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_4 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_5 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_6 = 0.0;
    double arm_solver_hold_pose_tau_ctrl_joint_7 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_1 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_2 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_3 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_4 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_5 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_6 = 0.0;
    double arm_solver_hold_pose_tau_msr_joint_7 = 0.0;
    KDL::Vector base_link_com_in_base_link_position_coord = KDL::Vector(-6.48E-4, -1.66E-4, 0.084487);
    KDL::Vector bracelet_link_com_in_bracelet_link_position_coord = KDL::Vector(2.81E-4, 0.011402, -0.029798);
    double clock_time_s = 0.0;
    double ctrl_hold_orientation_ang_x_decay_rate = 0.1;
    double ctrl_hold_orientation_ang_x_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_x_first_sample = false;
    double ctrl_hold_orientation_ang_x_kd = 2.0;
    double ctrl_hold_orientation_ang_x_ki = 50.0;
    double ctrl_hold_orientation_ang_x_kp = 250.0;
    double ctrl_hold_orientation_ang_x_previous_error = 0.0;
    double ctrl_hold_orientation_ang_y_decay_rate = 0.1;
    double ctrl_hold_orientation_ang_y_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_y_first_sample = false;
    double ctrl_hold_orientation_ang_y_kd = 2.0;
    double ctrl_hold_orientation_ang_y_ki = 50.0;
    double ctrl_hold_orientation_ang_y_kp = 250.0;
    double ctrl_hold_orientation_ang_y_previous_error = 0.0;
    double ctrl_hold_orientation_ang_z_decay_rate = 0.1;
    double ctrl_hold_orientation_ang_z_error_integral = 0.0;
    bool ctrl_hold_orientation_ang_z_first_sample = false;
    double ctrl_hold_orientation_ang_z_kd = 2.0;
    double ctrl_hold_orientation_ang_z_ki = 50.0;
    double ctrl_hold_orientation_ang_z_kp = 250.0;
    double ctrl_hold_orientation_ang_z_previous_error = 0.0;
    double ctrl_hold_orientation_err_ang_x = 0.0;
    double ctrl_hold_orientation_err_ang_y = 0.0;
    double ctrl_hold_orientation_err_ang_z = 0.0;
    double ctrl_hold_position_err_lin_x = 0.0;
    double ctrl_hold_position_err_lin_y = 0.0;
    double ctrl_hold_position_err_lin_z = 0.0;
    double ctrl_hold_position_lin_x_decay_rate = 0.1;
    double ctrl_hold_position_lin_x_error_integral = 0.0;
    bool ctrl_hold_position_lin_x_first_sample = false;
    double ctrl_hold_position_lin_x_kd = 2.0;
    double ctrl_hold_position_lin_x_ki = 50.0;
    double ctrl_hold_position_lin_x_kp = 550.0;
    double ctrl_hold_position_lin_x_previous_error = 0.0;
    double ctrl_hold_position_lin_y_decay_rate = 0.1;
    double ctrl_hold_position_lin_y_error_integral = 0.0;
    bool ctrl_hold_position_lin_y_first_sample = false;
    double ctrl_hold_position_lin_y_kd = 2.0;
    double ctrl_hold_position_lin_y_ki = 50.0;
    double ctrl_hold_position_lin_y_kp = 550.0;
    double ctrl_hold_position_lin_y_previous_error = 0.0;
    double ctrl_hold_position_lin_z_decay_rate = 0.1;
    double ctrl_hold_position_lin_z_error_integral = 0.0;
    bool ctrl_hold_position_lin_z_first_sample = false;
    double ctrl_hold_position_lin_z_kd = 2.0;
    double ctrl_hold_position_lin_z_ki = 50.0;
    double ctrl_hold_position_lin_z_kp = 550.0;
    double ctrl_hold_position_lin_z_previous_error = 0.0;
    double default_tolerance_Orientation = 0.1;
    double default_tolerance_Position = 0.05;
    double dt_measured_s = 0.001;
    double eacc_ctrl_hold_orientation_ang_x = 0.0;
    double eacc_ctrl_hold_orientation_ang_y = 0.0;
    double eacc_ctrl_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_hold_position_lin_x = 0.0;
    double eacc_ctrl_hold_position_lin_y = 0.0;
    double eacc_ctrl_hold_position_lin_z = 0.0;
    KDL::Wrench ext_force;
    KDL::Wrench ext_force_ft_bias;
    int ext_force_ft_settle = 0;
    KDL::Vector forearm_link_com_in_forearm_link_position_coord = KDL::Vector(-1.8E-5, -0.075478, -0.015006);
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
    KDL::Vector half_arm_1_link_com_in_half_arm_1_link_position_coord = KDL::Vector(-4.4E-5, -0.09958, -0.013278);
    KDL::Vector half_arm_2_link_com_in_half_arm_2_link_position_coord = KDL::Vector(-4.4E-5, -0.006641, -0.117892);
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
    KDL::Vector pinch_site_in_bracelet_link_position_coord = KDL::Vector(0.0, 0.0, -0.061525);
    KDL::Twist pose_diff_ctrl_hold_orientation;
    KDL::Twist pose_diff_ctrl_hold_position;
    KDL::Frame pose_ee_base;
    KDL::Rotation pose_ee_base_orientation_rel;
    KDL::Vector pose_ee_base_position_rel;
    double real_demo_hold_exec_timestep = 0.001;
    KDL::Vector shoulder_link_com_in_shoulder_link_position_coord = KDL::Vector(-2.3E-5, -0.010364, -0.07336);
    KDL::Vector spherical_wrist_1_link_com_in_spherical_wrist_1_link_position_coord = KDL::Vector(1.0E-6, -0.009432, -0.063883);
    KDL::Vector spherical_wrist_2_link_com_in_spherical_wrist_2_link_position_coord = KDL::Vector(1.0E-6, -0.045483, -0.00965);
    KDL::Frame start_pose;
    KDL::Rotation start_pose_orientation_rel;
    KDL::Vector start_pose_position_rel;
    KDL::Vector table_com_in_table_top_position_coord = KDL::Vector(0.0, 0.0, -0.05100416904566816);
    KDL::Vector table_top_in_world_position_coord = KDL::Vector(0.0, 0.0, 0.72);
    double update_rate = 1000.0;
    KDL::Vector wrist_anchor_in_bracelet_link_position_coord = KDL::Vector(0.0, -0.05639, -0.058475);
    KDL::Vector wrist_ft_mount_in_wrist_ft_body_position_coord = KDL::Vector(0.0, 0.0, 0.0365);
    KDL::Vector wrist_ft_site_in_wrist_ft_body_position_coord = KDL::Vector(0.0, 0.0, 0.0);
};
