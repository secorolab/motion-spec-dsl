#include "runtime.hpp"
#include "shared_state.hpp"
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
#include "introspect_model.hpp"
#endif
#include "motion_hold_pose.hpp"

#include <iostream>
#include "robot_config.hpp"
#include "device_io.hpp"

#include <unistd.h>

static void step_motion_hold_pose(motion_hold_pose_state &motion_hold_pose_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_hold_pose_state_instance.active) {
        motion_hold_pose_state_instance.active = true;
        motion_hold_pose_state_instance.active_steps = 0;
        motion_hold_pose_state_instance.snapshot_taken = false;
        motion_hold_pose_state_instance.ctrl_hold_position_lin_x.reset();
        motion_hold_pose_state_instance.ctrl_hold_position_lin_y.reset();
        motion_hold_pose_state_instance.ctrl_hold_position_lin_z.reset();
        motion_hold_pose_state_instance.ctrl_hold_orientation_ang_x.reset();
        motion_hold_pose_state_instance.ctrl_hold_orientation_ang_y.reset();
        motion_hold_pose_state_instance.ctrl_hold_orientation_ang_z.reset();
    }
    update_motion_hold_pose(motion_hold_pose_state_instance, shared, robot);
    monitor_until_motion_hold_pose();
    control_motion_hold_pose(motion_hold_pose_state_instance, shared, robot);
    apply_motion_hold_pose(motion_hold_pose_state_instance, robot);
    ++motion_hold_pose_state_instance.active_steps;
}

int main(int argc, char **argv) {
    motion_spec::runtime::install_stop_handlers();
    robot_io robot{};
    shared_data shared{};
    motion_spec_event_buffer events{};

    rclcpp::init(argc, argv);
    robot.ros_node = std::make_shared<rclcpp::Node>("motion_spec_monitor");

    const auto robot_config = motion_spec::config::load("/home/batsy/work/ms/src/motion-spec-dsl/models/real_demo_hold/robot.toml");
    kinova_state kinova_arm_solver_hold_pose_state{};
    kinova_arm_solver_hold_pose_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    ft_sensor_io ft_wrist_ft_io{};
    ft_wrist_ft_io.stale.limit = motion_spec::io::stale_after_cycles(
        motion_spec::config::stale_after_ms(robot_config, "arm1.wrist_ft"),
        motion_spec::runtime::kControlPeriodS);

    const auto cfg_arm_solver_hold_pose = motion_spec::config::device(robot_config, "agents.arm1");
    robif2b_kinova_gen3_nbx kinova_arm_solver_hold_pose{};
    kinova_arm_solver_hold_pose.conf.ip_address = cfg_arm_solver_hold_pose.ip.c_str();
    kinova_arm_solver_hold_pose.conf.port = cfg_arm_solver_hold_pose.port;
    kinova_arm_solver_hold_pose.conf.port_real_time = cfg_arm_solver_hold_pose.port_real_time;
    kinova_arm_solver_hold_pose.conf.user = cfg_arm_solver_hold_pose.user.c_str();
    kinova_arm_solver_hold_pose.conf.password = cfg_arm_solver_hold_pose.password.c_str();
    kinova_arm_solver_hold_pose.conf.session_timeout = cfg_arm_solver_hold_pose.session_timeout_ms;
    kinova_arm_solver_hold_pose.conf.connection_timeout = cfg_arm_solver_hold_pose.connection_timeout_ms;
    // The arm integrates a velocity command over this, and asserts on it every update: it is
    // the period the model declares, the same one the loop sleeps to.
    kinova_arm_solver_hold_pose.cycle_time = &motion_spec::runtime::kControlPeriodS;
    kinova_arm_solver_hold_pose.ctrl_mode = &kinova_arm_solver_hold_pose_state.ctrl_mode;
    kinova_arm_solver_hold_pose.jnt_pos_msr = &kinova_arm_solver_hold_pose_state.pos_msr[0];
    kinova_arm_solver_hold_pose.jnt_vel_msr = &kinova_arm_solver_hold_pose_state.vel_msr[0];
    kinova_arm_solver_hold_pose.jnt_trq_msr = &kinova_arm_solver_hold_pose_state.eff_msr[0];
    kinova_arm_solver_hold_pose.act_cur_msr = &kinova_arm_solver_hold_pose_state.cur_msr[0];
    kinova_arm_solver_hold_pose.imu_ang_vel_msr = &kinova_arm_solver_hold_pose_state.imu_ang_vel_msr[0];
    kinova_arm_solver_hold_pose.imu_lin_acc_msr = &kinova_arm_solver_hold_pose_state.imu_lin_acc_msr[0];
    kinova_arm_solver_hold_pose.jnt_pos_cmd = &kinova_arm_solver_hold_pose_state.pos_cmd[0];
    kinova_arm_solver_hold_pose.jnt_vel_cmd = &kinova_arm_solver_hold_pose_state.vel_cmd[0];
    kinova_arm_solver_hold_pose.jnt_trq_cmd = &kinova_arm_solver_hold_pose_state.eff_cmd[0];
    kinova_arm_solver_hold_pose.act_cur_cmd = &kinova_arm_solver_hold_pose_state.cur_cmd[0];
    kinova_arm_solver_hold_pose.success = &kinova_arm_solver_hold_pose_state.success;

    const auto cfg_ft_wrist_ft = motion_spec::config::serial(robot_config, "arm1.wrist_ft");
    robif2b_robotiq_ft_sensor_nbx ft_wrist_ft{};
    ft_wrist_ft.serial.port = cfg_ft_wrist_ft.port.c_str();
    ft_wrist_ft.serial.baudrate = cfg_ft_wrist_ft.baudrate;
    ft_wrist_ft.serial.timeout_ms = cfg_ft_wrist_ft.timeout_ms;
    ft_wrist_ft.serial.slave_address = cfg_ft_wrist_ft.slave_address;
    ft_wrist_ft.force_msr = &ft_wrist_ft_io.state.force[0];
    ft_wrist_ft.force_offset = &ft_wrist_ft_io.state.force_offset[0];
    ft_wrist_ft.moment_msr = &ft_wrist_ft_io.state.moment[0];
    ft_wrist_ft.moment_offset = &ft_wrist_ft_io.state.moment_offset[0];
    ft_wrist_ft.success = &ft_wrist_ft_io.state.success;

    KDL::Tree built_tree_arm_solver_hold_pose{};
    if (!scene_kdl::make_tree_world_tree(&built_tree_arm_solver_hold_pose)) {
        std::cerr << "make_tree_world_tree() failed for runtime arm_solver_hold_pose\n";
        return 1;
    }
    KDL::Chain chain_arm_solver_hold_pose{};
    if (!scene_kdl::make_chain_kinova_ft_2f85_chain(built_tree_arm_solver_hold_pose, &chain_arm_solver_hold_pose)) {
        std::cerr << "make_chain_kinova_ft_2f85_chain() failed for runtime arm_solver_hold_pose\n";
        return 1;
    }

    robot.arm_solver_hold_pose = manipulator_robot{
        .state = &kinova_arm_solver_hold_pose_state,
        .robot = &kinova_arm_solver_hold_pose,
        .chain = &chain_arm_solver_hold_pose,

    };
    robot.ft_wrist_ft = &ft_wrist_ft_io.sample;
    robot.ft_wrist_ft_bias_samples = motion_spec::config::bias_samples(robot_config, "arm1.wrist_ft");

    KDL::Wrench ext_force_measurement;
    robot.ext_force = &ext_force_measurement;
    motion_hold_pose_state motion_hold_pose_state_instance;
    robif2b_kinova_gen3_configure(&kinova_arm_solver_hold_pose);
    robif2b_kinova_gen3_recover(&kinova_arm_solver_hold_pose);
    robif2b_kinova_gen3_start(&kinova_arm_solver_hold_pose);
    if (!kinova_arm_solver_hold_pose_state.success) {
        std::cerr << "[run] device 'agents.arm1' failed to configure\n";
        return 1;
    }
    robif2b_robotiq_ft_configure(&ft_wrist_ft);
    if (!ft_wrist_ft_io.state.success) {
        std::cerr << "[run] device 'arm1.wrist_ft' failed to configure\n";
        return 1;
    }
    ft_wrist_ft_io.start(ft_wrist_ft);

    {
        // Named, not a literal: `section` returns a reference into robot_config, and gcc reads a
        // temporary argument as the thing the reference could dangle to.
        const std::string _js_key = "ros.joint_states";
        const toml::table &_js_cfg = motion_spec::config::section(robot_config, _js_key);
        robot.joint_states_divider =
            motion_spec::ros::publish_divider(_js_cfg["rate"].value_or(0.0));
        robot.joint_states_msg.name = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7"
        };
        const std::size_t _js_n = robot.joint_states_msg.name.size();
        robot.joint_states_msg.position.assign(_js_n, 0.0);
        robot.joint_states_msg.velocity.assign(_js_n, 0.0);
        robot.joint_states_msg.effort.assign(_js_n, 0.0);
        robot.joint_states_pub =
            std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
                robot.ros_node->create_publisher<sensor_msgs::msg::JointState>(
                    _js_cfg["topic"].value_or(std::string("/joint_states")), rclcpp::QoS(10)));
    }

    robot.ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    robot.ros_executor->add_node(robot.ros_node);
    robot.ros_spin_thread = std::thread([&robot]() { robot.ros_executor->spin(); });

    struct fsm_nbx *fsm = real_demo_hold_fsm::create_fsm();
    if (!fsm) return 1;
    robot.fsm_events = fsm->eventData;

    auto fsm_active_motion = [&](struct fsm_nbx *fsm) -> int {
        switch (fsm->currentStateIndex) {
        case real_demo_hold_fsm::S_HOLD: return 0;

        default: return -1;
        }
    };
    // The FSM's current state selects which motion runs; the motion's monitor advances the FSM. A
    // fallback hold state additionally re-evaluates the gated motion's WHEN precondition each tick.
    auto fsm_dispatch = [&](struct fsm_nbx *fsm) {
        switch (fsm->currentStateIndex) {
        case real_demo_hold_fsm::S_HOLD:
            step_motion_hold_pose(motion_hold_pose_state_instance, shared, robot);
            break;

        default: break;
        }
    };

    int _step = 0;
    int _fsm_prev_state = -1;
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
    motion_spec::introspection::RunPublisher _introspection_pub;
#endif
    timespec _next_tick{};
    clock_gettime(CLOCK_MONOTONIC, &_next_tick);
    double _prev_clock_s = 0.0;
    while (!motion_spec::runtime::stop_requested() && (true) && rclcpp::ok()) {
        shared.clock_time_s = motion_spec::runtime::monotonic_time_s();
        // Integrators step with the time that actually elapsed, not the period we asked for: the
        // loop sleeps to deadlines it can overrun. In sim the clock advances exactly one timestep
        // per tick, so this equals the nominal period.
        const double _dt_raw = (_step == 0)
            ? motion_spec::runtime::kControlPeriodS
            : shared.clock_time_s - _prev_clock_s;
        _prev_clock_s = shared.clock_time_s;
        shared.dt_measured_s = std::clamp(_dt_raw, motion_spec::runtime::kDtClampMin,
                                          motion_spec::runtime::kDtClampMax);

        if (static_cast<int>(fsm->currentStateIndex) != _fsm_prev_state) {
            _fsm_prev_state = static_cast<int>(fsm->currentStateIndex);
            std::cerr << "[fsm] state   " << real_demo_hold_fsm::STATE_URIS[fsm->currentStateIndex] << std::endl;
            // Leaving a state ends its motion's activation, so re-entering one is a fresh
            // start: active_steps returns to 0 and any trajectory replans from the current
            // pose instead of resuming the alpha it had when it was interrupted.
            motion_hold_pose_state_instance.active = false;
        }
        if (fsm->currentStateIndex == fsm->endStateIndex) {
            break;
        }
        // The heartbeat drives the FSM but is not recorded: the frame log is a time series, so
        // every frame already *is* the tick (t, step, wall_ns, period_ns). Logging a per-tick
        // event would restate that 24k times and leave last_event permanently pinned to it.
        produce_event(fsm->eventData, real_demo_hold_fsm::E_STEP);
        fsm_dispatch(fsm);
        if (robot.joint_states_pub && (_step % robot.joint_states_divider) == 0) {
            robot.joint_states_msg.header.stamp = robot.ros_node->now();
            robot.joint_states_msg.position[0] = shared.arm_solver_hold_pose_q_joint_1;
            robot.joint_states_msg.velocity[0] = shared.arm_solver_hold_pose_qd_joint_1;
            robot.joint_states_msg.effort[0] = shared.arm_solver_hold_pose_tau_ctrl_joint_1;
            robot.joint_states_msg.position[1] = shared.arm_solver_hold_pose_q_joint_2;
            robot.joint_states_msg.velocity[1] = shared.arm_solver_hold_pose_qd_joint_2;
            robot.joint_states_msg.effort[1] = shared.arm_solver_hold_pose_tau_ctrl_joint_2;
            robot.joint_states_msg.position[2] = shared.arm_solver_hold_pose_q_joint_3;
            robot.joint_states_msg.velocity[2] = shared.arm_solver_hold_pose_qd_joint_3;
            robot.joint_states_msg.effort[2] = shared.arm_solver_hold_pose_tau_ctrl_joint_3;
            robot.joint_states_msg.position[3] = shared.arm_solver_hold_pose_q_joint_4;
            robot.joint_states_msg.velocity[3] = shared.arm_solver_hold_pose_qd_joint_4;
            robot.joint_states_msg.effort[3] = shared.arm_solver_hold_pose_tau_ctrl_joint_4;
            robot.joint_states_msg.position[4] = shared.arm_solver_hold_pose_q_joint_5;
            robot.joint_states_msg.velocity[4] = shared.arm_solver_hold_pose_qd_joint_5;
            robot.joint_states_msg.effort[4] = shared.arm_solver_hold_pose_tau_ctrl_joint_5;
            robot.joint_states_msg.position[5] = shared.arm_solver_hold_pose_q_joint_6;
            robot.joint_states_msg.velocity[5] = shared.arm_solver_hold_pose_qd_joint_6;
            robot.joint_states_msg.effort[5] = shared.arm_solver_hold_pose_tau_ctrl_joint_6;
            robot.joint_states_msg.position[6] = shared.arm_solver_hold_pose_q_joint_7;
            robot.joint_states_msg.velocity[6] = shared.arm_solver_hold_pose_qd_joint_7;
            robot.joint_states_msg.effort[6] = shared.arm_solver_hold_pose_tau_ctrl_joint_7;
            robot.joint_states_pub->try_publish(robot.joint_states_msg);
        }

        if (!motion_spec::runtime::stop_requested()) {
            robif2b_kinova_gen3_update(&kinova_arm_solver_hold_pose);
            motion_spec::runtime::require_driver(kinova_arm_solver_hold_pose_state.success, "agents.arm1");
            const auto ft_wrist_ft_msr = ft_wrist_ft_io.sample.read();
            motion_spec::io::require_fresh(ft_wrist_ft_io.stale, ft_wrist_ft_msr.seq, ft_wrist_ft_msr.value.success, "arm1.wrist_ft");

        }
        const unsigned int _fsm_state_before = fsm->currentStateIndex;
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
        // The clock is this transition's guard, so this one heartbeat is the cause of the
        // state change -- record it, unlike the 24k that drive nothing.
        if (_fsm_state_before == real_demo_hold_fsm::S_START && fsm->currentStateIndex == real_demo_hold_fsm::S_HOLD) {
            events.record(0);
        }
        static_cast<void>(_fsm_state_before);
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
        motion_spec::introspection::sample_model(
            _introspection_pub,
            static_cast<int>(fsm->currentStateIndex),
            fsm_active_motion(fsm),
            motion_spec::runtime::monotonic_time_s(),
            static_cast<std::uint64_t>(_step),
            shared,
            events);
#endif
        ++_step;
        motion_spec::runtime::sleep_until_next(_next_tick, 1000000);
    }
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
    _introspection_pub.close();
#endif

    robif2b_kinova_gen3_stop(&kinova_arm_solver_hold_pose);
    robif2b_kinova_gen3_shutdown(&kinova_arm_solver_hold_pose);
    ft_wrist_ft_io.stop();
    robif2b_robotiq_ft_shutdown(&ft_wrist_ft);

    real_demo_hold_fsm::destroy_fsm(fsm);

    if (robot.ros_executor) {
        robot.ros_executor->cancel();
    }
    if (robot.ros_spin_thread.joinable()) {
        robot.ros_spin_thread.join();
    }
    robot.joint_states_pub.reset();
    robot.ros_executor.reset();
    robot.ros_node.reset();
    rclcpp::shutdown();

    if (motion_spec::runtime::stop_requested()) {
        std::cerr << "[run] interrupted; frame log closed at step " << _step << std::endl;
        return 130;
    }
    return 0;
}
