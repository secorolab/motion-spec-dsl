#include "runtime.hpp"
#include "shared_state.hpp"
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
#include "introspect_model.hpp"
#endif
#include "motion_home.hpp"
#include "motion_touchdown.hpp"
#include "motion_forward.hpp"
#include "motion_arc_motion.hpp"
#include "motion_compliance.hpp"

#include <cstring>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include "robot_config.hpp"

#include <unistd.h>

static void step_motion_home(motion_home_state &motion_home_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_home_state_instance.active) {
        motion_home_state_instance.active = true;
        motion_home_state_instance.active_steps = 0;
        motion_home_state_instance.snapshot_taken = false;
        motion_home_state_instance.ctrl_hold_position_lin_x.reset();
        motion_home_state_instance.ctrl_hold_position_lin_y.reset();
        motion_home_state_instance.ctrl_hold_position_lin_z.reset();
        motion_home_state_instance.ctrl_hold_orientation_ang_x.reset();
        motion_home_state_instance.ctrl_hold_orientation_ang_y.reset();
        motion_home_state_instance.ctrl_hold_orientation_ang_z.reset();
    }
    update_motion_home(motion_home_state_instance, shared, robot);
    monitor_until_motion_home(motion_home_state_instance, shared, robot, events);
    control_motion_home(motion_home_state_instance, shared, robot);
    apply_motion_home(motion_home_state_instance, robot);
    ++motion_home_state_instance.active_steps;
}

static void step_motion_touchdown(motion_touchdown_state &motion_touchdown_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_touchdown_state_instance.active) {
        motion_touchdown_state_instance.active = true;
        motion_touchdown_state_instance.active_steps = 0;
        motion_touchdown_state_instance.snapshot_taken = false;
        motion_touchdown_state_instance.ctrl_touchdown_x.reset();
        motion_touchdown_state_instance.ctrl_touchdown_y.reset();
        motion_touchdown_state_instance.ctrl_touchdown_z.reset();
        motion_touchdown_state_instance.ctrl_touchdown_ori_ang_x.reset();
        motion_touchdown_state_instance.ctrl_touchdown_ori_ang_y.reset();
        motion_touchdown_state_instance.ctrl_touchdown_ori_ang_z.reset();
    }
    update_motion_touchdown(motion_touchdown_state_instance, shared, robot);
    monitor_until_motion_touchdown(motion_touchdown_state_instance, shared, robot, events);
    control_motion_touchdown(motion_touchdown_state_instance, shared, robot);
    apply_motion_touchdown(motion_touchdown_state_instance, robot);
    ++motion_touchdown_state_instance.active_steps;
}

static void step_motion_forward(motion_forward_state &motion_forward_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_forward_state_instance.active) {
        motion_forward_state_instance.active = true;
        motion_forward_state_instance.active_steps = 0;
        motion_forward_state_instance.snapshot_taken = false;
        motion_forward_state_instance.ctrl_move_forward.reset();
        motion_forward_state_instance.ctrl_forward_ori_ang_x.reset();
        motion_forward_state_instance.ctrl_forward_ori_ang_y.reset();
        motion_forward_state_instance.ctrl_forward_ori_ang_z.reset();
    }
    update_motion_forward(motion_forward_state_instance, shared, robot);
    monitor_until_motion_forward(motion_forward_state_instance, shared, robot, events);
    control_motion_forward(motion_forward_state_instance, shared, robot);
    apply_motion_forward(motion_forward_state_instance, robot);
    ++motion_forward_state_instance.active_steps;
}

static void step_motion_arc_motion(motion_arc_motion_state &motion_arc_motion_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_arc_motion_state_instance.active) {
        motion_arc_motion_state_instance.active = true;
        motion_arc_motion_state_instance.active_steps = 0;
        motion_arc_motion_state_instance.snapshot_taken = false;
        motion_arc_motion_state_instance.ctrl_follow_tangent.reset();
        motion_arc_motion_state_instance.ctrl_follow_position_lin_normal_a.reset();
        motion_arc_motion_state_instance.ctrl_follow_position_lin_normal_b.reset();
        motion_arc_motion_state_instance.ctrl_arc_orientation_ang_x.reset();
        motion_arc_motion_state_instance.ctrl_arc_orientation_ang_y.reset();
        motion_arc_motion_state_instance.ctrl_arc_orientation_ang_z.reset();
        shared.arc_path_s = 0.0;
        shared.arc_path_along_speed = 0.0;
    }
    update_motion_arc_motion(motion_arc_motion_state_instance, shared, robot);
    monitor_until_motion_arc_motion(motion_arc_motion_state_instance, shared, robot, events);
    control_motion_arc_motion(motion_arc_motion_state_instance, shared, robot, events);
    apply_motion_arc_motion(motion_arc_motion_state_instance, robot);
    ++motion_arc_motion_state_instance.active_steps;
}

static void step_motion_compliance(motion_compliance_state &motion_compliance_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_compliance_state_instance.active) {
        motion_compliance_state_instance.active = true;
        motion_compliance_state_instance.active_steps = 0;
        motion_compliance_state_instance.snapshot_taken = false;
        motion_compliance_state_instance.ctrl_comply_x.reset();
        motion_compliance_state_instance.ctrl_comply_y.reset();
        motion_compliance_state_instance.ctrl_comply_z.reset();
        motion_compliance_state_instance.ctrl_comply_ori_ang_x.reset();
        motion_compliance_state_instance.ctrl_comply_ori_ang_y.reset();
        motion_compliance_state_instance.ctrl_comply_ori_ang_z.reset();
    }
    update_motion_compliance(motion_compliance_state_instance, shared, robot);
    monitor_until_motion_compliance(motion_compliance_state_instance, shared, robot, events);
    control_motion_compliance(motion_compliance_state_instance, shared, robot);
    apply_motion_compliance(motion_compliance_state_instance, robot);
    ++motion_compliance_state_instance.active_steps;
}

int main(int argc, char **argv) {
    motion_spec::runtime::install_stop_handlers();
    robot_io robot{};
    shared_data shared{};
    motion_spec_event_buffer events{};

    rclcpp::init(argc, argv);
    robot.ros_node = std::make_shared<rclcpp::Node>("motion_spec_monitor");
    robot.ros_node->declare_parameter<std::string>("scenario_context_id", "");
    robot.ros_param_handle = robot.ros_node->add_on_set_parameters_callback(
        [&robot](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : params) {
                if (param.get_name() != "scenario_context_id") continue;
                std::array<std::uint8_t, 16> parsed{};
                if (!motion_spec::ros::parse_uuid(param.as_string(), parsed)) {
                    result.successful = false;
                    result.reason = "scenario_context_id must be a hex UUID";
                    continue;
                }
                const std::lock_guard<std::mutex> lock(robot.ros_context_id_mutex);
                robot.ros_context_id = parsed;
            }
            return result;
        });
    robot.mon_forward_done_pub = std::make_shared<realtime_tools::RealtimePublisher<bdd_ros2_interfaces::msg::TrinaryStamped>>(
        robot.ros_node->create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>("/motion/forward_done", rclcpp::QoS(10)));

    const auto robot_config = motion_spec::config::load("/home/batsy/work/ms/src/motion-spec-dsl/models/admittance_arc_single/robot.toml");
    bool headless = false;
    bool steps_limited = false;
    int headless_steps = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
            headless_steps = std::atoi(argv[++i]);
            steps_limited = true;
        }
    }

    const auto cache_root = []() -> std::filesystem::path {
        if (const char *xdg = std::getenv("XDG_CACHE_HOME"); xdg && *xdg) {
            return std::filesystem::path(xdg) / "mj_kdl_wrapper";
        }
        if (const char *home = std::getenv("HOME"); home && *home) {
            return std::filesystem::path(home) / ".cache" / "mj_kdl_wrapper";
        }
        return {};
    };

    const auto find_asset_path = [&cache_root](const std::vector<std::filesystem::path> &relative_paths) -> std::filesystem::path {
        std::vector<std::filesystem::path> roots = {std::filesystem::current_path()};
        roots.push_back(std::filesystem::path(__FILE__).parent_path());
        const auto cache = cache_root();
        const auto append_cache_candidate = [&cache](std::vector<std::filesystem::path> &candidates,
                                                     const std::filesystem::path &relative) {
            if (cache.empty()) {
                return;
            }
            const auto text = relative.generic_string();
            const auto append_suffix = [&](const std::string &marker, const std::filesystem::path &cache_subdir) {
                const auto pos = text.find(marker);
                if (pos != std::string::npos) {
                    candidates.push_back(cache / cache_subdir / text.substr(pos + marker.size()));
                }
            };
            append_suffix("third_party/menagerie/", "menagerie");
            append_suffix("src/mj_kdl_wrapper/assets/", "assets");
            append_suffix("src/examples/assets/", "assets");
        };

        for (const auto &start : roots) {
            for (auto root = start; !root.empty(); root = root.parent_path()) {
                for (const auto &relative : relative_paths) {
                    if (relative.is_absolute() && std::filesystem::exists(relative)) {
                        return relative;
                    }
                    const auto candidate = root / relative;
                    if (std::filesystem::exists(candidate)) {
                        return candidate;
                    }
                }
                if (root == root.root_path()) {
                    break;
                }
            }
        }
        for (const auto &relative : relative_paths) {
            std::vector<std::filesystem::path> cache_candidates;
            if (const char *menagerie = std::getenv("MJ_KDL_MENAGERIE"); menagerie && *menagerie) {
                const auto text = relative.generic_string();
                const auto pos = text.find("third_party/menagerie/");
                if (pos != std::string::npos) {
                    cache_candidates.push_back(std::filesystem::path(menagerie) / text.substr(pos + std::string("third_party/menagerie/").size()));
                }
            }
            append_cache_candidate(cache_candidates, relative);
            for (const auto &candidate : cache_candidates) {
                if (std::filesystem::exists(candidate)) {
                    return candidate;
                }
            }
        }
        return {};
    };

    mj_kdl::Env mj_env{};
    mj_kdl::SceneSpec mj_scene{};
    mj_scene.timestep = 0.001;
    mj_scene.add_floor = true;
    mj_scene.add_skybox = true;
    std::deque<std::string> mj_path_storage;
    {
        const std::filesystem::path robot_mjcf_path = find_asset_path({"src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml"});
        if (robot_mjcf_path.empty()) {
            std::cerr << "MJCF model not found: src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml\n";
            return 1;
        }
        mj_path_storage.push_back(robot_mjcf_path.string());
        mj_kdl::RobotSpec mj_robot_spec{};
        mj_robot_spec.path = mj_path_storage.back().c_str();
        mj_robot_spec.prefix = "";
        mj_robot_spec.attach_to.kind = mj_kdl::AttachKind::Site;
        mj_robot_spec.attach_to.name = "table_table_top";
        mj_robot_spec.pos[0] = 0.0;
        mj_robot_spec.pos[1] = 0.0;
        mj_robot_spec.pos[2] = 0.0;
        mj_robot_spec.quat[0] = 0.0;
        mj_robot_spec.quat[1] = 0.0;
        mj_robot_spec.quat[2] = 0.0;
        mj_robot_spec.quat[3] = 1.0;
        {
            const std::filesystem::path attachment_path = find_asset_path({"src/mj_kdl_wrapper/assets/ft_sensor.xml"});
            if (attachment_path.empty()) {
                std::cerr << "MJCF attachment not found: src/mj_kdl_wrapper/assets/ft_sensor.xml\n";
                return 1;
            }
            mj_path_storage.push_back(attachment_path.string());
            mj_kdl::AttachmentSpec attachment_spec{};
            attachment_spec.mjcf_path = mj_path_storage.back().c_str();
            attachment_spec.attach_to.kind = mj_kdl::AttachKind::Site;
            attachment_spec.attach_to.name = "pinch_site";
            attachment_spec.prefix = "";
            attachment_spec.pos[0] = 0.0;
            attachment_spec.pos[1] = 0.0;
            attachment_spec.pos[2] = 0.0;
            attachment_spec.quat[0] = 0.0;
            attachment_spec.quat[1] = 0.0;
            attachment_spec.quat[2] = 0.0;
            attachment_spec.quat[3] = 1.0;
            mj_robot_spec.attachments.push_back(attachment_spec);
        }

        {
            const std::filesystem::path attachment_path = find_asset_path({"src/mj_kdl_wrapper/assets/robotiq_2f85/2f85.xml"});
            if (attachment_path.empty()) {
                std::cerr << "MJCF attachment not found: src/mj_kdl_wrapper/assets/robotiq_2f85/2f85.xml\n";
                return 1;
            }
            mj_path_storage.push_back(attachment_path.string());
            mj_kdl::AttachmentSpec attachment_spec{};
            attachment_spec.mjcf_path = mj_path_storage.back().c_str();
            attachment_spec.attach_to.kind = mj_kdl::AttachKind::Site;
            attachment_spec.attach_to.name = "wrist_ft_site";
            attachment_spec.prefix = "g_";
            attachment_spec.pos[0] = 0.0;
            attachment_spec.pos[1] = 0.0;
            attachment_spec.pos[2] = 0.0;
            attachment_spec.quat[0] = 0.0;
            attachment_spec.quat[1] = 0.0;
            attachment_spec.quat[2] = 0.0;
            attachment_spec.quat[3] = 1.0;
            mj_robot_spec.attachments.push_back(attachment_spec);
        }

        mj_scene.robots.push_back(mj_robot_spec);
    }

    {
        mj_kdl::SceneObject object_spec{};
        object_spec.name = "table";
        object_spec.attach_to.kind = mj_kdl::AttachKind::World;
        object_spec.attach_to.name = "";
        const std::filesystem::path object_path = find_asset_path({"src/mj_kdl_wrapper/assets/table.xml"});
        if (object_path.empty()) {
            std::cerr << "MJCF object not found: src/mj_kdl_wrapper/assets/table.xml\n";
            return 1;
        }
        object_spec.mjcf_path = object_path.string();
        object_spec.pos[0] = 0.0;
        object_spec.pos[1] = 0.0;
        object_spec.pos[2] = 0.72;
        object_spec.quat[0] = 0.0;
        object_spec.quat[1] = 0.0;
        object_spec.quat[2] = 0.0;
        object_spec.quat[3] = 1.0;
        object_spec.fixed = true;
        mj_scene.objects.push_back(object_spec);
    }

    if (!mj_kdl::init_env(&mj_env, &mj_scene)) {
        std::cerr << "mj_kdl::init_env() failed\n";
        return 1;
    }

    // Contact-solver policy: accuracy and grip fidelity bought with sim time per step.
    constexpr int kMjSolverIterations = 100;
    constexpr double kMjSolverTolerance = 1e-10;
    constexpr double kMjImpratio = 20.0;
    mj_env.model->opt.iterations = kMjSolverIterations;
    mj_env.model->opt.tolerance  = kMjSolverTolerance;
    mj_env.model->opt.impratio   = kMjImpratio;
    mj_env.model->opt.cone       = mjCONE_ELLIPTIC;

    mj_kdl::Robot mj_arm_solver_home{};
    mj_kdl::ToolFrameSpec mj_tool_arm_solver_home{};
    mj_tool_arm_solver_home.ft_sensors.push_back(mj_kdl::ForceTorqueSensorSpec{ .name = "wrist_ft", .frame_site = "wrist_ft_site" });
    KDL::Tree built_tree_arm_solver_home{};
    if (!scene_kdl::make_tree_world_tree(&built_tree_arm_solver_home)) {
        std::cerr << "make_tree_world_tree() failed for runtime arm_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    KDL::Chain built_chain_arm_solver_home{};
    if (!scene_kdl::make_chain_kinova_ft_2f85_chain(built_tree_arm_solver_home, &built_chain_arm_solver_home)) {
        std::cerr << "make_chain_kinova_ft_2f85_chain() failed for runtime arm_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    if (!mj_kdl::init_robot_from_chain(&mj_arm_solver_home, mj_env.model, mj_env.data, built_chain_arm_solver_home, {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"}, "", &mj_tool_arm_solver_home)) {
        std::cerr << "init_robot_from_chain() failed for runtime arm_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    mj_arm_solver_home.ctrl_mode = mj_kdl::CtrlMode::TORQUE;
    mj_kdl::env_add_robot(&mj_env, &mj_arm_solver_home);

    KDL::Chain *chain_arm_solver_home = &mj_arm_solver_home.chain;

    robot.arm_solver_home = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };
    robot.ft_wrist_ft_bias_samples = motion_spec::config::bias_samples(robot_config, "kinova_ft_2f85.wrist_ft");

    robot.arm_solver_touchdown = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_forward = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.rne_arm_solver_arc_motion = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.rne_arm_solver_compliance = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    mj_kdl::Viewer mj_viewer{};
    if (!headless) {
        if (!mj_kdl::init_window_sim(&mj_viewer, &mj_arm_solver_home)) {
            std::cerr << "mj_kdl::init_window_sim() failed\n";
            mj_kdl::cleanup(&mj_env);
            return 1;
        }
    }
    KDL::Wrench ext_force_measurement;
    robot.ext_force = &ext_force_measurement;
    motion_home_state motion_home_state_instance;
    motion_touchdown_state motion_touchdown_state_instance;
    motion_forward_state motion_forward_state_instance;
    motion_arc_motion_state motion_arc_motion_state_instance;
    motion_compliance_state motion_compliance_state_instance;

    mj_env.on_reset = [&](mj_kdl::ResetContext *) {
        {
            const double _q_home[] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
            KDL::JntArray _q(mj_arm_solver_home.n_joints);
            for (int _i = 0; _i < mj_arm_solver_home.n_joints && _i < static_cast<int>(std::size(_q_home)); ++_i)
                _q(_i) = _q_home[_i];
            mj_kdl::set_joint_pos(&mj_arm_solver_home, _q, false);
        }

        reset_motion_home(motion_home_state_instance);
                reset_motion_touchdown(motion_touchdown_state_instance);
                reset_motion_forward(motion_forward_state_instance);
                reset_motion_arc_motion(motion_arc_motion_state_instance);
                reset_motion_compliance(motion_compliance_state_instance);
    };
    mj_kdl::reset(&mj_env);
    double _prev_sim_time = mj_arm_solver_home.data->time;
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

    struct fsm_nbx *fsm = admittance_arc_single_fsm::create_fsm();
    if (!fsm) return 1;
    robot.fsm_events = fsm->eventData;

    auto fsm_active_motion = [&](struct fsm_nbx *fsm) -> int {
        switch (fsm->currentStateIndex) {
        case admittance_arc_single_fsm::S_HOME: return 0;

        case admittance_arc_single_fsm::S_TOUCHDOWN: return 1;

        case admittance_arc_single_fsm::S_FORWARD: return 2;

        case admittance_arc_single_fsm::S_ARC: return 3;

        case admittance_arc_single_fsm::S_ADMITTANCE: return 4;

        default: return -1;
        }
    };
    // The FSM's current state selects which motion runs; the motion's monitor advances the FSM. A
    // fallback hold state additionally re-evaluates the gated motion's WHEN precondition each tick.
    auto fsm_dispatch = [&](struct fsm_nbx *fsm) {
        switch (fsm->currentStateIndex) {
        case admittance_arc_single_fsm::S_HOME:
            step_motion_home(motion_home_state_instance, shared, robot, events);
            break;

        case admittance_arc_single_fsm::S_TOUCHDOWN:
            step_motion_touchdown(motion_touchdown_state_instance, shared, robot, events);
            break;

        case admittance_arc_single_fsm::S_FORWARD:
            step_motion_forward(motion_forward_state_instance, shared, robot, events);
            break;

        case admittance_arc_single_fsm::S_ARC:
            step_motion_arc_motion(motion_arc_motion_state_instance, shared, robot, events);
            break;

        case admittance_arc_single_fsm::S_ADMITTANCE:
            step_motion_compliance(motion_compliance_state_instance, shared, robot, events);
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
    std::array<std::uint8_t, 16> _scenario_context_id{};

    while (!motion_spec::runtime::stop_requested() && ((!headless || !steps_limited || headless_steps-- > 0)) && rclcpp::ok()) {
        shared.clock_time_s = robot.arm_solver_home.robot->data->time;
        // Integrators step with the time that actually elapsed, not the period we asked for: the
        // loop sleeps to deadlines it can overrun. In sim the clock advances exactly one timestep
        // per tick, so this equals the nominal period.
        const double _dt_raw = (_step == 0)
            ? motion_spec::runtime::kControlPeriodS
            : shared.clock_time_s - _prev_clock_s;
        _prev_clock_s = shared.clock_time_s;
        shared.dt_measured_s = std::clamp(_dt_raw, motion_spec::runtime::kDtClampMin,
                                          motion_spec::runtime::kDtClampMax);
        if (mj_arm_solver_home.data->time < _prev_sim_time - 1e-6) {
            mj_kdl::reset(&mj_env);
            fsm->currentStateIndex = fsm->startStateIndex;
        }
        _prev_sim_time = mj_arm_solver_home.data->time;
        if (static_cast<int>(fsm->currentStateIndex) != _fsm_prev_state) {
            _fsm_prev_state = static_cast<int>(fsm->currentStateIndex);
            std::cerr << "[fsm] state   " << admittance_arc_single_fsm::STATE_URIS[fsm->currentStateIndex] << std::endl;
            // Leaving a state ends its motion's activation, so re-entering one is a fresh
            // start: active_steps returns to 0 and any trajectory replans from the current
            // pose instead of resuming the alpha it had when it was interrupted.
            motion_home_state_instance.active = false;
            motion_touchdown_state_instance.active = false;
            motion_forward_state_instance.active = false;
            motion_arc_motion_state_instance.active = false;
            motion_compliance_state_instance.active = false;
        }
        if (fsm->currentStateIndex == fsm->endStateIndex) {
            break;
        }
        // The heartbeat drives the FSM but is not recorded: the frame log is a time series, so
        // every frame already *is* the tick (t, step, wall_ns, period_ns). Logging a per-tick
        // event would restate that 24k times and leave last_event permanently pinned to it.
        produce_event(fsm->eventData, admittance_arc_single_fsm::E_STEP);
        fsm_dispatch(fsm);
        if (robot.ros_context_id_mutex.try_lock()) {
            _scenario_context_id = robot.ros_context_id;
            robot.ros_context_id_mutex.unlock();
        }
        if (motion_forward_state_instance.active) {
            const bool _satisfied = motion_spec::runtime::constraint_satisfied(shared.eval_forward_until_reached_forward_err, shared.default_tolerance_Distance);
            bool _authored = false;
            if (_satisfied) {
                robot.mon_forward_done_pub_msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;
                _authored = true;
            }
            if (!_satisfied) {
                robot.mon_forward_done_pub_msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::FALSE;
                _authored = true;
            }
            if (_authored) {
                robot.mon_forward_done_pub_msg.stamp = robot.ros_node->now();
                robot.mon_forward_done_pub_msg.scenario_context_id.uuid = _scenario_context_id;
                robot.mon_forward_done_pub->try_publish(robot.mon_forward_done_pub_msg);
            }
        }

        if (robot.joint_states_pub && (_step % robot.joint_states_divider) == 0) {
            robot.joint_states_msg.header.stamp = robot.ros_node->now();
            robot.joint_states_msg.position[0] = shared.arm_solver_home_q_joint_1;
            robot.joint_states_msg.velocity[0] = shared.arm_solver_home_qd_joint_1;
            robot.joint_states_msg.effort[0] = shared.arm_solver_home_tau_ctrl_joint_1;
            robot.joint_states_msg.position[1] = shared.arm_solver_home_q_joint_2;
            robot.joint_states_msg.velocity[1] = shared.arm_solver_home_qd_joint_2;
            robot.joint_states_msg.effort[1] = shared.arm_solver_home_tau_ctrl_joint_2;
            robot.joint_states_msg.position[2] = shared.arm_solver_home_q_joint_3;
            robot.joint_states_msg.velocity[2] = shared.arm_solver_home_qd_joint_3;
            robot.joint_states_msg.effort[2] = shared.arm_solver_home_tau_ctrl_joint_3;
            robot.joint_states_msg.position[3] = shared.arm_solver_home_q_joint_4;
            robot.joint_states_msg.velocity[3] = shared.arm_solver_home_qd_joint_4;
            robot.joint_states_msg.effort[3] = shared.arm_solver_home_tau_ctrl_joint_4;
            robot.joint_states_msg.position[4] = shared.arm_solver_home_q_joint_5;
            robot.joint_states_msg.velocity[4] = shared.arm_solver_home_qd_joint_5;
            robot.joint_states_msg.effort[4] = shared.arm_solver_home_tau_ctrl_joint_5;
            robot.joint_states_msg.position[5] = shared.arm_solver_home_q_joint_6;
            robot.joint_states_msg.velocity[5] = shared.arm_solver_home_qd_joint_6;
            robot.joint_states_msg.effort[5] = shared.arm_solver_home_tau_ctrl_joint_6;
            robot.joint_states_msg.position[6] = shared.arm_solver_home_q_joint_7;
            robot.joint_states_msg.velocity[6] = shared.arm_solver_home_qd_joint_7;
            robot.joint_states_msg.effort[6] = shared.arm_solver_home_tau_ctrl_joint_7;
            robot.joint_states_pub->try_publish(robot.joint_states_msg);
        }

        mj_kdl::update(&mj_arm_solver_home);

        // Closing the viewer ends the run, but it must end the same way S_DONE does. std::exit
        // skips destructors, so the frame logger's writer thread never drained and the log was
        // left with a half-written record and no health file -- break, and let teardown run.
        if (!mj_kdl::step(&mj_arm_solver_home)) {
            break;
        }
        const unsigned int _fsm_state_before = fsm->currentStateIndex;
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
        // The clock is this transition's guard, so this one heartbeat is the cause of the
        // state change -- record it, unlike the 24k that drive nothing.
        if (_fsm_state_before == admittance_arc_single_fsm::S_START && fsm->currentStateIndex == admittance_arc_single_fsm::S_HOME) {
            events.record(8);
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

    if (!headless) {
        mj_kdl::cleanup(&mj_viewer);
    }
    mj_kdl::cleanup(&mj_env);
    mj_kdl::cleanup(&mj_arm_solver_home);

    admittance_arc_single_fsm::destroy_fsm(fsm);

    if (robot.ros_executor) {
        robot.ros_executor->cancel();
    }
    if (robot.ros_spin_thread.joinable()) {
        robot.ros_spin_thread.join();
    }
    robot.mon_forward_done_pub.reset();
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
