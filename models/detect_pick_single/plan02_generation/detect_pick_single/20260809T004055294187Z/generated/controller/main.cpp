#include "runtime.hpp"
#include "shared_state.hpp"
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
#include "introspect_model.hpp"
#endif
#include "motion_home.hpp"
#include "motion_detect_cube.hpp"
#include "motion_pick_above.hpp"
#include "motion_pick.hpp"
#include "motion_grasp_hold.hpp"
#include "motion_lift.hpp"
#include "motion_place_above.hpp"
#include "motion_place.hpp"
#include "motion_open_grasp_hold.hpp"
#include "motion_retreat.hpp"
#include "motion_recover.hpp"

#include <cstring>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include "robot_config.hpp"

#include <unistd.h>

static void step_motion_home(motion_home_state &motion_home_state_instance, shared_data &shared, robot_io &robot) {
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
    monitor_until_motion_home();
    control_motion_home(motion_home_state_instance, shared, robot);
    apply_motion_home(motion_home_state_instance, robot);
    ++motion_home_state_instance.active_steps;
}

static void step_motion_detect_cube(motion_detect_cube_state &motion_detect_cube_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_detect_cube_state_instance.active) {
        motion_detect_cube_state_instance.active = true;
        motion_detect_cube_state_instance.active_steps = 0;
        motion_detect_cube_state_instance.snapshot_taken = false;
        shared.locate_cube_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        {
            aruco_perception::action::LocateObjects::Goal _goal;
            _goal.target_iris = {
                "https://secorolab.github.io/models/scenes/detect-pick-single-mjc/pick_place_graph/cube/cube_origin"
            };
            rclcpp_action::Client<aruco_perception::action::LocateObjects>::SendGoalOptions _options;
            _options.goal_response_callback =
                [&robot](rclcpp_action::ClientGoalHandle<aruco_perception::action::LocateObjects>::SharedPtr handle) {
                    const std::lock_guard<std::mutex> lock(robot.locate_cube_client_mutex);
                    robot.locate_cube_client_goal = handle;
                    if (!handle) {
                        robot.locate_cube_client_staged_status =
                            action_msgs::msg::GoalStatus::STATUS_ABORTED;
                        robot.locate_cube_client_staged = true;
                    }
                };
            _options.result_callback =
                [&robot](const rclcpp_action::ClientGoalHandle<aruco_perception::action::LocateObjects>::WrappedResult &result) {
                    const std::lock_guard<std::mutex> lock(robot.locate_cube_client_mutex);
                    robot.locate_cube_client_goal.reset();
                    robot.locate_cube_client_staged = true;
                    if (result.code == rclcpp_action::ResultCode::CANCELED) {
                        robot.locate_cube_client_staged_status =
                            action_msgs::msg::GoalStatus::STATUS_CANCELED;
                        return;
                    }
                    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                        robot.locate_cube_client_staged_status =
                            action_msgs::msg::GoalStatus::STATUS_ABORTED;
                        return;
                    }
                    int accepted = 0;
                    for (const auto &detection : result.result->detections.detections) {
                        if (detection.results.empty()) continue;
                        const auto &pose = detection.results[0].pose.pose;
                        if (detection.id == "https://secorolab.github.io/models/scenes/detect-pick-single-mjc/pick_place_graph/cube/cube_origin" && detection.header.frame_id == "base_link") {
                            robot.locate_cube_client_staged_pose_cube_base = KDL::Frame(
                                KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y,
                                                          pose.orientation.z, pose.orientation.w),
                                KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
                            ++accepted;
                        }
                    }
                    robot.locate_cube_client_staged_status =
                        accepted == 1
                            ? action_msgs::msg::GoalStatus::STATUS_SUCCEEDED
                            : action_msgs::msg::GoalStatus::STATUS_ABORTED;
                };
            robot.locate_cube_client->async_send_goal(_goal, _options);
        }
        motion_detect_cube_state_instance.ctrl_dt_hold_position_lin_x.reset();
        motion_detect_cube_state_instance.ctrl_dt_hold_position_lin_y.reset();
        motion_detect_cube_state_instance.ctrl_dt_hold_position_lin_z.reset();
        motion_detect_cube_state_instance.ctrl_dt_hold_orientation_ang_x.reset();
        motion_detect_cube_state_instance.ctrl_dt_hold_orientation_ang_y.reset();
        motion_detect_cube_state_instance.ctrl_dt_hold_orientation_ang_z.reset();
    }
    update_motion_detect_cube(motion_detect_cube_state_instance, shared, robot);
    monitor_until_motion_detect_cube(motion_detect_cube_state_instance, shared, robot, events);
    control_motion_detect_cube(motion_detect_cube_state_instance, shared, robot);
    apply_motion_detect_cube(motion_detect_cube_state_instance, robot);
    ++motion_detect_cube_state_instance.active_steps;
}

static void step_motion_pick_above(motion_pick_above_state &motion_pick_above_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_pick_above_state_instance.active) {
        motion_pick_above_state_instance.active = true;
        motion_pick_above_state_instance.active_steps = 0;
        motion_pick_above_state_instance.snapshot_taken = false;
        motion_pick_above_state_instance.ctrl_pa_follow_tan.reset();
        motion_pick_above_state_instance.ctrl_pa_follow_lat_lin_normal_a.reset();
        motion_pick_above_state_instance.ctrl_pa_follow_lat_lin_normal_b.reset();
        motion_pick_above_state_instance.ctrl_pa_follow_ori_ang_x.reset();
        motion_pick_above_state_instance.ctrl_pa_follow_ori_ang_y.reset();
        motion_pick_above_state_instance.ctrl_pa_follow_ori_ang_z.reset();
        shared.approach_path_s = 0.0;
        shared.approach_path_along_speed = 0.0;
    }
    update_motion_pick_above(motion_pick_above_state_instance, shared, robot);
    monitor_until_motion_pick_above();
    control_motion_pick_above(motion_pick_above_state_instance, shared, robot);
    apply_motion_pick_above(motion_pick_above_state_instance, robot);
    ++motion_pick_above_state_instance.active_steps;
}

static void step_motion_pick(motion_pick_state &motion_pick_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_pick_state_instance.active) {
        motion_pick_state_instance.active = true;
        motion_pick_state_instance.active_steps = 0;
        motion_pick_state_instance.snapshot_taken = false;
        motion_pick_state_instance.ctrl_pk_hold_x.reset();
        motion_pick_state_instance.ctrl_pk_hold_y.reset();
        motion_pick_state_instance.ctrl_pk_lower_z.reset();
        motion_pick_state_instance.ctrl_pk_follow_ori_ang_x.reset();
        motion_pick_state_instance.ctrl_pk_follow_ori_ang_y.reset();
        motion_pick_state_instance.ctrl_pk_follow_ori_ang_z.reset();
        motion_pick_state_instance.ctrl_pk_support_z.reset();
    }
    update_motion_pick(motion_pick_state_instance, shared, robot);
    monitor_until_motion_pick();
    control_motion_pick(motion_pick_state_instance, shared, robot);
    apply_motion_pick(motion_pick_state_instance, robot);
    ++motion_pick_state_instance.active_steps;
}

static void step_motion_grasp_hold(motion_grasp_hold_state &motion_grasp_hold_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_grasp_hold_state_instance.active) {
        motion_grasp_hold_state_instance.active = true;
        motion_grasp_hold_state_instance.active_steps = 0;
        motion_grasp_hold_state_instance.snapshot_taken = false;
        motion_grasp_hold_state_instance.ctrl_cg_hold_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg_hold_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg_hold_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg_hold_orientation_ang_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg_hold_orientation_ang_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg_hold_orientation_ang_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg_support_z.reset();

    }
    update_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot);
    monitor_until_motion_grasp_hold();
    control_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot);
    apply_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot);
    ++motion_grasp_hold_state_instance.active_steps;
}

static void step_motion_lift(motion_lift_state &motion_lift_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_lift_state_instance.active) {
        motion_lift_state_instance.active = true;
        motion_lift_state_instance.active_steps = 0;
        motion_lift_state_instance.snapshot_taken = false;
        motion_lift_state_instance.ctrl_lt_hold_x.reset();
        motion_lift_state_instance.ctrl_lt_hold_y.reset();
        motion_lift_state_instance.ctrl_lt_lift_z.reset();
        motion_lift_state_instance.ctrl_lt_follow_ori_ang_x.reset();
        motion_lift_state_instance.ctrl_lt_follow_ori_ang_y.reset();
        motion_lift_state_instance.ctrl_lt_follow_ori_ang_z.reset();
        motion_lift_state_instance.ctrl_lt_support_z.reset();

    }
    update_motion_lift(motion_lift_state_instance, shared, robot);
    monitor_until_motion_lift(motion_lift_state_instance, shared, robot, events);
    control_motion_lift(motion_lift_state_instance, shared, robot);
    apply_motion_lift(motion_lift_state_instance, shared, robot);
    ++motion_lift_state_instance.active_steps;
}

static void step_motion_place_above(motion_place_above_state &motion_place_above_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_place_above_state_instance.active) {
        motion_place_above_state_instance.active = true;
        motion_place_above_state_instance.active_steps = 0;
        motion_place_above_state_instance.snapshot_taken = false;
        motion_place_above_state_instance.ctrl_pla_reach_x.reset();
        motion_place_above_state_instance.ctrl_pla_reach_y.reset();
        motion_place_above_state_instance.ctrl_pla_reach_z.reset();
        motion_place_above_state_instance.ctrl_pla_align_ori_ang_x.reset();
        motion_place_above_state_instance.ctrl_pla_align_ori_ang_y.reset();
        motion_place_above_state_instance.ctrl_pla_align_ori_ang_z.reset();
        motion_place_above_state_instance.ctrl_pla_support_z.reset();

    }
    update_motion_place_above(motion_place_above_state_instance, shared, robot);
    monitor_until_motion_place_above(motion_place_above_state_instance, shared, robot, events);
    control_motion_place_above(motion_place_above_state_instance, shared, robot);
    apply_motion_place_above(motion_place_above_state_instance, shared, robot);
    ++motion_place_above_state_instance.active_steps;
}

static void step_motion_place(motion_place_state &motion_place_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_place_state_instance.active) {
        motion_place_state_instance.active = true;
        motion_place_state_instance.active_steps = 0;
        motion_place_state_instance.snapshot_taken = false;
        motion_place_state_instance.ctrl_pl_reach_x.reset();
        motion_place_state_instance.ctrl_pl_reach_y.reset();
        motion_place_state_instance.ctrl_pl_reach_z.reset();
        motion_place_state_instance.ctrl_pl_align_ori_ang_x.reset();
        motion_place_state_instance.ctrl_pl_align_ori_ang_y.reset();
        motion_place_state_instance.ctrl_pl_align_ori_ang_z.reset();
        motion_place_state_instance.ctrl_pl_support_z.reset();

    }
    update_motion_place(motion_place_state_instance, shared, robot);
    monitor_until_motion_place(motion_place_state_instance, shared, robot, events);
    control_motion_place(motion_place_state_instance, shared, robot);
    apply_motion_place(motion_place_state_instance, shared, robot);
    ++motion_place_state_instance.active_steps;
}

static void step_motion_open_grasp_hold(motion_open_grasp_hold_state &motion_open_grasp_hold_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_open_grasp_hold_state_instance.active) {
        motion_open_grasp_hold_state_instance.active = true;
        motion_open_grasp_hold_state_instance.active_steps = 0;
        motion_open_grasp_hold_state_instance.snapshot_taken = false;
        motion_open_grasp_hold_state_instance.ctrl_og_hold_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_hold_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_hold_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_hold_orientation_ang_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_hold_orientation_ang_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_hold_orientation_ang_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og_support_z.reset();

    }
    update_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot);
    monitor_until_motion_open_grasp_hold();
    control_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot);
    apply_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot);
    ++motion_open_grasp_hold_state_instance.active_steps;
}

static void step_motion_retreat(motion_retreat_state &motion_retreat_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_retreat_state_instance.active) {
        motion_retreat_state_instance.active = true;
        motion_retreat_state_instance.active_steps = 0;
        motion_retreat_state_instance.snapshot_taken = false;
        motion_retreat_state_instance.ctrl_ret_reach_x.reset();
        motion_retreat_state_instance.ctrl_ret_reach_y.reset();
        motion_retreat_state_instance.ctrl_ret_reach_z.reset();
        motion_retreat_state_instance.ctrl_ret_align_ori_ang_x.reset();
        motion_retreat_state_instance.ctrl_ret_align_ori_ang_y.reset();
        motion_retreat_state_instance.ctrl_ret_align_ori_ang_z.reset();
        motion_retreat_state_instance.ctrl_ret_support_z.reset();

    }
    update_motion_retreat(motion_retreat_state_instance, shared, robot);
    monitor_until_motion_retreat(motion_retreat_state_instance, shared, robot, events);
    control_motion_retreat(motion_retreat_state_instance, shared, robot);
    apply_motion_retreat(motion_retreat_state_instance, shared, robot);
    ++motion_retreat_state_instance.active_steps;
}

static void step_motion_recover(motion_recover_state &motion_recover_state_instance, shared_data &shared, robot_io &robot, motion_spec_event_buffer &events) {
    if (!motion_recover_state_instance.active) {
        motion_recover_state_instance.active = true;
        motion_recover_state_instance.active_steps = 0;
        motion_recover_state_instance.snapshot_taken = false;
        motion_recover_state_instance.ctrl_rc_hold_x.reset();
        motion_recover_state_instance.ctrl_rc_hold_y.reset();
        motion_recover_state_instance.ctrl_rc_hold_z.reset();
        motion_recover_state_instance.ctrl_rc_hold_orientation_ang_x.reset();
        motion_recover_state_instance.ctrl_rc_hold_orientation_ang_y.reset();
        motion_recover_state_instance.ctrl_rc_hold_orientation_ang_z.reset();
        motion_recover_state_instance.ctrl_rc_support_z.reset();
    }
    update_motion_recover(motion_recover_state_instance, shared, robot);
    monitor_until_motion_recover();
    control_motion_recover(motion_recover_state_instance, shared, robot, events);
    apply_motion_recover(motion_recover_state_instance, robot);
    ++motion_recover_state_instance.active_steps;
}

int main(int argc, char **argv) {
    motion_spec::runtime::install_stop_handlers();
    robot_io robot{};
    shared_data shared{};
    motion_spec_event_buffer events{};

    rclcpp::init(argc, argv);
    robot.ros_node = std::make_shared<rclcpp::Node>("motion_spec_monitor");
    robot.locate_cube_client = rclcpp_action::create_client<aruco_perception::action::LocateObjects>(robot.ros_node, "/perception/locate");

    robot.bhv_events_pub =
        std::make_shared<realtime_tools::RealtimePublisher<bdd_ros2_interfaces::msg::Event>>(
            robot.ros_node->create_publisher<bdd_ros2_interfaces::msg::Event>(
                "/bdd/events", rclcpp::QoS(10)));
    robot.bhv_server = rclcpp_action::create_server<motion_spec::ros::BehaviourAction>(
        robot.ros_node,
        "pick_place",
        [&robot](const rclcpp_action::GoalUUID &,
                 std::shared_ptr<const motion_spec::ros::BehaviourAction::Goal> goal) {
            if (robot.bhv_goal_taken.exchange(true)) {
                RCLCPP_WARN(robot.ros_node->get_logger(),
                            "a behaviour goal was already served this run; rejecting");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (!goal->parameters.empty() || !goal->configs.empty()) {
                RCLCPP_WARN(robot.ros_node->get_logger(),
                            "ignoring %zu goal parameters and %zu goal configs",
                            goal->parameters.size(), goal->configs.size());
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<motion_spec::ros::BehaviourGoalHandle>) {
            // Accepted so the goal ends up CANCELED, but nothing is routed into the FSM: the run
            // plays out to its end state either way.
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&robot](const std::shared_ptr<motion_spec::ros::BehaviourGoalHandle> handle) {
            const std::lock_guard<std::mutex> lock(robot.bhv_goal_mutex);
            robot.bhv_staged_goal = handle;
        });

    const auto robot_config = motion_spec::config::load("/home/batsy/work/ms/src/motion-spec-dsl/models/detect_pick_single/robot.toml");
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
        object_spec.name = "cube";
        object_spec.attach_to.kind = mj_kdl::AttachKind::World;
        object_spec.attach_to.name = "";
        const std::filesystem::path object_path = find_asset_path({"src/mj_kdl_wrapper/assets/cube.xml"});
        if (object_path.empty()) {
            std::cerr << "MJCF object not found: src/mj_kdl_wrapper/assets/cube.xml\n";
            return 1;
        }
        object_spec.mjcf_path = object_path.string();
        object_spec.pos[0] = 0.5;
        object_spec.pos[1] = 0.0;
        object_spec.pos[2] = 0.76;
        object_spec.quat[0] = 0.0;
        object_spec.quat[1] = 0.0;
        object_spec.quat[2] = 0.0;
        object_spec.quat[3] = 1.0;
        object_spec.fixed = false;
        mj_scene.objects.push_back(object_spec);
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
    if (!scene_kdl::make_chain_kinova_2f85_chain(built_tree_arm_solver_home, &built_chain_arm_solver_home)) {
        std::cerr << "make_chain_kinova_2f85_chain() failed for runtime arm_solver_home\n";
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
    robot.ft_wrist_ft_bias_samples = motion_spec::config::bias_samples(robot_config, "arm1.wrist_ft");

    robot.arm_solver_detect_cube = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_pick_above = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_pick = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_grasp_hold = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_lift = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_place_above = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_place = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_open_grasp_hold = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_retreat = manipulator_robot{
        .robot = &mj_arm_solver_home,
        .chain = chain_arm_solver_home,
    };

    robot.arm_solver_recover = manipulator_robot{
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
    motion_home_state motion_home_state_instance;
    motion_detect_cube_state motion_detect_cube_state_instance;
    motion_pick_above_state motion_pick_above_state_instance;
    motion_pick_state motion_pick_state_instance;
    motion_grasp_hold_state motion_grasp_hold_state_instance;
    motion_lift_state motion_lift_state_instance;
    motion_place_above_state motion_place_above_state_instance;
    motion_place_state motion_place_state_instance;
    motion_open_grasp_hold_state motion_open_grasp_hold_state_instance;
    motion_retreat_state motion_retreat_state_instance;
    motion_recover_state motion_recover_state_instance;

    mj_env.on_reset = [&](mj_kdl::ResetContext *) {
        {
            const double _q_home[] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
            KDL::JntArray _q(mj_arm_solver_home.n_joints);
            for (int _i = 0; _i < mj_arm_solver_home.n_joints && _i < static_cast<int>(std::size(_q_home)); ++_i)
                _q(_i) = _q_home[_i];
            mj_kdl::set_joint_pos(&mj_arm_solver_home, _q, false);
        }

        reset_motion_home(motion_home_state_instance);
                reset_motion_detect_cube(motion_detect_cube_state_instance);
                reset_motion_pick_above(motion_pick_above_state_instance);
                reset_motion_pick(motion_pick_state_instance);
                reset_motion_grasp_hold(motion_grasp_hold_state_instance);
                reset_motion_lift(motion_lift_state_instance);
                reset_motion_place_above(motion_place_above_state_instance);
                reset_motion_place(motion_place_state_instance);
                reset_motion_open_grasp_hold(motion_open_grasp_hold_state_instance);
                reset_motion_retreat(motion_retreat_state_instance);
                reset_motion_recover(motion_recover_state_instance);
    };
    mj_kdl::reset(&mj_env);
    double _prev_sim_time = mj_arm_solver_home.data->time;
    robot.ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    robot.ros_executor->add_node(robot.ros_node);
    robot.ros_spin_thread = std::thread([&robot]() { robot.ros_executor->spin(); });

    struct fsm_nbx *fsm = detect_pick_single_fsm::create_fsm();
    if (!fsm) return 1;
    robot.fsm_events = fsm->eventData;

    auto fsm_active_motion = [&](struct fsm_nbx *fsm) -> int {
        switch (fsm->currentStateIndex) {
        case detect_pick_single_fsm::S_HOME: return 0;

        case detect_pick_single_fsm::S_DETECT: return 1;

        case detect_pick_single_fsm::S_PICK_ABOVE: return 2;

        case detect_pick_single_fsm::S_PICK: return 3;

        case detect_pick_single_fsm::S_GRASP: return 4;

        case detect_pick_single_fsm::S_LIFT: return 5;

        case detect_pick_single_fsm::S_PLACE_ABOVE: return 6;

        case detect_pick_single_fsm::S_PLACE: return 7;

        case detect_pick_single_fsm::S_OPEN: return 8;

        case detect_pick_single_fsm::S_RETREAT: return 9;

        case detect_pick_single_fsm::S_RECOVER: return 10;

        default: return -1;
        }
    };
    // The FSM's current state selects which motion runs; the motion's monitor advances the FSM. A
    // fallback hold state additionally re-evaluates the gated motion's WHEN precondition each tick.
    auto fsm_dispatch = [&](struct fsm_nbx *fsm) {
        switch (fsm->currentStateIndex) {
        case detect_pick_single_fsm::S_HOME:
            step_motion_home(motion_home_state_instance, shared, robot);
            monitor_when_motion_detect_cube(motion_detect_cube_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_DETECT:
            step_motion_detect_cube(motion_detect_cube_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_PICK_ABOVE:
            step_motion_pick_above(motion_pick_above_state_instance, shared, robot);
            monitor_when_motion_pick(motion_pick_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_PICK:
            step_motion_pick(motion_pick_state_instance, shared, robot);
            monitor_when_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_GRASP:
            step_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot);
            monitor_when_motion_lift(motion_lift_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_LIFT:
            step_motion_lift(motion_lift_state_instance, shared, robot, events);
            monitor_when_motion_place_above(motion_place_above_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_PLACE_ABOVE:
            step_motion_place_above(motion_place_above_state_instance, shared, robot, events);
            monitor_when_motion_place(motion_place_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_PLACE:
            step_motion_place(motion_place_state_instance, shared, robot, events);
            monitor_when_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_OPEN:
            step_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot);
            monitor_when_motion_retreat(motion_retreat_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_RETREAT:
            step_motion_retreat(motion_retreat_state_instance, shared, robot, events);
            break;

        case detect_pick_single_fsm::S_RECOVER:
            step_motion_recover(motion_recover_state_instance, shared, robot, events);
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
    std::shared_ptr<motion_spec::ros::BehaviourGoalHandle> _bhv_goal = nullptr;
    bool _bhv_goal_armed = false;

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
            std::cerr << "[fsm] state   " << detect_pick_single_fsm::STATE_URIS[fsm->currentStateIndex] << std::endl;
            // Leaving a state ends its motion's activation, so re-entering one is a fresh
            // start: active_steps returns to 0 and any trajectory replans from the current
            // pose instead of resuming the alpha it had when it was interrupted.
            if (motion_detect_cube_state_instance.active) {
            {
                const std::lock_guard<std::mutex> lock(robot.locate_cube_client_mutex);
                if (robot.locate_cube_client_goal) {
                    robot.locate_cube_client->async_cancel_goal(robot.locate_cube_client_goal);
                    robot.locate_cube_client_goal.reset();
                }
            }
            }

            motion_home_state_instance.active = false;
            motion_detect_cube_state_instance.active = false;
            motion_pick_above_state_instance.active = false;
            motion_pick_state_instance.active = false;
            motion_grasp_hold_state_instance.active = false;
            motion_lift_state_instance.active = false;
            motion_place_above_state_instance.active = false;
            motion_place_state_instance.active = false;
            motion_open_grasp_hold_state_instance.active = false;
            motion_retreat_state_instance.active = false;
            motion_recover_state_instance.active = false;
        }
        if (fsm->currentStateIndex == fsm->endStateIndex) {
            if (_bhv_goal) {
                auto _bhv_result = motion_spec::ros::behaviour_result(
                    _scenario_context_id, robot.ros_node->now(),
                    bdd_ros2_interfaces::msg::Trinary::TRUE);
                if (_bhv_goal->is_canceling()) {
                    _bhv_goal->canceled(_bhv_result);
                } else {
                    _bhv_goal->succeed(_bhv_result);
                }
                _bhv_goal.reset();
            }
            break;
        }
        if (robot.bhv_goal_mutex.try_lock()) {
            if (robot.bhv_staged_goal) {
                _bhv_goal = robot.bhv_staged_goal;
                robot.bhv_staged_goal.reset();
            }
            robot.bhv_goal_mutex.unlock();
        }
        if (_bhv_goal && !_bhv_goal_armed) {
            _bhv_goal_armed = true;
            _scenario_context_id = _bhv_goal->get_goal()->scenario_context_id.uuid;
            produce_event(robot.fsm_events, detect_pick_single_fsm::E_GOAL);
        }
        if (_bhv_goal && robot.fsm_events->currentEvents[detect_pick_single_fsm::E_CUBE_DETECTED]) {
            robot.bhv_event_msg.scenario_context_id.uuid = _scenario_context_id;
            robot.bhv_event_msg.stamp = robot.ros_node->now();
            robot.bhv_event_msg.uri = detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_CUBE_DETECTED];
            robot.bhv_events_pub->try_publish(robot.bhv_event_msg);
        }
        if (_bhv_goal && robot.fsm_events->currentEvents[detect_pick_single_fsm::E_GRASP_READY]) {
            robot.bhv_event_msg.scenario_context_id.uuid = _scenario_context_id;
            robot.bhv_event_msg.stamp = robot.ros_node->now();
            robot.bhv_event_msg.uri = detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_GRASP_READY];
            robot.bhv_events_pub->try_publish(robot.bhv_event_msg);
        }
        if (_bhv_goal && robot.fsm_events->currentEvents[detect_pick_single_fsm::E_RETREAT_SETTLED]) {
            robot.bhv_event_msg.scenario_context_id.uuid = _scenario_context_id;
            robot.bhv_event_msg.stamp = robot.ros_node->now();
            robot.bhv_event_msg.uri = detect_pick_single_fsm::EVENT_URIS[detect_pick_single_fsm::E_RETREAT_SETTLED];
            robot.bhv_events_pub->try_publish(robot.bhv_event_msg);
        }
        // The heartbeat drives the FSM but is not recorded: the frame log is a time series, so
        // every frame already *is* the tick (t, step, wall_ns, period_ns). Logging a per-tick
        // event would restate that 24k times and leave last_event permanently pinned to it.
        produce_event(fsm->eventData, detect_pick_single_fsm::E_STEP);
        fsm_dispatch(fsm);
        if (robot.locate_cube_client_mutex.try_lock()) {
            if (robot.locate_cube_client_staged) {
                shared.locate_cube_status = robot.locate_cube_client_staged_status;
                shared.pose_cube_base = robot.locate_cube_client_staged_pose_cube_base;
                robot.locate_cube_client_staged = false;
            }
            robot.locate_cube_client_mutex.unlock();
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
        if (_fsm_state_before == detect_pick_single_fsm::S_START && fsm->currentStateIndex == detect_pick_single_fsm::S_HOME) {
            events.record(16);
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

    detect_pick_single_fsm::destroy_fsm(fsm);

    if (_bhv_goal && _bhv_goal->is_active()) {
        // Stopping before the end state: what the behaviour did is genuinely unknown.
        auto _bhv_result = motion_spec::ros::behaviour_result(
            _scenario_context_id, robot.ros_node->now(),
            bdd_ros2_interfaces::msg::Trinary::UNKNOWN);
        if (_bhv_goal->is_canceling()) {
            _bhv_goal->canceled(_bhv_result);
        } else {
            _bhv_goal->abort(_bhv_result);
        }
    }
    _bhv_goal.reset();
    if (robot.ros_executor) {
        robot.ros_executor->cancel();
    }
    if (robot.ros_spin_thread.joinable()) {
        robot.ros_spin_thread.join();
    }
    robot.locate_cube_client_goal.reset();
    robot.locate_cube_client.reset();
    robot.bhv_events_pub.reset();
    robot.bhv_server.reset();
    robot.ros_executor.reset();
    robot.ros_node.reset();
    rclcpp::shutdown();

    if (motion_spec::runtime::stop_requested()) {
        std::cerr << "[run] interrupted; frame log closed at step " << _step << std::endl;
        return 130;
    }
    return 0;
}
