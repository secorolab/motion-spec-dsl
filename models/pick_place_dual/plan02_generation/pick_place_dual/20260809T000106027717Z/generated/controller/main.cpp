#include "runtime.hpp"
#include "shared_state.hpp"
#ifdef MOTION_SPEC_ENABLE_INTROSPECTION
#include "introspect_model.hpp"
#endif
#include "motion_home.hpp"
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
        motion_home_state_instance.ctrl_h1_position_lin_x.reset();
        motion_home_state_instance.ctrl_h1_position_lin_y.reset();
        motion_home_state_instance.ctrl_h1_position_lin_z.reset();
        motion_home_state_instance.ctrl_h1_orientation_ang_x.reset();
        motion_home_state_instance.ctrl_h1_orientation_ang_y.reset();
        motion_home_state_instance.ctrl_h1_orientation_ang_z.reset();
        motion_home_state_instance.ctrl_h2_position_lin_x.reset();
        motion_home_state_instance.ctrl_h2_position_lin_y.reset();
        motion_home_state_instance.ctrl_h2_position_lin_z.reset();
        motion_home_state_instance.ctrl_h2_orientation_ang_x.reset();
        motion_home_state_instance.ctrl_h2_orientation_ang_y.reset();
        motion_home_state_instance.ctrl_h2_orientation_ang_z.reset();
    }
    update_motion_home(motion_home_state_instance, shared, robot);
    monitor_until_motion_home();
    control_motion_home(motion_home_state_instance, shared, robot);
    apply_motion_home(motion_home_state_instance, robot);
    ++motion_home_state_instance.active_steps;
}

static void step_motion_pick_above(motion_pick_above_state &motion_pick_above_state_instance, shared_data &shared, robot_io &robot) {
    if (!motion_pick_above_state_instance.active) {
        motion_pick_above_state_instance.active = true;
        motion_pick_above_state_instance.active_steps = 0;
        motion_pick_above_state_instance.snapshot_taken = false;
        motion_pick_above_state_instance.ctrl_pa1_follow_tan.reset();
        motion_pick_above_state_instance.ctrl_pa1_follow_lat_lin_normal_a.reset();
        motion_pick_above_state_instance.ctrl_pa1_follow_lat_lin_normal_b.reset();
        motion_pick_above_state_instance.ctrl_pa1_follow_ori_ang_x.reset();
        motion_pick_above_state_instance.ctrl_pa1_follow_ori_ang_y.reset();
        motion_pick_above_state_instance.ctrl_pa1_follow_ori_ang_z.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_tan.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_lat_lin_normal_a.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_lat_lin_normal_b.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_ori_ang_x.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_ori_ang_y.reset();
        motion_pick_above_state_instance.ctrl_pa2_follow_ori_ang_z.reset();
        shared.arm1_approach_path_s = 0.0;
        shared.arm1_approach_path_along_speed = 0.0;
        shared.arm2_approach_path_s = 0.0;
        shared.arm2_approach_path_along_speed = 0.0;
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
        motion_pick_state_instance.ctrl_pk1_hold_x.reset();
        motion_pick_state_instance.ctrl_pk1_hold_y.reset();
        motion_pick_state_instance.ctrl_pk1_lower_z.reset();
        motion_pick_state_instance.ctrl_pk1_follow_ori_ang_x.reset();
        motion_pick_state_instance.ctrl_pk1_follow_ori_ang_y.reset();
        motion_pick_state_instance.ctrl_pk1_follow_ori_ang_z.reset();
        motion_pick_state_instance.ctrl_pk1_support_z.reset();
        motion_pick_state_instance.ctrl_pk2_hold_x.reset();
        motion_pick_state_instance.ctrl_pk2_hold_y.reset();
        motion_pick_state_instance.ctrl_pk2_lower_z.reset();
        motion_pick_state_instance.ctrl_pk2_follow_ori_ang_x.reset();
        motion_pick_state_instance.ctrl_pk2_follow_ori_ang_y.reset();
        motion_pick_state_instance.ctrl_pk2_follow_ori_ang_z.reset();
        motion_pick_state_instance.ctrl_pk2_support_z.reset();
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
        motion_grasp_hold_state_instance.ctrl_cg1_hold_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_hold_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_hold_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_hold_orientation_ang_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_hold_orientation_ang_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_hold_orientation_ang_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg1_support_z.reset();

        motion_grasp_hold_state_instance.ctrl_cg2_hold_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_hold_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_hold_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_hold_orientation_ang_x.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_hold_orientation_ang_y.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_hold_orientation_ang_z.reset();
        motion_grasp_hold_state_instance.ctrl_cg2_support_z.reset();

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
        motion_lift_state_instance.ctrl_lt1_hold_x.reset();
        motion_lift_state_instance.ctrl_lt1_hold_y.reset();
        motion_lift_state_instance.ctrl_lt1_lift_z.reset();
        motion_lift_state_instance.ctrl_lt1_follow_ori_ang_x.reset();
        motion_lift_state_instance.ctrl_lt1_follow_ori_ang_y.reset();
        motion_lift_state_instance.ctrl_lt1_follow_ori_ang_z.reset();
        motion_lift_state_instance.ctrl_lt1_support_z.reset();

        motion_lift_state_instance.ctrl_lt2_hold_x.reset();
        motion_lift_state_instance.ctrl_lt2_hold_y.reset();
        motion_lift_state_instance.ctrl_lt2_lift_z.reset();
        motion_lift_state_instance.ctrl_lt2_follow_ori_ang_x.reset();
        motion_lift_state_instance.ctrl_lt2_follow_ori_ang_y.reset();
        motion_lift_state_instance.ctrl_lt2_follow_ori_ang_z.reset();
        motion_lift_state_instance.ctrl_lt2_support_z.reset();

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
        motion_place_above_state_instance.ctrl_pla1_reach_x.reset();
        motion_place_above_state_instance.ctrl_pla1_reach_y.reset();
        motion_place_above_state_instance.ctrl_pla1_reach_z.reset();
        motion_place_above_state_instance.ctrl_pla1_align_ori_ang_x.reset();
        motion_place_above_state_instance.ctrl_pla1_align_ori_ang_y.reset();
        motion_place_above_state_instance.ctrl_pla1_align_ori_ang_z.reset();
        motion_place_above_state_instance.ctrl_pla1_support_z.reset();

        motion_place_above_state_instance.ctrl_pla2_reach_x.reset();
        motion_place_above_state_instance.ctrl_pla2_reach_y.reset();
        motion_place_above_state_instance.ctrl_pla2_reach_z.reset();
        motion_place_above_state_instance.ctrl_pla2_align_ori_ang_x.reset();
        motion_place_above_state_instance.ctrl_pla2_align_ori_ang_y.reset();
        motion_place_above_state_instance.ctrl_pla2_align_ori_ang_z.reset();
        motion_place_above_state_instance.ctrl_pla2_support_z.reset();

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
        motion_place_state_instance.ctrl_pl1_reach_x.reset();
        motion_place_state_instance.ctrl_pl1_reach_y.reset();
        motion_place_state_instance.ctrl_pl1_reach_z.reset();
        motion_place_state_instance.ctrl_pl1_align_ori_ang_x.reset();
        motion_place_state_instance.ctrl_pl1_align_ori_ang_y.reset();
        motion_place_state_instance.ctrl_pl1_align_ori_ang_z.reset();
        motion_place_state_instance.ctrl_pl1_support_z.reset();

        motion_place_state_instance.ctrl_pl2_reach_x.reset();
        motion_place_state_instance.ctrl_pl2_reach_y.reset();
        motion_place_state_instance.ctrl_pl2_reach_z.reset();
        motion_place_state_instance.ctrl_pl2_align_ori_ang_x.reset();
        motion_place_state_instance.ctrl_pl2_align_ori_ang_y.reset();
        motion_place_state_instance.ctrl_pl2_align_ori_ang_z.reset();
        motion_place_state_instance.ctrl_pl2_support_z.reset();

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
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_orientation_ang_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_orientation_ang_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_hold_orientation_ang_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og1_support_z.reset();

        motion_open_grasp_hold_state_instance.ctrl_og2_hold_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_hold_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_hold_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_hold_orientation_ang_x.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_hold_orientation_ang_y.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_hold_orientation_ang_z.reset();
        motion_open_grasp_hold_state_instance.ctrl_og2_support_z.reset();

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
        motion_retreat_state_instance.ctrl_ret1_reach_x.reset();
        motion_retreat_state_instance.ctrl_ret1_reach_y.reset();
        motion_retreat_state_instance.ctrl_ret1_reach_z.reset();
        motion_retreat_state_instance.ctrl_ret1_align_ori_ang_x.reset();
        motion_retreat_state_instance.ctrl_ret1_align_ori_ang_y.reset();
        motion_retreat_state_instance.ctrl_ret1_align_ori_ang_z.reset();
        motion_retreat_state_instance.ctrl_ret1_support_z.reset();

        motion_retreat_state_instance.ctrl_ret2_reach_x.reset();
        motion_retreat_state_instance.ctrl_ret2_reach_y.reset();
        motion_retreat_state_instance.ctrl_ret2_reach_z.reset();
        motion_retreat_state_instance.ctrl_ret2_align_ori_ang_x.reset();
        motion_retreat_state_instance.ctrl_ret2_align_ori_ang_y.reset();
        motion_retreat_state_instance.ctrl_ret2_align_ori_ang_z.reset();
        motion_retreat_state_instance.ctrl_ret2_support_z.reset();

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
        motion_recover_state_instance.ctrl_rc1_hold_x.reset();
        motion_recover_state_instance.ctrl_rc1_hold_y.reset();
        motion_recover_state_instance.ctrl_rc1_hold_z.reset();
        motion_recover_state_instance.ctrl_rc1_hold_orientation_ang_x.reset();
        motion_recover_state_instance.ctrl_rc1_hold_orientation_ang_y.reset();
        motion_recover_state_instance.ctrl_rc1_hold_orientation_ang_z.reset();
        motion_recover_state_instance.ctrl_rc1_support_z.reset();
        motion_recover_state_instance.ctrl_rc2_hold_x.reset();
        motion_recover_state_instance.ctrl_rc2_hold_y.reset();
        motion_recover_state_instance.ctrl_rc2_hold_z.reset();
        motion_recover_state_instance.ctrl_rc2_hold_orientation_ang_x.reset();
        motion_recover_state_instance.ctrl_rc2_hold_orientation_ang_y.reset();
        motion_recover_state_instance.ctrl_rc2_hold_orientation_ang_z.reset();
        motion_recover_state_instance.ctrl_rc2_support_z.reset();
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

    const auto robot_config = motion_spec::config::load("/home/batsy/work/ms/src/motion-spec-dsl/models/pick_place_dual/robot.toml");
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
        mj_robot_spec.prefix = "kinova1_";
        mj_robot_spec.attach_to.kind = mj_kdl::AttachKind::Site;
        mj_robot_spec.attach_to.name = "table_table_top";
        mj_robot_spec.pos[0] = -0.7;
        mj_robot_spec.pos[1] = 0.0;
        mj_robot_spec.pos[2] = 0.0;
        mj_robot_spec.quat[0] = 0.0;
        mj_robot_spec.quat[1] = 0.0;
        mj_robot_spec.quat[2] = 0.0;
        mj_robot_spec.quat[3] = 1.0;
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
            attachment_spec.attach_to.name = "pinch_site";
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
        const std::filesystem::path robot_mjcf_path = find_asset_path({"src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml"});
        if (robot_mjcf_path.empty()) {
            std::cerr << "MJCF model not found: src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml\n";
            return 1;
        }
        mj_path_storage.push_back(robot_mjcf_path.string());
        mj_kdl::RobotSpec mj_robot_spec{};
        mj_robot_spec.path = mj_path_storage.back().c_str();
        mj_robot_spec.prefix = "kinova2_";
        mj_robot_spec.attach_to.kind = mj_kdl::AttachKind::Site;
        mj_robot_spec.attach_to.name = "table_table_top";
        mj_robot_spec.pos[0] = 0.7;
        mj_robot_spec.pos[1] = 0.0;
        mj_robot_spec.pos[2] = 0.0;
        mj_robot_spec.quat[0] = 0.0;
        mj_robot_spec.quat[1] = 0.0;
        mj_robot_spec.quat[2] = 1.0;
        mj_robot_spec.quat[3] = 6.123233995736766E-17;
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
            attachment_spec.attach_to.name = "pinch_site";
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
        object_spec.pos[0] = -0.2;
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
        object_spec.name = "cube2";
        object_spec.attach_to.kind = mj_kdl::AttachKind::World;
        object_spec.attach_to.name = "";
        const std::filesystem::path object_path = find_asset_path({"src/mj_kdl_wrapper/assets/cube.xml"});
        if (object_path.empty()) {
            std::cerr << "MJCF object not found: src/mj_kdl_wrapper/assets/cube.xml\n";
            return 1;
        }
        object_spec.mjcf_path = object_path.string();
        object_spec.pos[0] = 0.2;
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

    mj_kdl::Robot mj_arm1_solver_home{};
    mj_kdl::ToolFrameSpec mj_tool_arm1_solver_home{};
    KDL::Tree built_tree_arm1_solver_home{};
    if (!scene_kdl::make_tree_world_tree(&built_tree_arm1_solver_home)) {
        std::cerr << "make_tree_world_tree() failed for runtime arm1_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    KDL::Chain built_chain_arm1_solver_home{};
    if (!scene_kdl::make_chain_arm1_tree_chain(built_tree_arm1_solver_home, &built_chain_arm1_solver_home)) {
        std::cerr << "make_chain_arm1_tree_chain() failed for runtime arm1_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    if (!mj_kdl::init_robot_from_chain(&mj_arm1_solver_home, mj_env.model, mj_env.data, built_chain_arm1_solver_home, {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"}, "kinova1_", &mj_tool_arm1_solver_home)) {
        std::cerr << "init_robot_from_chain() failed for runtime arm1_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    mj_arm1_solver_home.ctrl_mode = mj_kdl::CtrlMode::TORQUE;
    mj_kdl::env_add_robot(&mj_env, &mj_arm1_solver_home);

    KDL::Chain *chain_arm1_solver_home = &mj_arm1_solver_home.chain;

    robot.arm1_solver_home = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    mj_kdl::Robot mj_arm2_solver_home{};
    mj_kdl::ToolFrameSpec mj_tool_arm2_solver_home{};
    KDL::Tree built_tree_arm2_solver_home{};
    if (!scene_kdl::make_tree_world_tree(&built_tree_arm2_solver_home)) {
        std::cerr << "make_tree_world_tree() failed for runtime arm2_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    KDL::Chain built_chain_arm2_solver_home{};
    if (!scene_kdl::make_chain_arm2_tree_chain(built_tree_arm2_solver_home, &built_chain_arm2_solver_home)) {
        std::cerr << "make_chain_arm2_tree_chain() failed for runtime arm2_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    if (!mj_kdl::init_robot_from_chain(&mj_arm2_solver_home, mj_env.model, mj_env.data, built_chain_arm2_solver_home, {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"}, "kinova2_", &mj_tool_arm2_solver_home)) {
        std::cerr << "init_robot_from_chain() failed for runtime arm2_solver_home\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    mj_arm2_solver_home.ctrl_mode = mj_kdl::CtrlMode::TORQUE;
    mj_kdl::env_add_robot(&mj_env, &mj_arm2_solver_home);

    KDL::Chain *chain_arm2_solver_home = &mj_arm2_solver_home.chain;

    robot.arm2_solver_home = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_pick_above = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_pick_above = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_pick = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_pick = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_grasp_hold = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_grasp_hold = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_lift = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_lift = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_place_above = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_place_above = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_place = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_place = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_open_grasp_hold = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_open_grasp_hold = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_retreat = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_retreat = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    robot.arm1_solver_recover = manipulator_robot{
        .robot = &mj_arm1_solver_home,
        .chain = chain_arm1_solver_home,
    };

    robot.arm2_solver_recover = manipulator_robot{
        .robot = &mj_arm2_solver_home,
        .chain = chain_arm2_solver_home,
    };

    mj_kdl::Viewer mj_viewer{};
    if (!headless) {
        if (!mj_kdl::init_window_sim(&mj_viewer, &mj_arm1_solver_home)) {
            std::cerr << "mj_kdl::init_window_sim() failed\n";
            mj_kdl::cleanup(&mj_env);
            return 1;
        }
    }
    motion_home_state motion_home_state_instance;
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
            KDL::JntArray _q(mj_arm1_solver_home.n_joints);
            for (int _i = 0; _i < mj_arm1_solver_home.n_joints && _i < static_cast<int>(std::size(_q_home)); ++_i)
                _q(_i) = _q_home[_i];
            mj_kdl::set_joint_pos(&mj_arm1_solver_home, _q, false);
        }
        {
            const double _q_home[] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
            KDL::JntArray _q(mj_arm2_solver_home.n_joints);
            for (int _i = 0; _i < mj_arm2_solver_home.n_joints && _i < static_cast<int>(std::size(_q_home)); ++_i)
                _q(_i) = _q_home[_i];
            mj_kdl::set_joint_pos(&mj_arm2_solver_home, _q, false);
        }

        reset_motion_home(motion_home_state_instance);
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
    double _prev_sim_time = mj_arm1_solver_home.data->time;
    struct fsm_nbx *fsm = pick_place_dual_fsm::create_fsm();
    if (!fsm) return 1;
    robot.fsm_events = fsm->eventData;

    auto fsm_active_motion = [&](struct fsm_nbx *fsm) -> int {
        switch (fsm->currentStateIndex) {
        case pick_place_dual_fsm::S_HOME: return 0;

        case pick_place_dual_fsm::S_PICK_ABOVE: return 1;

        case pick_place_dual_fsm::S_PICK: return 2;

        case pick_place_dual_fsm::S_GRASP: return 3;

        case pick_place_dual_fsm::S_LIFT: return 4;

        case pick_place_dual_fsm::S_PLACE_ABOVE: return 5;

        case pick_place_dual_fsm::S_PLACE: return 6;

        case pick_place_dual_fsm::S_OPEN: return 7;

        case pick_place_dual_fsm::S_RETREAT: return 8;

        case pick_place_dual_fsm::S_RECOVER: return 9;

        default: return -1;
        }
    };
    // The FSM's current state selects which motion runs; the motion's monitor advances the FSM. A
    // fallback hold state additionally re-evaluates the gated motion's WHEN precondition each tick.
    auto fsm_dispatch = [&](struct fsm_nbx *fsm) {
        switch (fsm->currentStateIndex) {
        case pick_place_dual_fsm::S_HOME:
            step_motion_home(motion_home_state_instance, shared, robot);
            monitor_when_motion_pick_above(motion_pick_above_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_PICK_ABOVE:
            step_motion_pick_above(motion_pick_above_state_instance, shared, robot);
            monitor_when_motion_pick(motion_pick_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_PICK:
            step_motion_pick(motion_pick_state_instance, shared, robot);
            monitor_when_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_GRASP:
            step_motion_grasp_hold(motion_grasp_hold_state_instance, shared, robot);
            monitor_when_motion_lift(motion_lift_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_LIFT:
            step_motion_lift(motion_lift_state_instance, shared, robot, events);
            monitor_when_motion_place_above(motion_place_above_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_PLACE_ABOVE:
            step_motion_place_above(motion_place_above_state_instance, shared, robot, events);
            monitor_when_motion_place(motion_place_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_PLACE:
            step_motion_place(motion_place_state_instance, shared, robot, events);
            monitor_when_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_OPEN:
            step_motion_open_grasp_hold(motion_open_grasp_hold_state_instance, shared, robot);
            monitor_when_motion_retreat(motion_retreat_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_RETREAT:
            step_motion_retreat(motion_retreat_state_instance, shared, robot, events);
            break;

        case pick_place_dual_fsm::S_RECOVER:
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
    while (!motion_spec::runtime::stop_requested() && ((!headless || !steps_limited || headless_steps-- > 0))) {
        shared.clock_time_s = robot.arm1_solver_home.robot->data->time;
        // Integrators step with the time that actually elapsed, not the period we asked for: the
        // loop sleeps to deadlines it can overrun. In sim the clock advances exactly one timestep
        // per tick, so this equals the nominal period.
        const double _dt_raw = (_step == 0)
            ? motion_spec::runtime::kControlPeriodS
            : shared.clock_time_s - _prev_clock_s;
        _prev_clock_s = shared.clock_time_s;
        shared.dt_measured_s = std::clamp(_dt_raw, motion_spec::runtime::kDtClampMin,
                                          motion_spec::runtime::kDtClampMax);
        if (mj_arm1_solver_home.data->time < _prev_sim_time - 1e-6) {
            mj_kdl::reset(&mj_env);
            fsm->currentStateIndex = fsm->startStateIndex;
        }
        _prev_sim_time = mj_arm1_solver_home.data->time;
        if (static_cast<int>(fsm->currentStateIndex) != _fsm_prev_state) {
            _fsm_prev_state = static_cast<int>(fsm->currentStateIndex);
            std::cerr << "[fsm] state   " << pick_place_dual_fsm::STATE_URIS[fsm->currentStateIndex] << std::endl;
            // Leaving a state ends its motion's activation, so re-entering one is a fresh
            // start: active_steps returns to 0 and any trajectory replans from the current
            // pose instead of resuming the alpha it had when it was interrupted.
            motion_home_state_instance.active = false;
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
            break;
        }
        // The heartbeat drives the FSM but is not recorded: the frame log is a time series, so
        // every frame already *is* the tick (t, step, wall_ns, period_ns). Logging a per-tick
        // event would restate that 24k times and leave last_event permanently pinned to it.
        produce_event(fsm->eventData, pick_place_dual_fsm::E_STEP);
        fsm_dispatch(fsm);

        mj_kdl::update(&mj_arm1_solver_home);
        mj_kdl::update(&mj_arm2_solver_home);

        // Closing the viewer ends the run, but it must end the same way S_DONE does. std::exit
        // skips destructors, so the frame logger's writer thread never drained and the log was
        // left with a half-written record and no health file -- break, and let teardown run.
        if (!mj_kdl::step(&mj_arm1_solver_home)) {
            break;
        }
        const unsigned int _fsm_state_before = fsm->currentStateIndex;
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
        // The clock is this transition's guard, so this one heartbeat is the cause of the
        // state change -- record it, unlike the 24k that drive nothing.
        if (_fsm_state_before == pick_place_dual_fsm::S_START && fsm->currentStateIndex == pick_place_dual_fsm::S_HOME) {
            events.record(13);
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
    mj_kdl::cleanup(&mj_arm1_solver_home);

    mj_kdl::cleanup(&mj_arm2_solver_home);

    pick_place_dual_fsm::destroy_fsm(fsm);
    if (motion_spec::runtime::stop_requested()) {
        std::cerr << "[run] interrupted; frame log closed at step " << _step << std::endl;
        return 130;
    }
    return 0;
}
