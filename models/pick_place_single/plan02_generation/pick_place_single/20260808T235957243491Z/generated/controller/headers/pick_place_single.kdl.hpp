// This is an auto-generated file. Do not edit it directly.
//
// KDL trees and chains for the kinematics of: pick_place_single.robmot
#pragma once

#include <string_view>
#include <unordered_map>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>

namespace scene_kdl {

inline bool make_tree_world_tree(KDL::Tree *tree)
{
    *tree = KDL::Tree("world_tree/world_body");
    if (!tree->addSegment(
          KDL::Segment("world_tree/table",
            KDL::Joint("world_tree/table_on_world", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)),
            KDL::RigidBodyInertia(84.26442414492935, KDL::Vector(0.0, 0.0, -0.05100416904566816),
            KDL::RotationalInertia(12.757461560528643, 21.925927231369712, 32.45420747108136, 0.0, 0.0, 0.0))),
          "world_tree/world_body")) {
        return false;  // no segment "world_tree/world_body" to attach "world_tree/table" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/base_link",
            KDL::Joint("world_tree/arm_on_table", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)),
            KDL::RigidBodyInertia(1.697, KDL::Vector(-0.000648, -0.000166, 0.084487),
            KDL::RotationalInertia(0.004621997830409856, 0.004494995375227889, 0.002078996794362252, 9.000172192402649e-06, 6.000008041590608e-05, 9.00000553086247e-06))),
          "world_tree/table")) {
        return false;  // no segment "world_tree/table" to attach "kinova/base_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/shoulder_link",
            KDL::Joint("kinova/joint_1", KDL::Vector(0.0, 0.0, 0.15643), KDL::Vector(0.0, -1.2246467991473532e-16, -1.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, 0.0, 0.15643)),
            KDL::RigidBodyInertia(1.3773, KDL::Vector(-2.3e-05, -0.010364, -0.07336),
            KDL::RotationalInertia(0.004570003811695442, 0.0048309960179057955, 0.0014090001703987627, 1.000082024569336e-06, 1.999854657626552e-06, 0.000448000581204044))),
          "kinova/base_link")) {
        return false;  // no segment "kinova/base_link" to attach "kinova/shoulder_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/half_arm_1_link",
            KDL::Joint("kinova/joint_2", KDL::Vector(0.0, 0.005375, -0.12838), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.005375, -0.12838)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.09958, -0.013278),
            KDL::RotationalInertia(0.011087998016112646, 0.0010719992202473268, 0.01125502276364003, 5.0430807879481415e-06, 2.6350085927990395e-09, -0.000691005565634027))),
          "kinova/shoulder_link")) {
        return false;  // no segment "kinova/shoulder_link" to attach "kinova/half_arm_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/half_arm_2_link",
            KDL::Joint("kinova/joint_3", KDL::Vector(0.0, -0.21038, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.21038, -0.006375)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.006641, -0.117892),
            KDL::RotationalInertia(0.01093199583626794, 0.01112701208224181, 0.0010430020814902476, 1.6234017018333586e-10, -7.000643157742045e-06, 0.0006060007060898792))),
          "kinova/half_arm_1_link")) {
        return false;  // no segment "kinova/half_arm_1_link" to attach "kinova/half_arm_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/forearm_link",
            KDL::Joint("kinova/joint_4", KDL::Vector(0.0, 0.006375, -0.21038), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.006375, -0.21038)),
            KDL::RigidBodyInertia(0.9302, KDL::Vector(-1.8e-05, -0.075478, -0.015006),
            KDL::RotationalInertia(0.008146999901706557, 0.0006310006908962165, 0.008315995407397225, -9.501000863102778e-07, 3.21038857868019e-09, -0.0005000015218802465))),
          "kinova/half_arm_2_link")) {
        return false;  // no segment "kinova/half_arm_2_link" to attach "kinova/forearm_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/spherical_wrist_1_link",
            KDL::Joint("kinova/joint_5", KDL::Vector(0.0, -0.20843, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.20843, -0.006375)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.009432, -0.063883),
            KDL::RotationalInertia(0.001596, 0.0016069980120088246, 0.0003989999879911757, 6.879198340614489e-21, 1.2728114693678706e-21, 0.0002559992687750574))),
          "kinova/forearm_link")) {
        return false;  // no segment "kinova/forearm_link" to attach "kinova/spherical_wrist_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/spherical_wrist_2_link",
            KDL::Joint("kinova/joint_6", KDL::Vector(0.0, 0.00017505, -0.10593), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.00017505, -0.10593)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.045483, -0.00965),
            KDL::RotationalInertia(0.0016409999999999997, 0.0004099995285298059, 0.0016410004714701937, -8.295218490800401e-21, 9.713023024165384e-20, -0.00027799965201411465))),
          "kinova/spherical_wrist_1_link")) {
        return false;  // no segment "kinova/spherical_wrist_1_link" to attach "kinova/spherical_wrist_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova/bracelet_link",
            KDL::Joint("kinova/joint_7", KDL::Vector(0.0, -0.10593, -0.00017505), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.10593, -0.00017505)),
            KDL::RigidBodyInertia(0.5, KDL::Vector(0.000281, 0.011402, -0.029798),
            KDL::RotationalInertia(0.0005869997255105773, 0.0003690003455320899, 0.0006089999289573331, 3.00185764659545e-06, 3.000803527862622e-06, -0.00011800055321349603))),
          "kinova/spherical_wrist_2_link")) {
        return false;  // no segment "kinova/spherical_wrist_2_link" to attach "kinova/bracelet_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("ft_tree/wrist_ft_body",
            KDL::Joint("kinova_2f85/ft_on_pinch_site", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, 0.0, -0.061525)),
            KDL::RigidBodyInertia(0.01, KDL::Vector(0.0, 0.0, 0.0),
            KDL::RotationalInertia(1e-05, 1e-05, 1e-05, 0.0, 0.0, 0.0))),
          "kinova/bracelet_link")) {
        return false;  // no segment "kinova/bracelet_link" to attach "ft_tree/wrist_ft_body" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper/g_base_mount",
            KDL::Joint("kinova_2f85/g_mount_on_ft", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.007)),
            KDL::RigidBodyInertia(0.15000285516377976, KDL::Vector(-0.0003605835958214157, 8.536274800937428e-05, -5.9199141450506514e-05),
            KDL::RotationalInertia(5.217217253247701e-05, 5.359968409474146e-05, 0.00010234508271477983, -2.1121394872370117e-09, -1.4277807079157603e-07, -6.010509618238863e-12))),
          "ft_tree/wrist_ft_body")) {
        return false;  // no segment "ft_tree/wrist_ft_body" to attach "gripper/g_base_mount" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper/g_base",
            KDL::Joint("gripper/g_base_on_mount", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)),
            KDL::RigidBodyInertia(0.9026054836789594, KDL::Vector(0.0, -2.3289840972067775e-05, 0.04247701482094086),
            KDL::RotationalInertia(0.0009287010595313735, 0.0005835572393484526, 0.0004776404071743112, 1.202439660816404e-11, -3.605993341917436e-16, -3.695091338611354e-07))),
          "gripper/g_base_mount")) {
        return false;  // no segment "gripper/g_base_mount" to attach "gripper/g_base" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper/g_left_driver",
            KDL::Joint("gripper/g_left_driver_joint", KDL::Vector(0.0, -0.0306011, 0.054904), KDL::Vector(-1.0, 1.2246467991473532e-16, 0.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 1.0, 6.123233995736766e-17), KDL::Vector(0.0, -0.0306011, 0.054904)),
            KDL::RigidBodyInertia(0.00899563, KDL::Vector(0.0, 0.0177547, 0.00107314),
            KDL::RotationalInertia(1.7235199999999992e-06, 3.286147803680565e-07, 1.6024512196319424e-06, 0.0, 0.0, -9.199011484869232e-08))),
          "gripper/g_base")) {
        return false;  // no segment "gripper/g_base" to attach "gripper/g_left_driver" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper/g_base/g_pinch",
            KDL::Joint(KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.145)),
            KDL::RigidBodyInertia::Zero()),
          "gripper/g_base")) {
        return false;  // no segment "gripper/g_base" to attach "gripper/g_base/g_pinch" to
    }
    return true;
}

// Slices the chain out of a tree make_tree_world_tree built, so several
// chains over one tree build it once.
inline bool make_chain_kinova_2f85_chain(const KDL::Tree &tree, KDL::Chain *chain)
{
    // 'kinova/base_link' -> 'gripper/g_base/g_pinch'
    return tree.getChain("kinova/base_link", "gripper/g_base/g_pinch", *chain);
}


// The model element each name in this header stands for. KDL knows a segment or joint by the
// name it is built with, and a tree or chain by the function that makes it, so those are the
// names to look up: what the scene called the thing is not recoverable from them otherwise.
inline const std::unordered_map<std::string_view, std::string_view> kIris = {
    {"world_tree", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree"},
    {"world_tree/world_body", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/world_body"},
    {"world_tree/table", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table"},
    {"world_tree/table_on_world", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/table_on_world"},
    {"kinova/base_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/base_link"},
    {"world_tree/arm_on_table", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/world_tree/arm_on_table"},
    {"kinova/shoulder_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/shoulder_link"},
    {"kinova/joint_1", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_1"},
    {"kinova/half_arm_1_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_1_link"},
    {"kinova/joint_2", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_2"},
    {"kinova/half_arm_2_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/half_arm_2_link"},
    {"kinova/joint_3", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_3"},
    {"kinova/forearm_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/forearm_link"},
    {"kinova/joint_4", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_4"},
    {"kinova/spherical_wrist_1_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_1_link"},
    {"kinova/joint_5", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_5"},
    {"kinova/spherical_wrist_2_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/spherical_wrist_2_link"},
    {"kinova/joint_6", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_6"},
    {"kinova/bracelet_link", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/bracelet_link"},
    {"kinova/joint_7", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova/joint_7"},
    {"ft_tree/wrist_ft_body", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/ft_tree/wrist_ft_body"},
    {"kinova_2f85/ft_on_pinch_site", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova_2f85/ft_on_pinch_site"},
    {"gripper/g_base_mount", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_mount"},
    {"kinova_2f85/g_mount_on_ft", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova_2f85/g_mount_on_ft"},
    {"gripper/g_base", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base"},
    {"gripper/g_base_on_mount", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base_on_mount"},
    {"gripper/g_left_driver", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver"},
    {"gripper/g_left_driver_joint", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_left_driver_joint"},
    {"gripper/g_base/g_pinch", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/gripper/g_base/g_pinch"},
    {"kinova_2f85_chain", "https://secorolab.github.io/models/scenes/pick-place-single-mjc/kinova_2f85/chain"},
};

}  // namespace scene_kdl
