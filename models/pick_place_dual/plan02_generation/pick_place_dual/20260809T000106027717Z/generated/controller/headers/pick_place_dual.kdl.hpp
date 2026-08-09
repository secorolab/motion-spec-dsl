// This is an auto-generated file. Do not edit it directly.
//
// KDL trees and chains for the kinematics of: pick_place_dual.robmot
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
          KDL::Segment("kinova1/base_link",
            KDL::Joint("world_tree/arm1_on_table", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(-0.7, 0.0, 0.0)),
            KDL::RigidBodyInertia(1.697, KDL::Vector(-0.000648, -0.000166, 0.084487),
            KDL::RotationalInertia(0.004621997830409856, 0.004494995375227889, 0.002078996794362252, 9.000172192402649e-06, 6.000008041590608e-05, 9.00000553086247e-06))),
          "world_tree/table")) {
        return false;  // no segment "world_tree/table" to attach "kinova1/base_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/base_link",
            KDL::Joint("world_tree/arm2_on_table", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 1.0, 6.123233995736766e-17), KDL::Vector(0.7, 0.0, 0.0)),
            KDL::RigidBodyInertia(1.697, KDL::Vector(-0.000648, -0.000166, 0.084487),
            KDL::RotationalInertia(0.004621997830409856, 0.004494995375227889, 0.002078996794362252, 9.000172192402649e-06, 6.000008041590608e-05, 9.00000553086247e-06))),
          "world_tree/table")) {
        return false;  // no segment "world_tree/table" to attach "kinova2/base_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/shoulder_link",
            KDL::Joint("kinova1/joint_1", KDL::Vector(0.0, 0.0, 0.15643), KDL::Vector(0.0, -1.2246467991473532e-16, -1.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, 0.0, 0.15643)),
            KDL::RigidBodyInertia(1.3773, KDL::Vector(-2.3e-05, -0.010364, -0.07336),
            KDL::RotationalInertia(0.004570003811695442, 0.0048309960179057955, 0.0014090001703987627, 1.000082024569336e-06, 1.999854657626552e-06, 0.000448000581204044))),
          "kinova1/base_link")) {
        return false;  // no segment "kinova1/base_link" to attach "kinova1/shoulder_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/shoulder_link",
            KDL::Joint("kinova2/joint_1", KDL::Vector(0.0, 0.0, 0.15643), KDL::Vector(0.0, -1.2246467991473532e-16, -1.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, 0.0, 0.15643)),
            KDL::RigidBodyInertia(1.3773, KDL::Vector(-2.3e-05, -0.010364, -0.07336),
            KDL::RotationalInertia(0.004570003811695442, 0.0048309960179057955, 0.0014090001703987627, 1.000082024569336e-06, 1.999854657626552e-06, 0.000448000581204044))),
          "kinova2/base_link")) {
        return false;  // no segment "kinova2/base_link" to attach "kinova2/shoulder_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/half_arm_1_link",
            KDL::Joint("kinova1/joint_2", KDL::Vector(0.0, 0.005375, -0.12838), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.005375, -0.12838)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.09958, -0.013278),
            KDL::RotationalInertia(0.011087998016112646, 0.0010719992202473268, 0.01125502276364003, 5.0430807879481415e-06, 2.6350085927990395e-09, -0.000691005565634027))),
          "kinova1/shoulder_link")) {
        return false;  // no segment "kinova1/shoulder_link" to attach "kinova1/half_arm_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/half_arm_1_link",
            KDL::Joint("kinova2/joint_2", KDL::Vector(0.0, 0.005375, -0.12838), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.005375, -0.12838)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.09958, -0.013278),
            KDL::RotationalInertia(0.011087998016112646, 0.0010719992202473268, 0.01125502276364003, 5.0430807879481415e-06, 2.6350085927990395e-09, -0.000691005565634027))),
          "kinova2/shoulder_link")) {
        return false;  // no segment "kinova2/shoulder_link" to attach "kinova2/half_arm_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/half_arm_2_link",
            KDL::Joint("kinova1/joint_3", KDL::Vector(0.0, -0.21038, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.21038, -0.006375)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.006641, -0.117892),
            KDL::RotationalInertia(0.01093199583626794, 0.01112701208224181, 0.0010430020814902476, 1.6234017018333586e-10, -7.000643157742045e-06, 0.0006060007060898792))),
          "kinova1/half_arm_1_link")) {
        return false;  // no segment "kinova1/half_arm_1_link" to attach "kinova1/half_arm_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/half_arm_2_link",
            KDL::Joint("kinova2/joint_3", KDL::Vector(0.0, -0.21038, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.21038, -0.006375)),
            KDL::RigidBodyInertia(1.1636, KDL::Vector(-4.4e-05, -0.006641, -0.117892),
            KDL::RotationalInertia(0.01093199583626794, 0.01112701208224181, 0.0010430020814902476, 1.6234017018333586e-10, -7.000643157742045e-06, 0.0006060007060898792))),
          "kinova2/half_arm_1_link")) {
        return false;  // no segment "kinova2/half_arm_1_link" to attach "kinova2/half_arm_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/forearm_link",
            KDL::Joint("kinova1/joint_4", KDL::Vector(0.0, 0.006375, -0.21038), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.006375, -0.21038)),
            KDL::RigidBodyInertia(0.9302, KDL::Vector(-1.8e-05, -0.075478, -0.015006),
            KDL::RotationalInertia(0.008146999901706557, 0.0006310006908962165, 0.008315995407397225, -9.501000863102778e-07, 3.21038857868019e-09, -0.0005000015218802465))),
          "kinova1/half_arm_2_link")) {
        return false;  // no segment "kinova1/half_arm_2_link" to attach "kinova1/forearm_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/forearm_link",
            KDL::Joint("kinova2/joint_4", KDL::Vector(0.0, 0.006375, -0.21038), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.006375, -0.21038)),
            KDL::RigidBodyInertia(0.9302, KDL::Vector(-1.8e-05, -0.075478, -0.015006),
            KDL::RotationalInertia(0.008146999901706557, 0.0006310006908962165, 0.008315995407397225, -9.501000863102778e-07, 3.21038857868019e-09, -0.0005000015218802465))),
          "kinova2/half_arm_2_link")) {
        return false;  // no segment "kinova2/half_arm_2_link" to attach "kinova2/forearm_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/spherical_wrist_1_link",
            KDL::Joint("kinova1/joint_5", KDL::Vector(0.0, -0.20843, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.20843, -0.006375)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.009432, -0.063883),
            KDL::RotationalInertia(0.001596, 0.0016069980120088246, 0.0003989999879911757, 6.879198340614489e-21, 1.2728114693678706e-21, 0.0002559992687750574))),
          "kinova1/forearm_link")) {
        return false;  // no segment "kinova1/forearm_link" to attach "kinova1/spherical_wrist_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/spherical_wrist_1_link",
            KDL::Joint("kinova2/joint_5", KDL::Vector(0.0, -0.20843, -0.006375), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.20843, -0.006375)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.009432, -0.063883),
            KDL::RotationalInertia(0.001596, 0.0016069980120088246, 0.0003989999879911757, 6.879198340614489e-21, 1.2728114693678706e-21, 0.0002559992687750574))),
          "kinova2/forearm_link")) {
        return false;  // no segment "kinova2/forearm_link" to attach "kinova2/spherical_wrist_1_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/spherical_wrist_2_link",
            KDL::Joint("kinova1/joint_6", KDL::Vector(0.0, 0.00017505, -0.10593), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.00017505, -0.10593)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.045483, -0.00965),
            KDL::RotationalInertia(0.0016409999999999997, 0.0004099995285298059, 0.0016410004714701937, -8.295218490800401e-21, 9.713023024165384e-20, -0.00027799965201411465))),
          "kinova1/spherical_wrist_1_link")) {
        return false;  // no segment "kinova1/spherical_wrist_1_link" to attach "kinova1/spherical_wrist_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/spherical_wrist_2_link",
            KDL::Joint("kinova2/joint_6", KDL::Vector(0.0, 0.00017505, -0.10593), KDL::Vector(0.0, -1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, 0.00017505, -0.10593)),
            KDL::RigidBodyInertia(0.6781, KDL::Vector(1e-06, -0.045483, -0.00965),
            KDL::RotationalInertia(0.0016409999999999997, 0.0004099995285298059, 0.0016410004714701937, -8.295218490800401e-21, 9.713023024165384e-20, -0.00027799965201411465))),
          "kinova2/spherical_wrist_1_link")) {
        return false;  // no segment "kinova2/spherical_wrist_1_link" to attach "kinova2/spherical_wrist_2_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova1/bracelet_link",
            KDL::Joint("kinova1/joint_7", KDL::Vector(0.0, -0.10593, -0.00017505), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.10593, -0.00017505)),
            KDL::RigidBodyInertia(0.5, KDL::Vector(0.000281, 0.011402, -0.029798),
            KDL::RotationalInertia(0.0005869997255105773, 0.0003690003455320899, 0.0006089999289573331, 3.00185764659545e-06, 3.000803527862622e-06, -0.00011800055321349603))),
          "kinova1/spherical_wrist_2_link")) {
        return false;  // no segment "kinova1/spherical_wrist_2_link" to attach "kinova1/bracelet_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("kinova2/bracelet_link",
            KDL::Joint("kinova2/joint_7", KDL::Vector(0.0, -0.10593, -0.00017505), KDL::Vector(0.0, 1.0, 2.220446049250313e-16), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865477), KDL::Vector(0.0, -0.10593, -0.00017505)),
            KDL::RigidBodyInertia(0.5, KDL::Vector(0.000281, 0.011402, -0.029798),
            KDL::RotationalInertia(0.0005869997255105773, 0.0003690003455320899, 0.0006089999289573331, 3.00185764659545e-06, 3.000803527862622e-06, -0.00011800055321349603))),
          "kinova2/spherical_wrist_2_link")) {
        return false;  // no segment "kinova2/spherical_wrist_2_link" to attach "kinova2/bracelet_link" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper1/g_base_mount",
            KDL::Joint("arm1_tree/gripper1_on_arm1", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, -8.572527594031473e-19, -0.068525)),
            KDL::RigidBodyInertia(0.15000285516377976, KDL::Vector(-0.0003605835958214157, 8.536274800937428e-05, -5.9199141450506514e-05),
            KDL::RotationalInertia(5.217217253247701e-05, 5.359968409474146e-05, 0.00010234508271477983, -2.1121394872370117e-09, -1.4277807079157603e-07, -6.010509618238863e-12))),
          "kinova1/bracelet_link")) {
        return false;  // no segment "kinova1/bracelet_link" to attach "gripper1/g_base_mount" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper2/g_base_mount",
            KDL::Joint("arm2_tree/gripper2_on_arm2", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 6.123233995736766e-17), KDL::Vector(0.0, -8.572527594031473e-19, -0.068525)),
            KDL::RigidBodyInertia(0.15000285516377976, KDL::Vector(-0.0003605835958214157, 8.536274800937428e-05, -5.9199141450506514e-05),
            KDL::RotationalInertia(5.217217253247701e-05, 5.359968409474146e-05, 0.00010234508271477983, -2.1121394872370117e-09, -1.4277807079157603e-07, -6.010509618238863e-12))),
          "kinova2/bracelet_link")) {
        return false;  // no segment "kinova2/bracelet_link" to attach "gripper2/g_base_mount" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper1/g_base",
            KDL::Joint("gripper1/g_base_on_mount", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)),
            KDL::RigidBodyInertia(0.9026054836789594, KDL::Vector(0.0, -2.3289840972067775e-05, 0.04247701482094086),
            KDL::RotationalInertia(0.0009287010595313735, 0.0005835572393484526, 0.0004776404071743112, 1.202439660816404e-11, -3.605993341917436e-16, -3.695091338611354e-07))),
          "gripper1/g_base_mount")) {
        return false;  // no segment "gripper1/g_base_mount" to attach "gripper1/g_base" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper2/g_base",
            KDL::Joint("gripper2/g_base_on_mount", KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)),
            KDL::RigidBodyInertia(0.9026054836789594, KDL::Vector(0.0, -2.3289840972067775e-05, 0.04247701482094086),
            KDL::RotationalInertia(0.0009287010595313735, 0.0005835572393484526, 0.0004776404071743112, 1.202439660816404e-11, -3.605993341917436e-16, -3.695091338611354e-07))),
          "gripper2/g_base_mount")) {
        return false;  // no segment "gripper2/g_base_mount" to attach "gripper2/g_base" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper1/g_left_driver",
            KDL::Joint("gripper1/g_left_driver_joint", KDL::Vector(0.0, -0.0306011, 0.054904), KDL::Vector(-1.0, 1.2246467991473532e-16, 0.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 1.0, 6.123233995736766e-17), KDL::Vector(0.0, -0.0306011, 0.054904)),
            KDL::RigidBodyInertia(0.00899563, KDL::Vector(0.0, 0.0177547, 0.00107314),
            KDL::RotationalInertia(1.7235199999999992e-06, 3.286147803680565e-07, 1.6024512196319424e-06, 0.0, 0.0, -9.199011484869232e-08))),
          "gripper1/g_base")) {
        return false;  // no segment "gripper1/g_base" to attach "gripper1/g_left_driver" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper2/g_left_driver",
            KDL::Joint("gripper2/g_left_driver_joint", KDL::Vector(0.0, -0.0306011, 0.054904), KDL::Vector(-1.0, 1.2246467991473532e-16, 0.0), KDL::Joint::RotAxis),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 1.0, 6.123233995736766e-17), KDL::Vector(0.0, -0.0306011, 0.054904)),
            KDL::RigidBodyInertia(0.00899563, KDL::Vector(0.0, 0.0177547, 0.00107314),
            KDL::RotationalInertia(1.7235199999999992e-06, 3.286147803680565e-07, 1.6024512196319424e-06, 0.0, 0.0, -9.199011484869232e-08))),
          "gripper2/g_base")) {
        return false;  // no segment "gripper2/g_base" to attach "gripper2/g_left_driver" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper1/g_base/g_pinch",
            KDL::Joint(KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.145)),
            KDL::RigidBodyInertia::Zero()),
          "gripper1/g_base")) {
        return false;  // no segment "gripper1/g_base" to attach "gripper1/g_base/g_pinch" to
    }
    if (!tree->addSegment(
          KDL::Segment("gripper2/g_base/g_pinch",
            KDL::Joint(KDL::Joint::None),
            KDL::Frame(KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.145)),
            KDL::RigidBodyInertia::Zero()),
          "gripper2/g_base")) {
        return false;  // no segment "gripper2/g_base" to attach "gripper2/g_base/g_pinch" to
    }
    return true;
}

// Slices the chain out of a tree make_tree_world_tree built, so several
// chains over one tree build it once.
inline bool make_chain_arm1_tree_chain(const KDL::Tree &tree, KDL::Chain *chain)
{
    // 'kinova1/base_link' -> 'gripper1/g_base/g_pinch'
    return tree.getChain("kinova1/base_link", "gripper1/g_base/g_pinch", *chain);
}

// Slices the chain out of a tree make_tree_world_tree built, so several
// chains over one tree build it once.
inline bool make_chain_arm2_tree_chain(const KDL::Tree &tree, KDL::Chain *chain)
{
    // 'kinova2/base_link' -> 'gripper2/g_base/g_pinch'
    return tree.getChain("kinova2/base_link", "gripper2/g_base/g_pinch", *chain);
}


// The model element each name in this header stands for. KDL knows a segment or joint by the
// name it is built with, and a tree or chain by the function that makes it, so those are the
// names to look up: what the scene called the thing is not recoverable from them otherwise.
inline const std::unordered_map<std::string_view, std::string_view> kIris = {
    {"world_tree", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree"},
    {"world_tree/world_body", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/world_body"},
    {"world_tree/table", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table"},
    {"world_tree/table_on_world", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/table_on_world"},
    {"kinova1/base_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/base_link"},
    {"world_tree/arm1_on_table", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/arm1_on_table"},
    {"kinova2/base_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/base_link"},
    {"world_tree/arm2_on_table", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/world_tree/arm2_on_table"},
    {"kinova1/shoulder_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/shoulder_link"},
    {"kinova1/joint_1", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_1"},
    {"kinova2/shoulder_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/shoulder_link"},
    {"kinova2/joint_1", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_1"},
    {"kinova1/half_arm_1_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_1_link"},
    {"kinova1/joint_2", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_2"},
    {"kinova2/half_arm_1_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_1_link"},
    {"kinova2/joint_2", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_2"},
    {"kinova1/half_arm_2_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/half_arm_2_link"},
    {"kinova1/joint_3", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_3"},
    {"kinova2/half_arm_2_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/half_arm_2_link"},
    {"kinova2/joint_3", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_3"},
    {"kinova1/forearm_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/forearm_link"},
    {"kinova1/joint_4", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_4"},
    {"kinova2/forearm_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/forearm_link"},
    {"kinova2/joint_4", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_4"},
    {"kinova1/spherical_wrist_1_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_1_link"},
    {"kinova1/joint_5", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_5"},
    {"kinova2/spherical_wrist_1_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_1_link"},
    {"kinova2/joint_5", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_5"},
    {"kinova1/spherical_wrist_2_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/spherical_wrist_2_link"},
    {"kinova1/joint_6", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_6"},
    {"kinova2/spherical_wrist_2_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/spherical_wrist_2_link"},
    {"kinova2/joint_6", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_6"},
    {"kinova1/bracelet_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/bracelet_link"},
    {"kinova1/joint_7", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova1/joint_7"},
    {"kinova2/bracelet_link", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/bracelet_link"},
    {"kinova2/joint_7", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/kinova2/joint_7"},
    {"gripper1/g_base_mount", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_mount"},
    {"arm1_tree/gripper1_on_arm1", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/arm1_tree/gripper1_on_arm1"},
    {"gripper2/g_base_mount", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_mount"},
    {"arm2_tree/gripper2_on_arm2", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/arm2_tree/gripper2_on_arm2"},
    {"gripper1/g_base", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base"},
    {"gripper1/g_base_on_mount", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base_on_mount"},
    {"gripper2/g_base", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base"},
    {"gripper2/g_base_on_mount", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base_on_mount"},
    {"gripper1/g_left_driver", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver"},
    {"gripper1/g_left_driver_joint", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_left_driver_joint"},
    {"gripper2/g_left_driver", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver"},
    {"gripper2/g_left_driver_joint", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_left_driver_joint"},
    {"gripper1/g_base/g_pinch", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper1/g_base/g_pinch"},
    {"gripper2/g_base/g_pinch", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/gripper2/g_base/g_pinch"},
    {"arm1_tree_chain", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/arm1_tree/chain"},
    {"arm2_tree_chain", "https://secorolab.github.io/models/scenes/pick-place-dual-mjc/arm2_tree/chain"},
};

}  // namespace scene_kdl
