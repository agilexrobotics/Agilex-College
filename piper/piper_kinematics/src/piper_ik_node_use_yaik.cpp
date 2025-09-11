#include "yaik_cpp_common.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <Eigen/Geometry>
#include <memory>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>
#include <deque>
#include <piper_msgs/PosCmd.h>
#include <std_msgs/Float64.h>
#include <algorithm>

using namespace yaik_cpp;

struct spherical_wrist_six_axis_ik {

// Constants for solver
static constexpr int robot_nq = 6;
static constexpr int max_n_solutions = 16;
static constexpr int n_tree_nodes = 24;
static constexpr int intermediate_solution_size = 9;
static constexpr double pose_tolerance = 1e-6;
static constexpr double pose_tolerance_degenerate = 1e-4;
static constexpr double zero_tolerance = 1e-6;
using RawIKWorksace = ::yaik_cpp::RawIkWorkspaceTemplate<intermediate_solution_size, max_n_solutions, robot_nq>;

// Robot parameters
static constexpr double a_0 = 0.32;
static constexpr double a_1 = 1.28;
static constexpr double a_2 = 0.2;
static constexpr double d_3 = 1.1825;
static constexpr double d_4 = 0.2;
static constexpr double pre_transform_special_symbol_23 = 0.78;

// Unknown offsets from original unknown value to raw value
// Original value are the ones corresponded to robot (usually urdf/sdf)
// Raw value are the ones used in the solver
// unknown_i_raw = unknown_i_original + unknown_i_offset_original2raw
static constexpr double th_0_offset_original2raw = 0.0;
static constexpr double th_1_offset_original2raw = -1.5707963267948966;
static constexpr double th_2_offset_original2raw = 0.0;
static constexpr double th_3_offset_original2raw = 3.141592653589793;
static constexpr double th_4_offset_original2raw = 3.141592653589793;
static constexpr double th_5_offset_original2raw = 0.0;

// The transformation between raw and original ee target
// Original value are the ones corresponded to robot (usually urdf/sdf)
// Raw value are the ones used in the solver
// ee_original = pre_transform * ee_raw * post_transform
// ee_raw = dh_forward_transform(theta_raw)
static Eigen::Matrix4d endEffectorTargetOriginalToRaw(const Eigen::Matrix4d& T_ee)
{
    const double r_11 = T_ee(0, 0);
    const double r_12 = T_ee(0, 1);
    const double r_13 = T_ee(0, 2);
    const double Px = T_ee(0, 3);
    const double r_21 = T_ee(1, 0);
    const double r_22 = T_ee(1, 1);
    const double r_23 = T_ee(1, 2);
    const double Py = T_ee(1, 3);
    const double r_31 = T_ee(2, 0);
    const double r_32 = T_ee(2, 1);
    const double r_33 = T_ee(2, 2);
    const double Pz = T_ee(2, 3);
    Eigen::Matrix4d ee_transformed;
    ee_transformed.setIdentity();
    ee_transformed(0, 0) = -1.0*r_11 + 6.1232339957367697e-17*r_13;
    ee_transformed(0, 1) = -1.0*r_12;
    ee_transformed(0, 2) = 6.1232339957367697e-17*r_11 + 1.0*r_13;
    ee_transformed(0, 3) = 1.0*Px;
    ee_transformed(1, 0) = -1.0*r_21 + 6.1232339957367697e-17*r_23;
    ee_transformed(1, 1) = -1.0*r_22;
    ee_transformed(1, 2) = 6.1232339957367697e-17*r_21 + 1.0*r_23;
    ee_transformed(1, 3) = 1.0*Py;
    ee_transformed(2, 0) = -1.0*r_31 + 6.1232339957367697e-17*r_33;
    ee_transformed(2, 1) = -1.0*r_32;
    ee_transformed(2, 2) = 6.1232339957367697e-17*r_31 + 1.0*r_33;
    ee_transformed(2, 3) = 1.0*Pz - 1.0*pre_transform_special_symbol_23;
    return ee_transformed;
}

static Eigen::Matrix4d endEffectorTargetRawToOriginal(const Eigen::Matrix4d& T_ee)
{
    const double r_11 = T_ee(0, 0);
    const double r_12 = T_ee(0, 1);
    const double r_13 = T_ee(0, 2);
    const double Px = T_ee(0, 3);
    const double r_21 = T_ee(1, 0);
    const double r_22 = T_ee(1, 1);
    const double r_23 = T_ee(1, 2);
    const double Py = T_ee(1, 3);
    const double r_31 = T_ee(2, 0);
    const double r_32 = T_ee(2, 1);
    const double r_33 = T_ee(2, 2);
    const double Pz = T_ee(2, 3);
    Eigen::Matrix4d ee_transformed;
    ee_transformed.setIdentity();
    ee_transformed(0, 0) = -1.0*r_11 + 6.1232339957367697e-17*r_13;
    ee_transformed(0, 1) = -1.0*r_12;
    ee_transformed(0, 2) = 6.1232339957367697e-17*r_11 + 1.0*r_13;
    ee_transformed(0, 3) = 1.0*Px;
    ee_transformed(1, 0) = -1.0*r_21 + 6.1232339957367697e-17*r_23;
    ee_transformed(1, 1) = -1.0*r_22;
    ee_transformed(1, 2) = 6.1232339957367697e-17*r_21 + 1.0*r_23;
    ee_transformed(1, 3) = 1.0*Py;
    ee_transformed(2, 0) = -1.0*r_31 + 6.1232339957367697e-17*r_33;
    ee_transformed(2, 1) = -1.0*r_32;
    ee_transformed(2, 2) = 6.1232339957367697e-17*r_31 + 1.0*r_33;
    ee_transformed(2, 3) = 1.0*Pz + 1.0*pre_transform_special_symbol_23;
    return ee_transformed;
}

///************* Below are the actual FK and IK implementations *******************
static Eigen::Matrix4d computeFK(const std::array<double, robot_nq>& theta_input_original)
{
    // Extract the variables
    const double th_0 = theta_input_original[0] + th_0_offset_original2raw;
    const double th_1 = theta_input_original[1] + th_1_offset_original2raw;
    const double th_2 = theta_input_original[2] + th_2_offset_original2raw;
    const double th_3 = theta_input_original[3] + th_3_offset_original2raw;
    const double th_4 = theta_input_original[4] + th_4_offset_original2raw;
    const double th_5 = theta_input_original[5] + th_5_offset_original2raw;
    
    // Temp variable for efficiency
    const double x0 = std::sin(th_5);
    const double x1 = std::sin(th_0);
    const double x2 = std::cos(th_3);
    const double x3 = std::sin(th_3);
    const double x4 = std::cos(th_0);
    const double x5 = std::sin(th_1);
    const double x6 = std::sin(th_2);
    const double x7 = x5*x6;
    const double x8 = std::cos(th_1);
    const double x9 = std::cos(th_2);
    const double x10 = -x4*x7 + x4*x8*x9;
    const double x11 = x1*x2 - x10*x3;
    const double x12 = std::cos(th_5);
    const double x13 = std::sin(th_4);
    const double x14 = x5*x9;
    const double x15 = x6*x8;
    const double x16 = -x14*x4 - x15*x4;
    const double x17 = std::cos(th_4);
    const double x18 = x1*x3 + x10*x2;
    const double x19 = -x13*x16 + x17*x18;
    const double x20 = -x13*x18 - x16*x17;
    const double x21 = a_1*x8;
    const double x22 = -x1*x7 + x1*x8*x9;
    const double x23 = -x2*x4 - x22*x3;
    const double x24 = -x1*x14 - x1*x15;
    const double x25 = x2*x22 - x3*x4;
    const double x26 = -x13*x24 + x17*x25;
    const double x27 = -x13*x25 - x17*x24;
    const double x28 = -x14 - x15;
    const double x29 = x7 - x8*x9;
    const double x30 = -x13*x29 + x17*x2*x28;
    const double x31 = -x13*x2*x28 - x17*x29;
    // End of temp variables
    Eigen::Matrix4d ee_pose_raw;
    ee_pose_raw.setIdentity();
    ee_pose_raw(0, 0) = -x0*x11 + x12*x19;
    ee_pose_raw(0, 1) = -x0*x19 - x11*x12;
    ee_pose_raw(0, 2) = x20;
    ee_pose_raw(0, 3) = a_0*x4 + a_2*x10 + d_3*x16 + d_4*x20 + x21*x4;
    ee_pose_raw(1, 0) = -x0*x23 + x12*x26;
    ee_pose_raw(1, 1) = -x0*x26 - x12*x23;
    ee_pose_raw(1, 2) = x27;
    ee_pose_raw(1, 3) = a_0*x1 + a_2*x22 + d_3*x24 + d_4*x27 + x1*x21;
    ee_pose_raw(2, 0) = x0*x28*x3 + x12*x30;
    ee_pose_raw(2, 1) = -x0*x30 + x12*x28*x3;
    ee_pose_raw(2, 2) = x31;
    ee_pose_raw(2, 3) = -a_1*x5 + a_2*x28 + d_3*x29 + d_4*x31;
    return endEffectorTargetRawToOriginal(ee_pose_raw);
}

static void computeTwistJacobian(const std::array<double, robot_nq>& theta_input_original, Eigen::Matrix<double, 6, robot_nq>& jacobian)
{
    // Extract the variables
    const double th_0 = theta_input_original[0] + th_0_offset_original2raw;
    const double th_1 = theta_input_original[1] + th_1_offset_original2raw;
    const double th_2 = theta_input_original[2] + th_2_offset_original2raw;
    const double th_3 = theta_input_original[3] + th_3_offset_original2raw;
    const double th_4 = theta_input_original[4] + th_4_offset_original2raw;
    const double th_5 = theta_input_original[5] + th_5_offset_original2raw;
    
    // Temp variable for efficiency
    const double x0 = std::sin(th_0);
    const double x1 = 1.0*x0;
    const double x2 = -x1;
    const double x3 = std::cos(th_2);
    const double x4 = std::sin(th_1);
    const double x5 = std::cos(th_0);
    const double x6 = 1.0*x5;
    const double x7 = x4*x6;
    const double x8 = std::sin(th_2);
    const double x9 = std::cos(th_1);
    const double x10 = x6*x9;
    const double x11 = -x10*x8 - x3*x7;
    const double x12 = std::cos(th_3);
    const double x13 = std::sin(th_3);
    const double x14 = 1.0*x3*x5*x9 - x7*x8;
    const double x15 = 1.0*x0*x12 - x13*x14;
    const double x16 = std::cos(th_4);
    const double x17 = std::sin(th_4);
    const double x18 = -x11*x16 - x17*(x1*x13 + x12*x14);
    const double x19 = x1*x4;
    const double x20 = x1*x9;
    const double x21 = -x19*x3 - x20*x8;
    const double x22 = 1.0*x0*x3*x9 - x19*x8;
    const double x23 = -x12*x6 - x13*x22;
    const double x24 = -x16*x21 - x17*(x12*x22 - x13*x6);
    const double x25 = 1.0*x8;
    const double x26 = 1.0*x3;
    const double x27 = x25*x4 - x26*x9;
    const double x28 = -x25*x9 - x26*x4;
    const double x29 = x13*x28;
    const double x30 = -x12*x17*x28 - x16*x27;
    const double x31 = 1.0*a_1*x4;
    const double x32 = pre_transform_special_symbol_23 - x31;
    const double x33 = a_2*x28 + d_3*x27 + pre_transform_special_symbol_23 - x31;
    const double x34 = a_0*x1 + a_1*x20;
    const double x35 = a_2*x22 + d_3*x21 + x34;
    const double x36 = d_4*x30 + x33;
    const double x37 = d_4*x24 + x35;
    const double x38 = a_0*x6 + a_1*x10;
    const double x39 = a_2*x14 + d_3*x11 + x38;
    const double x40 = d_4*x18 + x39;
    const double x41 = 1.0*a_0;
    // End of temp variables
    
    jacobian.setZero();
    jacobian(0, 1) = x2;
    jacobian(0, 2) = x2;
    jacobian(0, 3) = x11;
    jacobian(0, 4) = x15;
    jacobian(0, 5) = x18;
    jacobian(1, 1) = x6;
    jacobian(1, 2) = x6;
    jacobian(1, 3) = x21;
    jacobian(1, 4) = x23;
    jacobian(1, 5) = x24;
    jacobian(2, 0) = 1.0;
    jacobian(2, 3) = x27;
    jacobian(2, 4) = -x29;
    jacobian(2, 5) = x30;
    jacobian(3, 1) = -pre_transform_special_symbol_23*x6;
    jacobian(3, 2) = -x32*x6;
    jacobian(3, 3) = -x21*x33 + x27*x35;
    jacobian(3, 4) = -x23*x33 - x29*x35;
    jacobian(3, 5) = -x24*x36 + x30*x37;
    jacobian(4, 1) = -pre_transform_special_symbol_23*x1;
    jacobian(4, 2) = -x1*x32;
    jacobian(4, 3) = x11*x33 - x27*x39;
    jacobian(4, 4) = x15*x33 + x29*x39;
    jacobian(4, 5) = x18*x36 - x30*x40;
    jacobian(5, 1) = std::pow(x0, 2)*x41 + x41*std::pow(x5, 2);
    jacobian(5, 2) = x1*x34 + x38*x6;
    jacobian(5, 3) = -x11*x35 + x21*x39;
    jacobian(5, 4) = -x15*x35 + x23*x39;
    jacobian(5, 5) = -x18*x37 + x24*x40;
    return;
}

static void computeAngularVelocityJacobian(const std::array<double, robot_nq>& theta_input_original, Eigen::Matrix<double, 3, robot_nq>& jacobian)
{
    // Extract the variables
    const double th_0 = theta_input_original[0] + th_0_offset_original2raw;
    const double th_1 = theta_input_original[1] + th_1_offset_original2raw;
    const double th_2 = theta_input_original[2] + th_2_offset_original2raw;
    const double th_3 = theta_input_original[3] + th_3_offset_original2raw;
    const double th_4 = theta_input_original[4] + th_4_offset_original2raw;
    const double th_5 = theta_input_original[5] + th_5_offset_original2raw;
    
    // Temp variable for efficiency
    const double x0 = std::sin(th_0);
    const double x1 = 1.0*x0;
    const double x2 = -x1;
    const double x3 = std::cos(th_2);
    const double x4 = std::sin(th_1);
    const double x5 = std::cos(th_0);
    const double x6 = 1.0*x5;
    const double x7 = x4*x6;
    const double x8 = std::sin(th_2);
    const double x9 = std::cos(th_1);
    const double x10 = -x3*x7 - x6*x8*x9;
    const double x11 = std::cos(th_3);
    const double x12 = std::sin(th_3);
    const double x13 = 1.0*x3*x5*x9 - x7*x8;
    const double x14 = std::cos(th_4);
    const double x15 = std::sin(th_4);
    const double x16 = x1*x4;
    const double x17 = -x1*x8*x9 - x16*x3;
    const double x18 = 1.0*x0*x3*x9 - x16*x8;
    const double x19 = 1.0*x8;
    const double x20 = 1.0*x3;
    const double x21 = x19*x4 - x20*x9;
    const double x22 = -x19*x9 - x20*x4;
    // End of temp variables
    
    jacobian.setZero();
    jacobian(0, 1) = x2;
    jacobian(0, 2) = x2;
    jacobian(0, 3) = x10;
    jacobian(0, 4) = 1.0*x0*x11 - x12*x13;
    jacobian(0, 5) = -x10*x14 - x15*(x1*x12 + x11*x13);
    jacobian(1, 1) = x6;
    jacobian(1, 2) = x6;
    jacobian(1, 3) = x17;
    jacobian(1, 4) = -x11*x6 - x12*x18;
    jacobian(1, 5) = -x14*x17 - x15*(x11*x18 - x12*x6);
    jacobian(2, 0) = 1.0;
    jacobian(2, 3) = x21;
    jacobian(2, 4) = -x12*x22;
    jacobian(2, 5) = -x11*x15*x22 - x14*x21;
    return;
}

static void computeTransformPointJacobian(const std::array<double, robot_nq>& theta_input_original, Eigen::Vector3d& point_on_ee, Eigen::Matrix<double, 3, robot_nq>& jacobian)
{
    // Extract the variables
    const double th_0 = theta_input_original[0] + th_0_offset_original2raw;
    const double th_1 = theta_input_original[1] + th_1_offset_original2raw;
    const double th_2 = theta_input_original[2] + th_2_offset_original2raw;
    const double th_3 = theta_input_original[3] + th_3_offset_original2raw;
    const double th_4 = theta_input_original[4] + th_4_offset_original2raw;
    const double th_5 = theta_input_original[5] + th_5_offset_original2raw;
    const double p_on_ee_x = point_on_ee[0];
    const double p_on_ee_y = point_on_ee[1];
    const double p_on_ee_z = point_on_ee[2];
    
    // Temp variable for efficiency
    const double x0 = 1.0*p_on_ee_y;
    const double x1 = std::cos(th_0);
    const double x2 = 1.0*x1;
    const double x3 = p_on_ee_z*x2;
    const double x4 = std::sin(th_1);
    const double x5 = 1.0*x4;
    const double x6 = a_1*x5;
    const double x7 = pre_transform_special_symbol_23 - x6;
    const double x8 = std::sin(th_2);
    const double x9 = x5*x8;
    const double x10 = std::cos(th_2);
    const double x11 = std::cos(th_1);
    const double x12 = 1.0*x11;
    const double x13 = -x10*x12 + x9;
    const double x14 = std::sin(th_0);
    const double x15 = x10*x5;
    const double x16 = x12*x8;
    const double x17 = -x14*x15 - x14*x16;
    const double x18 = -x15 - x16;
    const double x19 = a_2*x18 + d_3*x13 + pre_transform_special_symbol_23 - x6;
    const double x20 = 1.0*x10*x11*x14 - x14*x9;
    const double x21 = 1.0*x14;
    const double x22 = a_0*x21 + a_1*x12*x14;
    const double x23 = a_2*x20 + d_3*x17 + x22;
    const double x24 = std::sin(th_3);
    const double x25 = x18*x24;
    const double x26 = std::cos(th_3);
    const double x27 = -x2*x26 - x20*x24;
    const double x28 = std::cos(th_4);
    const double x29 = std::sin(th_4);
    const double x30 = -x13*x28 - x18*x26*x29;
    const double x31 = -x17*x28 - x29*(-x2*x24 + x20*x26);
    const double x32 = d_4*x30 + x19;
    const double x33 = d_4*x31 + x23;
    const double x34 = 1.0*p_on_ee_x;
    const double x35 = p_on_ee_z*x21;
    const double x36 = x2*x4;
    const double x37 = x11*x2;
    const double x38 = -x10*x36 - x37*x8;
    const double x39 = 1.0*x1*x10*x11 - x36*x8;
    const double x40 = a_0*x2 + a_1*x37;
    const double x41 = a_2*x39 + d_3*x38 + x40;
    const double x42 = 1.0*x14*x26 - x24*x39;
    const double x43 = -x28*x38 - x29*(x21*x24 + x26*x39);
    const double x44 = d_4*x43 + x41;
    const double x45 = x1*x34;
    const double x46 = x0*x14;
    const double x47 = 1.0*a_0;
    // End of temp variables
    
    jacobian.setZero();
    jacobian(0, 0) = -x0;
    jacobian(0, 1) = -pre_transform_special_symbol_23*x2 + x3;
    jacobian(0, 2) = -x2*x7 + x3;
    jacobian(0, 3) = -p_on_ee_y*x13 + p_on_ee_z*x17 + x13*x23 - x17*x19;
    jacobian(0, 4) = p_on_ee_y*x25 + p_on_ee_z*x27 - x19*x27 - x23*x25;
    jacobian(0, 5) = -p_on_ee_y*x30 + p_on_ee_z*x31 + x30*x33 - x31*x32;
    jacobian(1, 0) = x34;
    jacobian(1, 1) = -pre_transform_special_symbol_23*x21 + x35;
    jacobian(1, 2) = -x21*x7 + x35;
    jacobian(1, 3) = p_on_ee_x*x13 - p_on_ee_z*x38 - x13*x41 + x19*x38;
    jacobian(1, 4) = -p_on_ee_x*x25 - p_on_ee_z*x42 + x18*x24*x41 + x19*x42;
    jacobian(1, 5) = p_on_ee_x*x30 - p_on_ee_z*x43 - x30*x44 + x32*x43;
    jacobian(2, 1) = std::pow(x1, 2)*x47 + std::pow(x14, 2)*x47 - x45 - x46;
    jacobian(2, 2) = 1.0*x1*x40 + 1.0*x14*x22 - x45 - x46;
    jacobian(2, 3) = -p_on_ee_x*x17 + p_on_ee_y*x38 + x17*x41 - x23*x38;
    jacobian(2, 4) = -p_on_ee_x*x27 + p_on_ee_y*x42 - x23*x42 + x27*x41;
    jacobian(2, 5) = -p_on_ee_x*x31 + p_on_ee_y*x43 + x31*x44 - x33*x43;
    return;
}

static void computeRawIK(const Eigen::Matrix4d& T_ee, SolutionQueue<intermediate_solution_size, max_n_solutions>& solution_queue, NodeIndexWorkspace<max_n_solutions>& node_index_workspace, std::vector<std::array<double, robot_nq>>& ik_output)
{
    // Extracting the ik target symbols
    const double r_11 = T_ee(0, 0);
    const double r_12 = T_ee(0, 1);
    const double r_13 = T_ee(0, 2);
    const double Px = T_ee(0, 3);
    const double r_21 = T_ee(1, 0);
    const double r_22 = T_ee(1, 1);
    const double r_23 = T_ee(1, 2);
    const double Py = T_ee(1, 3);
    const double r_31 = T_ee(2, 0);
    const double r_32 = T_ee(2, 1);
    const double r_33 = T_ee(2, 2);
    const double Pz = T_ee(2, 3);
    const auto& ee_rotation = T_ee.block<3, 3>(0, 0);
    const auto& ee_translation = T_ee.block<3, 1>(0, 3);
    const Eigen::Vector3d inv_ee_translation = - ee_rotation.transpose() * ee_translation;
    const double inv_Px = inv_ee_translation(0);
    const double inv_Py = inv_ee_translation(1);
    const double inv_Pz = inv_ee_translation(2);
    
    solution_queue.reset();
    node_index_workspace.reset(n_tree_nodes);
    
    using RawSolution = IntermediateSolution<intermediate_solution_size>;
    auto make_raw_solution = []() -> RawSolution { return {}; };
    
    auto append_solution_to_queue = [&solution_queue](RawSolution solution_2_add) -> int {
        return solution_queue.push_solution(solution_2_add);
    };
    
    auto add_input_index_to = [&node_index_workspace](int node_idx, int solution_idx) -> void {
        if (solution_idx < 0) return;
        node_index_workspace.append_index_to_node(node_idx, solution_idx);
    };
    
    // Code for equation all-zero dispatcher node 0
    auto EquationAllZeroDispatcherNode_node_0_processor= [&]()
    {
        const bool checked_result = std::fabs(Px - d_4*r_13) <= 9.9999999999999995e-7 && std::fabs(Py - d_4*r_23) <= 9.9999999999999995e-7;
        if (!checked_result)  // To non-degenerate node
            node_index_workspace.node_input_validity_vector[1] = true;
    };
    
    // Invoke the processor
    EquationAllZeroDispatcherNode_node_0_processor();
    // Finish code for equation all-zero dispatcher node 0
    
    // Code for explicit solution node 1, solved variable is th_0
    auto ExplicitSolutionNode_node_1_solve_th_0_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(1);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(1);
        if (!this_input_valid)
            return;
        
        // The explicit solution of root node
        const bool condition_0 = std::fabs(Px - d_4*r_13) >= zero_tolerance || std::fabs(Py - d_4*r_23) >= zero_tolerance;
        if (condition_0)
        {
            // Temp variable for efficiency
            const double x0 = std::atan2(Py - d_4*r_23, Px - d_4*r_13);
            // End of temp variables
            
            auto solution_0 = make_raw_solution();
            solution_0[1] = x0;
            int appended_idx = append_solution_to_queue(solution_0);
            add_input_index_to(2, appended_idx);
        }
        
        const bool condition_1 = std::fabs(Px - d_4*r_13) >= zero_tolerance || std::fabs(Py - d_4*r_23) >= zero_tolerance;
        if (condition_1)
        {
            // Temp variable for efficiency
            const double x0 = std::atan2(Py - d_4*r_23, Px - d_4*r_13);
            // End of temp variables
            
            auto solution_1 = make_raw_solution();
            solution_1[1] = x0 + M_PI;
            int appended_idx = append_solution_to_queue(solution_1);
            add_input_index_to(2, appended_idx);
        }
        
    };
    // Invoke the processor
    ExplicitSolutionNode_node_1_solve_th_0_processor();
    // Finish code for explicit solution node 1
    
    // Code for non-branch dispatcher node 2
    // Actually, there is no code
    
    // Code for explicit solution node 3, solved variable is th_2
    auto ExplicitSolutionNode_node_3_solve_th_2_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(2);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(2);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 3
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            
            const bool condition_0 = 2*std::fabs(a_1*a_2) >= zero_tolerance || 2*std::fabs(a_1*d_3) >= zero_tolerance || std::fabs(std::pow(Px, 2) - 2*Px*a_0*std::cos(th_0) - 2*Px*d_4*r_13 + std::pow(Py, 2) - 2*Py*a_0*std::sin(th_0) - 2*Py*d_4*r_23 + std::pow(Pz, 2) - 2*Pz*d_4*r_33 + std::pow(a_0, 2) + 2*a_0*d_4*r_13*std::cos(th_0) + 2*a_0*d_4*r_23*std::sin(th_0) - std::pow(a_1, 2) - std::pow(a_2, 2) - std::pow(d_3, 2) + std::pow(d_4, 2)*std::pow(r_13, 2) + std::pow(d_4, 2)*std::pow(r_23, 2) + std::pow(d_4, 2)*std::pow(r_33, 2)) >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = 2*a_1;
                const double x1 = std::atan2(-d_3*x0, a_2*x0);
                const double x2 = std::pow(a_2, 2);
                const double x3 = std::pow(a_1, 2);
                const double x4 = 4*x3;
                const double x5 = std::pow(d_3, 2);
                const double x6 = 2*Px;
                const double x7 = d_4*r_13;
                const double x8 = 2*Py;
                const double x9 = d_4*r_23;
                const double x10 = a_0*std::cos(th_0);
                const double x11 = a_0*std::sin(th_0);
                const double x12 = std::pow(d_4, 2);
                const double x13 = std::pow(Px, 2) + std::pow(Py, 2) + std::pow(Pz, 2) - 2*Pz*d_4*r_33 + std::pow(a_0, 2) + std::pow(r_13, 2)*x12 + std::pow(r_23, 2)*x12 + std::pow(r_33, 2)*x12 - x10*x6 + 2*x10*x7 - x11*x8 + 2*x11*x9 - x2 - x3 - x5 - x6*x7 - x8*x9;
                const double x14 = safe_sqrt(-std::pow(x13, 2) + x2*x4 + x4*x5);
                // End of temp variables
                RawSolution solution_0(this_solution);
                solution_0[4] = x1 + std::atan2(x14, x13);
                int appended_idx = append_solution_to_queue(solution_0);
                add_input_index_to(4, appended_idx);
            }
            
            const bool condition_1 = 2*std::fabs(a_1*a_2) >= zero_tolerance || 2*std::fabs(a_1*d_3) >= zero_tolerance || std::fabs(std::pow(Px, 2) - 2*Px*a_0*std::cos(th_0) - 2*Px*d_4*r_13 + std::pow(Py, 2) - 2*Py*a_0*std::sin(th_0) - 2*Py*d_4*r_23 + std::pow(Pz, 2) - 2*Pz*d_4*r_33 + std::pow(a_0, 2) + 2*a_0*d_4*r_13*std::cos(th_0) + 2*a_0*d_4*r_23*std::sin(th_0) - std::pow(a_1, 2) - std::pow(a_2, 2) - std::pow(d_3, 2) + std::pow(d_4, 2)*std::pow(r_13, 2) + std::pow(d_4, 2)*std::pow(r_23, 2) + std::pow(d_4, 2)*std::pow(r_33, 2)) >= zero_tolerance;
            if (condition_1)
            {
                // Temp variable for efficiency
                const double x0 = 2*a_1;
                const double x1 = std::atan2(-d_3*x0, a_2*x0);
                const double x2 = std::pow(a_2, 2);
                const double x3 = std::pow(a_1, 2);
                const double x4 = 4*x3;
                const double x5 = std::pow(d_3, 2);
                const double x6 = 2*Px;
                const double x7 = d_4*r_13;
                const double x8 = 2*Py;
                const double x9 = d_4*r_23;
                const double x10 = a_0*std::cos(th_0);
                const double x11 = a_0*std::sin(th_0);
                const double x12 = std::pow(d_4, 2);
                const double x13 = std::pow(Px, 2) + std::pow(Py, 2) + std::pow(Pz, 2) - 2*Pz*d_4*r_33 + std::pow(a_0, 2) + std::pow(r_13, 2)*x12 + std::pow(r_23, 2)*x12 + std::pow(r_33, 2)*x12 - x10*x6 + 2*x10*x7 - x11*x8 + 2*x11*x9 - x2 - x3 - x5 - x6*x7 - x8*x9;
                const double x14 = safe_sqrt(-std::pow(x13, 2) + x2*x4 + x4*x5);
                // End of temp variables
                const double tmp_sol_value = x1 + std::atan2(-x14, x13);
                solution_queue.get_solution(node_input_i_idx_in_queue)[4] = tmp_sol_value;
                add_input_index_to(4, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_3_solve_th_2_processor();
    // Finish code for explicit solution node 2
    
    // Code for equation all-zero dispatcher node 4
    auto EquationAllZeroDispatcherNode_node_4_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(4);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(4);
        if (!this_input_valid)
            return;
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            const bool checked_result = std::fabs(Pz - d_4*r_33) <= 9.9999999999999995e-7 && std::fabs(-Px*std::cos(th_0) - Py*std::sin(th_0) + a_0 + d_4*r_13*std::cos(th_0) + d_4*r_23*std::sin(th_0)) <= 9.9999999999999995e-7;
            if (!checked_result)  // To non-degenerate node
                add_input_index_to(5, node_input_i_idx_in_queue);
        }
    };
    
    // Invoke the processor
    EquationAllZeroDispatcherNode_node_4_processor();
    // Finish code for equation all-zero dispatcher node 4
    
    // Code for explicit solution node 5, solved variable is th_1th_2_soa
    auto ExplicitSolutionNode_node_5_solve_th_1th_2_soa_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(5);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(5);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 5
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            const double th_2 = this_solution[4];
            
            const bool condition_0 = std::fabs(Pz - d_4*r_33) >= 9.9999999999999995e-7 || std::fabs(-Px*std::cos(th_0) - Py*std::sin(th_0) + a_0 + d_4*r_13*std::cos(th_0) + d_4*r_23*std::sin(th_0)) >= 9.9999999999999995e-7;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = Pz - d_4*r_33;
                const double x1 = -a_1*std::cos(th_2) - a_2;
                const double x2 = a_1*std::sin(th_2) - d_3;
                const double x3 = std::cos(th_0);
                const double x4 = std::sin(th_0);
                const double x5 = -Px*x3 - Py*x4 + a_0 + d_4*r_13*x3 + d_4*r_23*x4;
                // End of temp variables
                const double tmp_sol_value = std::atan2(x0*x1 - x2*x5, x0*x2 + x1*x5);
                solution_queue.get_solution(node_input_i_idx_in_queue)[3] = tmp_sol_value;
                add_input_index_to(6, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_5_solve_th_1th_2_soa_processor();
    // Finish code for explicit solution node 5
    
    // Code for non-branch dispatcher node 6
    // Actually, there is no code
    
    // Code for explicit solution node 7, solved variable is th_1
    auto ExplicitSolutionNode_node_7_solve_th_1_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(6);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(6);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 7
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_1th_2_soa = this_solution[3];
            const double th_2 = this_solution[4];
            
            const bool condition_0 = 1 >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                // End of temp variables
                const double tmp_sol_value = th_1th_2_soa - th_2;
                solution_queue.get_solution(node_input_i_idx_in_queue)[2] = tmp_sol_value;
                add_input_index_to(8, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_7_solve_th_1_processor();
    // Finish code for explicit solution node 6
    
    // Code for non-branch dispatcher node 8
    // Actually, there is no code
    
    // Code for explicit solution node 9, solved variable is th_4
    auto ExplicitSolutionNode_node_9_solve_th_4_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(8);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(8);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 9
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            const double th_1 = this_solution[2];
            const double th_2 = this_solution[4];
            
            const bool condition_0 = std::fabs(r_13*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::cos(th_0) + r_23*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::sin(th_0) + r_33*(-std::sin(th_1)*std::sin(th_2) + std::cos(th_1)*std::cos(th_2))) <= 1;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = std::sin(th_1);
                const double x1 = std::sin(th_2);
                const double x2 = std::cos(th_1);
                const double x3 = std::cos(th_2);
                const double x4 = x0*x3 + x1*x2;
                const double x5 = safe_acos(r_13*x4*std::cos(th_0) + r_23*x4*std::sin(th_0) + r_33*(-x0*x1 + x2*x3));
                // End of temp variables
                RawSolution solution_0(this_solution);
                solution_0[7] = x5;
                int appended_idx = append_solution_to_queue(solution_0);
                add_input_index_to(10, appended_idx);
            }
            
            const bool condition_1 = std::fabs(r_13*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::cos(th_0) + r_23*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::sin(th_0) + r_33*(-std::sin(th_1)*std::sin(th_2) + std::cos(th_1)*std::cos(th_2))) <= 1;
            if (condition_1)
            {
                // Temp variable for efficiency
                const double x0 = std::sin(th_1);
                const double x1 = std::sin(th_2);
                const double x2 = std::cos(th_1);
                const double x3 = std::cos(th_2);
                const double x4 = x0*x3 + x1*x2;
                const double x5 = safe_acos(r_13*x4*std::cos(th_0) + r_23*x4*std::sin(th_0) + r_33*(-x0*x1 + x2*x3));
                // End of temp variables
                const double tmp_sol_value = -x5;
                solution_queue.get_solution(node_input_i_idx_in_queue)[7] = tmp_sol_value;
                add_input_index_to(10, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_9_solve_th_4_processor();
    // Finish code for explicit solution node 8
    
    // Code for solved_variable dispatcher node 10
    auto SolvedVariableDispatcherNode_node_10_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(10);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(10);
        if (!this_input_valid)
            return;
        
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            bool taken_by_degenerate = false;
            const double th_4 = this_solution[7];
            
            const bool degenerate_valid_0 = std::fabs(th_4) <= 9.9999999999999995e-7;
            if (degenerate_valid_0)
            {
                taken_by_degenerate = true;
                add_input_index_to(14, node_input_i_idx_in_queue);
            }
            
            const bool degenerate_valid_1 = std::fabs(th_4 - M_PI) <= 9.9999999999999995e-7;
            if (degenerate_valid_1)
            {
                taken_by_degenerate = true;
                add_input_index_to(19, node_input_i_idx_in_queue);
            }
            
            if (!taken_by_degenerate)
                add_input_index_to(11, node_input_i_idx_in_queue);
        }
    };
    
    // Invoke the processor
    SolvedVariableDispatcherNode_node_10_processor();
    // Finish code for solved_variable dispatcher node 10
    
    // Code for explicit solution node 19, solved variable is th_3th_5_soa
    auto ExplicitSolutionNode_node_19_solve_th_3th_5_soa_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(19);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(19);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 19
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            
            const bool condition_0 = 1 >= zero_tolerance || std::fabs(r_11*std::sin(th_0) - r_21*std::cos(th_0)) >= zero_tolerance || std::fabs(r_12*std::sin(th_0) - r_22*std::cos(th_0)) >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = std::sin(th_0);
                const double x1 = std::cos(th_0);
                // End of temp variables
                const double tmp_sol_value = std::atan2(-r_11*x0 + r_21*x1, -r_12*x0 + r_22*x1);
                solution_queue.get_solution(node_input_i_idx_in_queue)[6] = tmp_sol_value;
                add_input_index_to(20, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_19_solve_th_3th_5_soa_processor();
    // Finish code for explicit solution node 19
    
    // Code for non-branch dispatcher node 20
    // Actually, there is no code
    
    // Code for explicit solution node 21, solved variable is th_3
    auto ExplicitSolutionNode_node_21_solve_th_3_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(20);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(20);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 21
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            
            const bool condition_0 = true;
            if (condition_0)
            {
                // Temp variable for efficiency
                // End of temp variables
                const double tmp_sol_value = 0;
                solution_queue.get_solution(node_input_i_idx_in_queue)[5] = tmp_sol_value;
                add_input_index_to(22, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_21_solve_th_3_processor();
    // Finish code for explicit solution node 20
    
    // Code for non-branch dispatcher node 22
    // Actually, there is no code
    
    // Code for explicit solution node 23, solved variable is th_5
    auto ExplicitSolutionNode_node_23_solve_th_5_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(22);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(22);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 23
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_3 = this_solution[5];
            const double th_3th_5_soa = this_solution[6];
            
            const bool condition_0 = 1 >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                // End of temp variables
                const double tmp_sol_value = -th_3 + th_3th_5_soa;
                solution_queue.get_solution(node_input_i_idx_in_queue)[8] = tmp_sol_value;
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_23_solve_th_5_processor();
    // Finish code for explicit solution node 22
    
    // Code for explicit solution node 14, solved variable is negative_th_5_positive_th_3__soa
    auto ExplicitSolutionNode_node_14_solve_negative_th_5_positive_th_3__soa_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(14);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(14);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 14
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            
            const bool condition_0 = 1 >= zero_tolerance || std::fabs(r_11*std::sin(th_0) - r_21*std::cos(th_0)) >= zero_tolerance || std::fabs(r_12*std::sin(th_0) - r_22*std::cos(th_0)) >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = std::sin(th_0);
                const double x1 = std::cos(th_0);
                // End of temp variables
                const double tmp_sol_value = std::atan2(r_11*x0 - r_21*x1, -r_12*x0 + r_22*x1);
                solution_queue.get_solution(node_input_i_idx_in_queue)[0] = tmp_sol_value;
                add_input_index_to(15, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_14_solve_negative_th_5_positive_th_3__soa_processor();
    // Finish code for explicit solution node 14
    
    // Code for non-branch dispatcher node 15
    // Actually, there is no code
    
    // Code for explicit solution node 16, solved variable is th_3
    auto ExplicitSolutionNode_node_16_solve_th_3_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(15);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(15);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 16
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            
            const bool condition_0 = true;
            if (condition_0)
            {
                // Temp variable for efficiency
                // End of temp variables
                const double tmp_sol_value = 0;
                solution_queue.get_solution(node_input_i_idx_in_queue)[5] = tmp_sol_value;
                add_input_index_to(17, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_16_solve_th_3_processor();
    // Finish code for explicit solution node 15
    
    // Code for non-branch dispatcher node 17
    // Actually, there is no code
    
    // Code for explicit solution node 18, solved variable is th_5
    auto ExplicitSolutionNode_node_18_solve_th_5_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(17);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(17);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 18
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double negative_th_5_positive_th_3__soa = this_solution[0];
            const double th_3 = this_solution[5];
            
            const bool condition_0 = 1 >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                // End of temp variables
                const double tmp_sol_value = -negative_th_5_positive_th_3__soa + th_3;
                solution_queue.get_solution(node_input_i_idx_in_queue)[8] = tmp_sol_value;
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_18_solve_th_5_processor();
    // Finish code for explicit solution node 17
    
    // Code for explicit solution node 11, solved variable is th_3
    auto ExplicitSolutionNode_node_11_solve_th_3_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(11);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(11);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 11
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            const double th_1 = this_solution[2];
            const double th_2 = this_solution[4];
            const double th_4 = this_solution[7];
            
            const bool condition_0 = std::fabs(r_13*std::sin(th_0) - r_23*std::cos(th_0)) >= zero_tolerance || std::fabs(-r_13*(-std::sin(th_1)*std::sin(th_2) + std::cos(th_1)*std::cos(th_2))*std::cos(th_0) - r_23*(-std::sin(th_1)*std::sin(th_2) + std::cos(th_1)*std::cos(th_2))*std::sin(th_0) + r_33*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))) >= zero_tolerance || std::fabs(std::sin(th_4)) >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = 1.0/std::sin(th_4);
                const double x1 = std::sin(th_0);
                const double x2 = std::cos(th_0);
                const double x3 = std::sin(th_1);
                const double x4 = std::cos(th_2);
                const double x5 = std::sin(th_2);
                const double x6 = std::cos(th_1);
                const double x7 = -x3*x5 + x4*x6;
                // End of temp variables
                const double tmp_sol_value = std::atan2(x0*(-r_13*x1 + r_23*x2), x0*(-r_13*x2*x7 - r_23*x1*x7 + r_33*(x3*x4 + x5*x6)));
                solution_queue.get_solution(node_input_i_idx_in_queue)[5] = tmp_sol_value;
                add_input_index_to(12, node_input_i_idx_in_queue);
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_11_solve_th_3_processor();
    // Finish code for explicit solution node 11
    
    // Code for non-branch dispatcher node 12
    // Actually, there is no code
    
    // Code for explicit solution node 13, solved variable is th_5
    auto ExplicitSolutionNode_node_13_solve_th_5_processor = [&]() -> void
    {
        const auto& this_node_input_index = node_index_workspace.get_input_indices_for_node(12);
        const bool this_input_valid = node_index_workspace.is_input_indices_valid_for_node(12);
        if (!this_input_valid)
            return;
        
        // The solution of non-root node 13
        for(int i = 0; i < this_node_input_index.size(); i++)
        {
            int node_input_i_idx_in_queue = this_node_input_index[i];
            if (!solution_queue.is_solution_valid(node_input_i_idx_in_queue))
                continue;
            const auto& this_solution = solution_queue.get_solution(node_input_i_idx_in_queue);
            const double th_0 = this_solution[1];
            const double th_1 = this_solution[2];
            const double th_2 = this_solution[4];
            const double th_4 = this_solution[7];
            
            const bool condition_0 = std::fabs(r_11*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::cos(th_0) + r_21*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::sin(th_0) - r_31*(std::sin(th_1)*std::sin(th_2) - std::cos(th_1)*std::cos(th_2))) >= zero_tolerance || std::fabs(r_12*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::cos(th_0) + r_22*(std::sin(th_1)*std::cos(th_2) + std::sin(th_2)*std::cos(th_1))*std::sin(th_0) + r_32*(-std::sin(th_1)*std::sin(th_2) + std::cos(th_1)*std::cos(th_2))) >= zero_tolerance || std::fabs(std::sin(th_4)) >= zero_tolerance;
            if (condition_0)
            {
                // Temp variable for efficiency
                const double x0 = 1.0/std::sin(th_4);
                const double x1 = std::sin(th_1);
                const double x2 = std::sin(th_2);
                const double x3 = std::cos(th_1);
                const double x4 = std::cos(th_2);
                const double x5 = -x1*x2 + x3*x4;
                const double x6 = x1*x4 + x2*x3;
                const double x7 = x6*std::cos(th_0);
                const double x8 = x6*std::sin(th_0);
                // End of temp variables
                const double tmp_sol_value = std::atan2(x0*(-r_12*x7 - r_22*x8 - r_32*x5), x0*(r_11*x7 + r_21*x8 + r_31*x5));
                solution_queue.get_solution(node_input_i_idx_in_queue)[8] = tmp_sol_value;
            }
            else
            {
                solution_queue.solutions_validity[node_input_i_idx_in_queue] = false;
            }
        }
    };
    // Invoke the processor
    ExplicitSolutionNode_node_13_solve_th_5_processor();
    // Finish code for explicit solution node 12
    
    // Collect the output
    for(int i = 0; i < solution_queue.size(); i++)
    {
        if(!solution_queue.solutions_validity[i])
            continue;
        const auto& raw_ik_out_i = solution_queue.get_solution(i);
        std::array<double, robot_nq> new_ik_i;
        const double value_at_0 = raw_ik_out_i[1];  // th_0
        new_ik_i[0] = value_at_0;
        const double value_at_1 = raw_ik_out_i[2];  // th_1
        new_ik_i[1] = value_at_1;
        const double value_at_2 = raw_ik_out_i[4];  // th_2
        new_ik_i[2] = value_at_2;
        const double value_at_3 = raw_ik_out_i[5];  // th_3
        new_ik_i[3] = value_at_3;
        const double value_at_4 = raw_ik_out_i[7];  // th_4
        new_ik_i[4] = value_at_4;
        const double value_at_5 = raw_ik_out_i[8];  // th_5
        new_ik_i[5] = value_at_5;
        ik_output.push_back(new_ik_i);
    }
}

static void computeRawIK(const Eigen::Matrix4d& T_ee_raw, RawIKWorksace& workspace)
{
    workspace.raw_ik_out.clear();
    workspace.raw_ik_out.reserve(max_n_solutions);
    computeRawIK(T_ee_raw, workspace.solution_queue, workspace.node_index_workspace, workspace.raw_ik_out);
}

static void computeIKUnChecked(const Eigen::Matrix4d& T_ee, RawIKWorksace& workspace, std::vector<std::array<double, robot_nq>>& ik_output)
{
    const Eigen::Matrix4d& T_ee_raw = endEffectorTargetOriginalToRaw(T_ee);
    computeRawIK(T_ee_raw, workspace);
    const auto& raw_ik_out = workspace.raw_ik_out;
    ik_output.clear();
    for(int i = 0; i < raw_ik_out.size(); i++)
    {
        auto ik_out_i = raw_ik_out[i];
        ik_out_i[0] -= th_0_offset_original2raw;
        ik_out_i[1] -= th_1_offset_original2raw;
        ik_out_i[2] -= th_2_offset_original2raw;
        ik_out_i[3] -= th_3_offset_original2raw;
        ik_out_i[4] -= th_4_offset_original2raw;
        ik_out_i[5] -= th_5_offset_original2raw;
        ik_output.push_back(ik_out_i);
    }
}

static void wrapAngleToPi(std::vector<std::array<double, robot_nq>>& ik_output)
{
    for(int i = 0; i < ik_output.size(); i++)
    {
        // Wrap angular value to [-pi, pi]
        auto& solution_i = ik_output[i];
        // Revolute unknown th_0
        while(solution_i[0] > M_PI)
            solution_i[0] -= 2 * M_PI;
        while(solution_i[0] < - M_PI)
            solution_i[0] += 2 * M_PI;
        // Revolute unknown th_1
        while(solution_i[1] > M_PI)
            solution_i[1] -= 2 * M_PI;
        while(solution_i[1] < - M_PI)
            solution_i[1] += 2 * M_PI;
        // Revolute unknown th_2
        while(solution_i[2] > M_PI)
            solution_i[2] -= 2 * M_PI;
        while(solution_i[2] < - M_PI)
            solution_i[2] += 2 * M_PI;
        // Revolute unknown th_3
        while(solution_i[3] > M_PI)
            solution_i[3] -= 2 * M_PI;
        while(solution_i[3] < - M_PI)
            solution_i[3] += 2 * M_PI;
        // Revolute unknown th_4
        while(solution_i[4] > M_PI)
            solution_i[4] -= 2 * M_PI;
        while(solution_i[4] < - M_PI)
            solution_i[4] += 2 * M_PI;
        // Revolute unknown th_5
        while(solution_i[5] > M_PI)
            solution_i[5] -= 2 * M_PI;
        while(solution_i[5] < - M_PI)
            solution_i[5] += 2 * M_PI;
    }
}

static void computeIK(const Eigen::Matrix4d& T_ee, RawIKWorksace& workspace, std::vector<std::array<double, robot_nq>>& ik_output)
{
    const Eigen::Matrix4d& T_ee_raw = endEffectorTargetOriginalToRaw(T_ee);
    computeRawIK(T_ee_raw, workspace);
    const auto& raw_ik_out = workspace.raw_ik_out;
    ik_output.clear();
    for(int i = 0; i < raw_ik_out.size(); i++)
    {
        auto ik_out_i = raw_ik_out[i];
        ik_out_i[0] -= th_0_offset_original2raw;
        ik_out_i[1] -= th_1_offset_original2raw;
        ik_out_i[2] -= th_2_offset_original2raw;
        ik_out_i[3] -= th_3_offset_original2raw;
        ik_out_i[4] -= th_4_offset_original2raw;
        ik_out_i[5] -= th_5_offset_original2raw;
        const Eigen::Matrix4d ee_pose_i = computeFK(ik_out_i);
        double ee_pose_diff = (ee_pose_i - T_ee).squaredNorm();
        if (ee_pose_diff < pose_tolerance)
            ik_output.push_back(ik_out_i);
    }
    
    if (!ik_output.empty())
    {
        wrapAngleToPi(ik_output);
        removeDuplicate<robot_nq>(ik_output, zero_tolerance);
        return;
    }
    
    // Disturbing method for degenerate handling
    Eigen::Matrix4d T_ee_raw_disturbed = yaik_cpp::disturbTransform(T_ee_raw);
    Eigen::Matrix4d T_ee_disturbed = endEffectorTargetRawToOriginal(T_ee_raw_disturbed);
    computeRawIK(T_ee_raw_disturbed, workspace);
    const auto& raw_ik_out_disturb = workspace.raw_ik_out;
    for(int i = 0; i < raw_ik_out_disturb.size(); i++)
    {
        auto ik_out_i = raw_ik_out_disturb[i];
        ik_out_i[0] -= th_0_offset_original2raw;
        ik_out_i[1] -= th_1_offset_original2raw;
        ik_out_i[2] -= th_2_offset_original2raw;
        ik_out_i[3] -= th_3_offset_original2raw;
        ik_out_i[4] -= th_4_offset_original2raw;
        ik_out_i[5] -= th_5_offset_original2raw;
        Eigen::Matrix4d ee_pose_i = computeFK(ik_out_i);
        double ee_pose_diff = (ee_pose_i - T_ee_disturbed).squaredNorm();
        if (ee_pose_diff > pose_tolerance_degenerate)
            continue;
        
        // Try numerical refinement
        yaik_cpp::numericalRefinement<robot_nq>(computeFK, computeTwistJacobian, T_ee, ik_out_i);
        ee_pose_i = computeFK(ik_out_i);
        ee_pose_diff = (ee_pose_i - T_ee).squaredNorm();
        if (ee_pose_diff < pose_tolerance_degenerate)
            ik_output.push_back(ik_out_i);
    }
    
    wrapAngleToPi(ik_output);
    removeDuplicate<robot_nq>(ik_output, zero_tolerance);
}

static std::vector<std::array<double, robot_nq>> computeIK(const Eigen::Matrix4d& T_ee)
{
    std::vector<std::array<double, robot_nq>> ik_output;
    RawIKWorksace raw_ik_workspace;
    computeIK(T_ee, raw_ik_workspace, ik_output);
    return ik_output;
}

}; // struct spherical_wrist_six_axis_ik

// Code below for debug
void test_ik_solve_spherical_wrist_six_axis()
{
    std::array<double, spherical_wrist_six_axis_ik::robot_nq> theta;
    std::random_device rd;
    std::uniform_real_distribution<double> distribution;
    for(auto i = 0; i < theta.size(); i++)
        theta[i] = distribution(rd);
    const Eigen::Matrix4d ee_pose = spherical_wrist_six_axis_ik::computeFK(theta);
    auto ik_output = spherical_wrist_six_axis_ik::computeIK(ee_pose);
    for(int i = 0; i < ik_output.size(); i++)
    {
        Eigen::Matrix4d ee_pose_i = spherical_wrist_six_axis_ik::computeFK(ik_output[i]);
        double ee_pose_diff = (ee_pose_i - ee_pose).norm();
        std::cout << "For solution " << i << " Pose different with ground-truth " << ee_pose_diff << std::endl;
    }
}

// int main()
// {
//     test_ik_solve_spherical_wrist_six_axis();
// }

class PiperIKNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Publisher ik_status_pub_;
    ros::Subscriber gripper_cmd_sub_;
    ros::Subscriber real_joint_position_sub_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber marker_feedback_sub_;
    ros::Timer publish_timer_;
    
    double gripper_cmd_;
    sensor_msgs::JointState current_joint_state_;
    sensor_msgs::JointState real_joint_state_;
    std::vector<double> previous_solution_;
    double publish_rate_;

public:
    PiperIKNode(const std::string& node_name) : nh_(node_name), gripper_cmd_(0.0), publish_rate_(50.0) {
        initialize();
    }

    void initialize() {
        std::string joint_state_topic;
        std::string target_pose_topic;
        std::string gripper_cmd_topic;
        std::string real_joint_position_topic;
        std::string marker_feedback_topic;
        
        nh_.param<std::string>("joint_state_topic", joint_state_topic, "/joint_states");
        nh_.param<std::string>("target_pose_topic", target_pose_topic, "/target_pose");
        nh_.param<std::string>("gripper_cmd_topic", gripper_cmd_topic, "/gripper_cmd_topic");
        nh_.param<std::string>("real_joint_position_topic", real_joint_position_topic, "/joint_states_single");
        nh_.param<std::string>("marker_feedback_topic", marker_feedback_topic, "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic");
        nh_.param<double>("publish_rate", publish_rate_, 50);

        // Initialize publishers and subscribers
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 10);
        ik_status_pub_ = nh_.advertise<std_msgs::Bool>("/ik_status", 10);
        gripper_cmd_sub_ = nh_.subscribe(gripper_cmd_topic, 1, &PiperIKNode::gripperCmdCallback, this);
        real_joint_position_sub_ = nh_.subscribe(real_joint_position_topic, 10, &PiperIKNode::jointPositionCallback, this);
        target_pose_sub_ = nh_.subscribe(target_pose_topic, 10, &PiperIKNode::targetPoseCallback, this);
        marker_feedback_sub_ = nh_.subscribe(marker_feedback_topic, 10, &PiperIKNode::markerFeedbackCallback, this);

        // Initialize joint state
        current_joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        current_joint_state_.position.resize(7, 0.0);

        // Initialize timer for continuous publishing
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &PiperIKNode::publishJointState, this);
    }

    void gripperCmdCallback(const std_msgs::Float64::ConstPtr& msg) {
        gripper_cmd_ = msg->data;
    }

    void markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
        if (msg->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = msg->pose;
            pose_stamped.header = msg->header;
            targetPoseCallback(boost::make_shared<geometry_msgs::PoseStamped>(pose_stamped));
        }
    }

    void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        try {
            // Convert PoseStamped to Eigen::Matrix4d
            Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
            
            // Position
            target_pose(0, 3) = msg->pose.position.x;
            target_pose(1, 3) = msg->pose.position.y;
            target_pose(2, 3) = msg->pose.position.z;
            
            // Orientation (quaternion to rotation matrix)
            Eigen::Quaterniond q(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            );
            target_pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
            
            auto ik_output = spherical_wrist_six_axis_ik::computeIK(target_pose);

            if (!ik_output.empty()) {
                // Update current joint state (first 6 joints)
                for (int i = 0; i < 6; ++i) {
                    current_joint_state_.position[i] = ik_output[0][i]; // Use first solution
                }
                current_joint_state_.header.stamp = ros::Time::now();

                // Log results
                ROS_INFO("IK Solution Found:");
                for (size_t i = 0; i < ik_output[0].size(); ++i) {
                    ROS_INFO("  Joint %zu: %.4f rad (%.2f deg)", 
                           i+1, ik_output[0][i], ik_output[0][i] * 180.0 / M_PI);
                }

                // Publish TF for visualization
                std::vector<double> joint_values(ik_output[0].begin(), ik_output[0].end());
                publishSolutionTF(joint_values, target_pose);

                // Publish IK status
                std_msgs::Bool ik_status;
                ik_status.data = true;
                ik_status_pub_.publish(ik_status);
            } else {
                ROS_WARN("No IK solution found");
                std_msgs::Bool ik_status;
                ik_status.data = false;
                ik_status_pub_.publish(ik_status);
            }

        } catch (const std::exception& e) {
            ROS_ERROR("IK failed: %s", e.what());
            std_msgs::Bool ik_status;
            ik_status.data = false;
            ik_status_pub_.publish(ik_status);
        }
    }

    void publishSolutionTF(const std::vector<double>& joint_values, const Eigen::Matrix4d& target_pose) {
        static tf::TransformBroadcaster tf_broadcaster;
        
        // Publish target pose
        tf::Transform target_tf;
        target_tf.setOrigin(tf::Vector3(
            target_pose(0, 3),
            target_pose(1, 3),
            target_pose(2, 3)
        ));
        
        Eigen::Quaterniond q(target_pose.block<3, 3>(0, 0));
        tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
        target_tf.setRotation(tf_q);
        
        tf_broadcaster.sendTransform(tf::StampedTransform(
            target_tf, ros::Time::now(), "base_link", "ik_target"));

        // Compute FK for current solution
        Eigen::Matrix4d current_pose = spherical_wrist_six_axis_ik::computeFK(
            {joint_values[0], joint_values[1], joint_values[2], 
             joint_values[3], joint_values[4], joint_values[5]});
        
        tf::Transform current_tf;
        current_tf.setOrigin(tf::Vector3(
            current_pose(0, 3),
            current_pose(1, 3),
            current_pose(2, 3)
        ));
        
        Eigen::Quaterniond q_current(current_pose.block<3, 3>(0, 0));
        tf::Quaternion tf_q_current(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        current_tf.setRotation(tf_q_current);
        
        tf_broadcaster.sendTransform(tf::StampedTransform(
            current_tf, ros::Time::now(), "base_link", "ik_solution"));
    }

    void jointPositionCallback(const sensor_msgs::JointStateConstPtr& msg) {
        try {
            real_joint_state_ = *msg;
            
            // Update previous solution with real joint positions (first 6 joints)
            if (msg->position.size() >= 6) {
                std::vector<double> new_previous;
                for (int i = 0; i < 6; ++i) {
                    new_previous.push_back(msg->position[i]);
                }
                previous_solution_ = new_previous;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error in real joint position callback: %s", e.what());
        }
    }

    void publishJointState(const ros::TimerEvent&) {
        current_joint_state_.header.stamp = ros::Time::now();
        if(gripper_cmd_ > 0){
            current_joint_state_.position[6] = gripper_cmd_;
        } else {
            current_joint_state_.position[6] = 0.0;
        }
        
        joint_state_pub_.publish(current_joint_state_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "piper_ik_node_use_yaik");
    
    PiperIKNode ik_node("piper_ik_node_use_yaik");
    
    ros::spin();
    
    return 0;
}