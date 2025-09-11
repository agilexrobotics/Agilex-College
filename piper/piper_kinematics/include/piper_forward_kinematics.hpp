#ifndef PIPER_FORWARD_KINEMATICS_HPP
#define PIPER_FORWARD_KINEMATICS_HPP

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

class PiperForwardKinematics {
public:
    enum DHType { STANDARD, MODIFIED };
    
    PiperForwardKinematics(DHType type = STANDARD) : dh_type_(type) {
        setupDHParameters();
    }

    Eigen::Matrix4d computeFK(const std::vector<double>& joint_values) {
        if (joint_values.size() < 6) {
            throw std::runtime_error("Piper arm requires at least 6 joint values for FK");
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        for (size_t i = 0; i < 6; ++i) {
            double theta = joint_values[i] + dh_params_[i][3];  // θ = joint_value + θ_offset
            double d = dh_params_[i][2];                       // d = d_fixed (如果是旋转关节)
            
            T *= computeTransform(
                dh_params_[i][0],  // alpha
                dh_params_[i][1],  // a
                d,                 // d
                theta              // theta
            );
        }
        // std::cout << "\n目标位姿矩阵: " << std::endl;
        // std::cout << T << std::endl;
        return T;
    }

    // 新增方法：获取DH参数
    const std::vector<std::vector<double>>& getDHParams() const {
        return dh_params_;
    }
    
    // 添加获取DH类型的方法
    DHType getDHType() const { return dh_type_; }

    // 新增方法：计算单个变换
    Eigen::Matrix4d computeSingleTransform(size_t joint_index, double joint_value) const {
        if (joint_index >= dh_params_.size()) {
            throw std::runtime_error("Joint index out of range");
        }

        double theta = joint_value + dh_params_[joint_index][3];
        double d = dh_params_[joint_index][2];
        
        return computeTransform(
            dh_params_[joint_index][0],  // alpha
            dh_params_[joint_index][1],  // a
            d,                          // d
            theta                       // theta
        );
    }

private:
    DHType dh_type_;
    std::vector<std::vector<double>> dh_params_;

    void setupDHParameters() {
        if (dh_type_ == STANDARD) {
            // 标准DH参数 [alpha, a, d, theta_offset]
            dh_params_ = {
                {-M_PI/2,   0,          0.123,      0},                     // Joint 1
                {0,         0.28503,    0,          -172.22/180*M_PI},      // Joint 2 
                {M_PI/2,    -0.021984,  0,          -102.78/180*M_PI},      // Joint 3
                {-M_PI/2,   0,          0.25075,    0},                     // Joint 4
                {M_PI/2,    0,          0,          0},                     // Joint 5
                {0,         0,          0.211,      0}                      // Joint 6
            };
        } else {
            // 改进DH参数 [alpha, a, d, theta_offset]
            dh_params_ = {
                {0,         0,          0.123,      0},                     // Joint 1
                {-M_PI/2,   0,          0,          -172.22/180*M_PI},      // Joint 2 
                {0,         0.28503,    0,          -102.78/180*M_PI},      // Joint 3
                {M_PI/2,    -0.021984,  0.25075,    0},                     // Joint 4
                {-M_PI/2,   0,          0,          0},                     // Joint 5
                {M_PI/2,    0,          0.211,      0}                      // Joint 6
            };
        }
    }

    Eigen::Matrix4d computeTransform(double alpha, double a, double d, double theta) const {
        Eigen::Matrix4d T;
        if (dh_type_ == STANDARD) {
            // 标准DH变换矩阵
            T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                 0,           sin(alpha),             cos(alpha),            d,
                 0,           0,                      0,                     1;
        } else {
            // 改进DH变换矩阵
            T << cos(theta),            -sin(theta),            0,             a,
                 sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
                 sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d,
                 0,                     0,                      0,             1;
        }
        return T;
    }
};

#endif