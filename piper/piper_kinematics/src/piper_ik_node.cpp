#include "piper_jacobian_ik.hpp"
#include "piper_forward_kinematics.hpp"
#include "piper_analytical_ik.hpp"
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

class PiperIKNode {
public:
    PiperIKNode() {
        // Initialize parameters
        ros::NodeHandle nh("~");
        nh.param<std::string>("dh_type", dh_type_, "modified");
        nh.param<std::string>("joint_state_topic", joint_state_topic_, "/joint_states");
        nh.param<std::string>("target_pose_topic", target_pose_topic_, "/target_pose");
        nh.param<std::string>("gripper_cmd_topic", gripper_cmd_topic_, "/gripper_cmd_topic");
        nh.param<std::string>("sub_real_joint_position_topic", sub_real_joint_position_topic_, "/joint_states_single");
        nh.param<std::string>("marker_feedback_topic", marker_feedback_topic_, "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic");
        nh.param<int>("max_iterations", max_iterations_, 50);
        nh.param<double>("position_tolerance", position_tolerance_, 1e-3);
        nh.param<double>("orientation_tolerance", orientation_tolerance_, 1e-2);
        nh.param<double>("damping_factor", damping_factor_, 0.1);
        nh.param<bool>("use_analytical_jacobian", use_analytical_jacobian_, false);
        nh.param<double>("publish_rate", publish_rate_, 30.0);
        nh.param<double>("filter_alpha", filter_alpha_, 0.7);
        nh.param<int>("history_size", history_size_, 5);
        nh.param<bool>("use_analytic_ik", use_analytic_ik_, false);

        // Initialize IK solver
        PiperForwardKinematics::DHType dh_type = (dh_type_ == "standard") ? 
            PiperForwardKinematics::STANDARD : PiperForwardKinematics::MODIFIED;
        
        ik_numerical_solver_ = std::make_unique<PiperJacobianIK>(dh_type);
        ik_analytical_solver_ = std::make_unique<PiperAnalyticalIK>(dh_type);
        
        // Set solver parameters
        ik_numerical_solver_->setMaxIterations(max_iterations_);
        ik_numerical_solver_->setPositionTolerance(position_tolerance_);
        ik_numerical_solver_->setOrientationTolerance(orientation_tolerance_);
        ik_numerical_solver_->setDampingFactor(damping_factor_);
        ik_numerical_solver_->useAnalyticalJacobian(use_analytical_jacobian_);


        // Set joint limits
        joint_limits_ = {
            {-2.618, 2.618},    // Joint 1
            {0, M_PI},          // Joint 2
            {-M_PI, 0},         // Joint 3
            {-2.967, 2.967},    // Joint 4
            {-1.2, 1.2},       // Joint 5
            {-1.22, 1.22}      // Joint 6
        };
        ik_numerical_solver_->setJointLimits(joint_limits_);
        ik_analytical_solver_->setJointLimits(joint_limits_);

        // Initialize publishers and subscribers
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
            joint_state_topic_, 10);
        
        lowend_pub_ = nh_.advertise<piper_msgs::PosCmd>(
            "/pos_cmd", 10);

        ik_status_pub_ = nh_.advertise<std_msgs::Bool>(
            "/ik_status", 10);
        
        real_joint_position_sub_ = nh_.subscribe(
            sub_real_joint_position_topic_, 10,
            &PiperIKNode::realJointPositionCallback, this);
        
        target_pose_sub_ = nh_.subscribe(
            target_pose_topic_, 10,
            &PiperIKNode::targetPoseCallback, this);
            
        marker_feedback_sub_ = nh_.subscribe(
            marker_feedback_topic_, 10,
            &PiperIKNode::markerFeedbackCallback, this);

        // Initialize timer for continuous publishing
        publish_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &PiperIKNode::publishJointState, this);

        gripper_cmd_sub_ = nh_.subscribe(
            gripper_cmd_topic_, 1,
            &PiperIKNode::gripperCmdCallback, this);

        // Initial joint state (home position)
        current_joint_state_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        real_joint_state_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        real_joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        
        // Initialize previous solution
        previous_solution_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        ROS_INFO("Piper IK Node initialized and ready");
    }

private:
    // Joint filter class for smoothing
    class JointFilter {
    public:
        JointFilter(size_t num_joints, double alpha = 0.5) 
            : alpha_(alpha), prev_values_(num_joints, 0.0) {}
        
        std::vector<double> filter(const std::vector<double>& new_values) {
            if (new_values.size() != prev_values_.size()) {
                throw std::runtime_error("Joint count mismatch");
            }
            
            std::vector<double> filtered;
            for (size_t i = 0; i < new_values.size(); ++i) {
                filtered.push_back(alpha_ * new_values[i] + (1 - alpha_) * prev_values_[i]);
            }
            
            prev_values_ = filtered;
            return filtered;
        }
        
        void reset(const std::vector<double>& initial_values) {
            prev_values_ = initial_values;
        }
        
    private:
        double alpha_;
        std::vector<double> prev_values_;
    };

    void realJointPositionCallback(const sensor_msgs::JointStateConstPtr& msg) {
        try {
            real_joint_state_.position = msg->position;
            real_joint_state_.header.stamp = ros::Time::now();
            
            // Update previous solution with real joint positions (first 6 joints)
            if (msg->position.size() >= 6) {
                std::vector<double> new_previous;
                for (int i = 0; i < 6; ++i) {
                    new_previous.push_back(msg->position[i]);
                }
                previous_solution_ = ensureAngleContinuity(previous_solution_, new_previous);
                
                // Reset filter with current values
                if (joint_filter_) {
                    joint_filter_->reset(new_previous);
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error in real joint position callback: %s", e.what());
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

            // for piper_ros_pinocciao ik
            tf::Quaternion local_q;
            double local_roll,local_pitch,local_yaw;
            target_pose_lowend.x = msg->pose.position.x;
            target_pose_lowend.y = msg->pose.position.y;
            target_pose_lowend.z = msg->pose.position.z;
            // std::cout << "target_pose_lowend.x = " << target_pose_lowend.x << std::endl;
            // std::cout << "target_pose_lowend.y = " << target_pose_lowend.y << std::endl;
            // std::cout << "target_pose_lowend.z = " << target_pose_lowend.z << std::endl;

            tf::quaternionMsgToTF(msg->pose.orientation,local_q); 
            tf::Matrix3x3(local_q).getRPY(local_roll,local_pitch,local_yaw); 
            local_roll = std::clamp(local_roll, -3.1, 3.1);
            local_pitch = std::clamp(local_pitch, -1.5, 1.5);
            local_yaw = std::clamp(local_yaw, -3.1, 3.1);

            target_pose_lowend.roll = local_roll;
            target_pose_lowend.pitch = local_pitch;
            target_pose_lowend.yaw = local_yaw;


            // std::cout << "target_pose_lowend.roll = " << local_roll << std::endl;
            // std::cout << "target_pose_lowend.pitch = " << local_pitch << std::endl;
            // std::cout << "target_pose_lowend.yaw = " << local_yaw << std::endl;

            // Use previous solution as initial guess for continuity
            std::vector<double> initial_guess = previous_solution_;
            
            // Compute IK with continuity enforcement
            Eigen::VectorXd final_error;
            std::vector<double> solution = computeIKWithContinuity(
                initial_guess, target_pose, true, &final_error);

            // Apply filtering for smoothness
            std::vector<double> smoothed_solution;
            if (joint_filter_) {
                smoothed_solution = joint_filter_->filter(solution);
            } else {
                smoothed_solution = solution;
            }
            
            // Update current joint state (first 6 joints)
            for (int i = 0; i < 6; ++i) {
                current_joint_state_.position[i] = smoothed_solution[i];
            }
            current_joint_state_.header.stamp = ros::Time::now();

            // Save current solution for next iteration
            previous_solution_ = smoothed_solution;

            // Log results
            ROS_INFO("IK Solution Found:");
            for (size_t i = 0; i < smoothed_solution.size(); ++i) {
                ROS_INFO("  Joint %zu: %.4f rad (%.2f deg)", 
                       i+1, smoothed_solution[i], smoothed_solution[i] * 180.0 / M_PI);
            }
            ROS_INFO("Final error - Position: %.6f mm, Orientation: %.6f rad",
                   final_error.head<3>().norm()*1000, final_error.tail<3>().norm());

            // Publish TF for visualization
            publishSolutionTF(smoothed_solution, target_pose);

        } catch (const std::exception& e) {
            ROS_ERROR("IK failed: %s", e.what());
        }
    }

    // IK computation with continuity enforcement
    std::vector<double> computeIKWithContinuity(const std::vector<double>& initial_guess,
                                               const Eigen::Matrix4d& target_pose,
                                               bool verbose = false,
                                               Eigen::VectorXd* final_error = nullptr) {
        try{// Compute inverse kinematics
        std::vector<double> solution = ik_numerical_solver_->computeIK(
            initial_guess, target_pose, verbose, final_error);

        if_ik_success_.data = true;
        ik_status_pub_.publish(if_ik_success_);
        
        // Ensure angle continuity (avoid ±2π jumps)
        return ensureAngleContinuity(initial_guess, solution);
        }
        catch (const std::exception& e) {
            // 设置失败状态
            if_ik_success_.data = false;
            ik_status_pub_.publish(if_ik_success_);
            throw; // 重新抛出异常
        }
    }

    // Ensure angle continuity between consecutive solutions
    std::vector<double> ensureAngleContinuity(const std::vector<double>& prev_angles, 
                                             const std::vector<double>& new_angles) {
        if (prev_angles.size() != new_angles.size()) {
            return new_angles;
        }
        
        std::vector<double> continuous_angles = new_angles;
        
        for (size_t i = 0; i < prev_angles.size(); ++i) {
            double diff = new_angles[i] - prev_angles[i];
            
            // If angle change exceeds π, adjust by ±2π to minimize change
            if (diff > M_PI) {
                continuous_angles[i] -= 2 * M_PI;
            } else if (diff < -M_PI) {
                continuous_angles[i] += 2 * M_PI;
            }
            
            // Ensure still within joint limits
            continuous_angles[i] = std::clamp(continuous_angles[i], 
                                            joint_limits_[i].first, 
                                            joint_limits_[i].second);
        }
        
        return continuous_angles;
    }

    void markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
        if (msg->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = msg->pose;
            pose_stamped.header = msg->header;
            targetPoseCallback(boost::make_shared<geometry_msgs::PoseStamped>(pose_stamped));
        }
    }

    void gripperCmdCallback(const std_msgs::Float64::ConstPtr& msg) {
        gripper_cmd_ = msg->data;
    }

    void publishJointState(const ros::TimerEvent&) {
        current_joint_state_.header.stamp = ros::Time::now();
        if(gripper_cmd_>0){
            current_joint_state_.position[6] = gripper_cmd_;
        }
        else{
            current_joint_state_.position[6] = 0.0;
        }
        current_joint_state_.name[6] = "joint7";
        
        //joint_state_pub_.publish(current_joint_state_);
        if(use_analytic_ik_){
            target_pose_lowend.gripper = gripper_cmd_;
            lowend_pub_.publish(target_pose_lowend);
        }
        else{
            joint_state_pub_.publish(current_joint_state_);
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

        // Publish current solution pose
        PiperForwardKinematics::DHType dh_type = (dh_type_ == "standard") ? 
            PiperForwardKinematics::STANDARD : PiperForwardKinematics::MODIFIED;
        PiperForwardKinematics fk(dh_type);

        Eigen::Matrix4d current_pose = fk.computeFK(joint_values);
        
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

    std::unique_ptr<PiperJacobianIK> ik_numerical_solver_;
    std::unique_ptr<PiperAnalyticalIK> ik_analytical_solver_;
    std::unique_ptr<JointFilter> joint_filter_;
    ros::Publisher joint_state_pub_;
    ros::Publisher lowend_pub_;
    ros::Publisher ik_status_pub_;
    ros::Subscriber real_joint_position_sub_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber marker_feedback_sub_;
    ros::Subscriber gripper_cmd_sub_;
    ros::Timer publish_timer_;
    sensor_msgs::JointState current_joint_state_;
    sensor_msgs::JointState real_joint_state_;
    std::vector<double> previous_solution_; // Store previous solution for continuity
    piper_msgs::PosCmd target_pose_lowend;
    
    ros::NodeHandle nh_;
    std::string dh_type_;
    std::string joint_state_topic_;
    std::string target_pose_topic_;
    std::string gripper_cmd_topic_;
    std::string sub_real_joint_position_topic_;
    std::string marker_feedback_topic_;
    double gripper_cmd_;
    int max_iterations_;
    double position_tolerance_;
    double orientation_tolerance_;
    double damping_factor_;
    bool use_analytical_jacobian_;
    bool use_analytic_ik_;
    double publish_rate_;
    double filter_alpha_;
    int history_size_;
    std_msgs::Bool if_ik_success_;
    std::vector<std::pair<double, double>> joint_limits_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "piper_ik_node");
    PiperIKNode node;
    ros::spin();
    return 0;
}