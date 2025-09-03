#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

bool image_received = false;
bool depth_received = false;
cv::Mat K;
cv::Mat image;
cv::Mat depth;
cv::Mat bgr;
cv::Mat hsv;
cv::Mat dst;
cv::Point cpoint;

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    int r = 10;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
        if (!depth.empty()) {
            depth_received = true;
            depth = cv_ptr->image.clone();

                // Create circular mask
                cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
                cv::circle(mask, cpoint, r, cv::Scalar(255), -1);
                
                // Calculate average depth in the region
                cv::Mat depth_roi;
                depth.copyTo(depth_roi, mask);
                
                // Convert to float for calculation (depth is typically uint16)
                cv::Mat depth_float;
                depth_roi.convertTo(depth_float, CV_32F);
                
                // Mask out zero values (invalid depth)
                depth_float.setTo(std::numeric_limits<float>::quiet_NaN(), depth_roi == 0);
                
                // Calculate mean depth, ignoring NaN values
                cv::Scalar mean_depth = cv::mean(depth_float, mask);
                
                if (!std::isnan(mean_depth[0])) {
                    ROS_INFO("Average depth at (%d,%d): %f mm", 
                            cpoint.x, cpoint.y, mean_depth[0]);
                    
                    // Optional: Visualize the region
                    cv::Mat depth_vis;
                    cv::normalize(depth, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
                    cv::cvtColor(depth_vis, depth_vis, cv::COLOR_GRAY2BGR);
                    cv::circle(depth_vis, cpoint, r, cv::Scalar(0, 0, 255), 2);
                    cv::imshow("Depth ROI", depth_vis);
                    cv::waitKey(1);
                }
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in depth callback: %s", e.what());
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        image = cv_ptr->image.clone();
        if (!image.empty()) {
            image_received = true;
            bgr = image.clone();
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in image callback: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "handpose_detection_node");
    ros::NodeHandle nh;
    std::string image_topic, depth_topic;
    nh.param<std::string>("image_topic_name", image_topic, "/camera/color/image_raw");
    nh.param<std::string>("depth_topic_name", depth_topic, "/camera/depth/image_raw");
    
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber depth_sub = nh.subscribe(depth_topic, 1, depthCallback);
    
    // Create windows once
    cv::namedWindow("origin_image", cv::WINDOW_GUI_EXPANDED);
    cv::namedWindow("hsv_image", cv::WINDOW_GUI_EXPANDED);
     
    ros::Rate r(30);

    while(ros::ok()) {
        if (image_received && !image.empty()) {
            cv::imshow("origin_image", image);
        }
        
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC key
            break;
        }
        
        ros::spinOnce();
        r.sleep();
    }
    
    cv::destroyAllWindows();
    return 0;
}