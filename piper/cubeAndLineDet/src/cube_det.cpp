#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

bool camera_info_received = false;
bool image_received = false;
bool depth_received = false;
cv::Mat K;
cv::Mat image;
cv::Mat depth;
cv::Mat bgr;
cv::Mat hsv;
cv::Mat dst;
cv::Point cpoint;

int hmin = 0, hmin_Max = 360;
int hmax = 180, hmax_Max = 180;
int smin = 0, smin_Max = 255;
int smax = 255, smax_Max = 255;
int vmin = 106, vmin_Max = 255;
int vmax = 255, vmax_Max = 255;

ros::Publisher marker_pub;

void callBack(int, void*);
void onMouse(int event, int x, int y, int flags, void* userdata);
void publishCoordinateMarker(float x, float y, float z);

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN && !hsv.empty()) {
        cv::Vec3b hsv_pixel = hsv.at<cv::Vec3b>(y, x);
        int h = hsv_pixel[0], s = hsv_pixel[1], v = hsv_pixel[2];
        ROS_INFO("Clicked at (%d, %d) - HSV: (%d, %d, %d)", x, y, h, s, v);
        
        int range = 20; // Adjustable range
        hmin = std::max(0, h - range); hmax = std::min(hmax_Max, h + range);
        smin = std::max(0, s - range); smax = std::min(smax_Max, s + range);
        vmin = std::max(0, v - range); vmax = std::min(vmax_Max, v + range);
        
        cv::setTrackbarPos("hmin", "hsv_image", hmin);
        cv::setTrackbarPos("hmax", "hsv_image", hmax);
        cv::setTrackbarPos("smin", "hsv_image", smin);
        cv::setTrackbarPos("smax", "hsv_image", smax);
        cv::setTrackbarPos("vmin", "hsv_image", vmin);
        cv::setTrackbarPos("vmax", "hsv_image", vmax);
        callBack(0, 0);
    }
}

void callBack(int, void*) {
    if (hsv.empty()) return;
    
    dst = cv::Mat::zeros(image.size(), image.type());
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), mask);
    
    for (int r = 0; r < bgr.rows; r++) {
        for (int c = 0; c < bgr.cols; c++) {
            if (mask.at<uchar>(r, c) == 255) {
                dst.at<cv::Vec3b>(r, c) = bgr.at<cv::Vec3b>(r, c);
            }
        }
    }
    
    cv::Mat gray, binary;
    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 127, 255, cv::THRESH_OTSU);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    
    int max_matches = 0;
    cv::Rect best_rect;
    cv::Point center_point;

    for (size_t i = 0; i < contours.size(); i++) {
        cv::Rect rect = cv::boundingRect(contours[i]);
        cv::Mat rect_mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::drawContours(rect_mask, contours, i, cv::Scalar(255), cv::FILLED);
        
        cv::Mat rect_hsv;
        hsv.copyTo(rect_hsv, rect_mask);
        cv::Mat rect_color_mask;
        cv::inRange(rect_hsv, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), rect_color_mask);
        
        int matches = cv::countNonZero(rect_color_mask(rect));
        if (matches > max_matches) {
            max_matches = matches;
            best_rect = rect;
            center_point = cv::Point(rect.x + rect.width/2, rect.y + rect.height/2);
        }
        cv::rectangle(dst, rect, cv::Scalar(0, 0, 255), 2);
    }

    if (max_matches > 1000) {
        cv::rectangle(dst, best_rect, cv::Scalar(0, 255, 0), 3);
        cv::circle(dst, center_point, 5, cv::Scalar(255, 0, 0), -1);
        ROS_INFO("Best rectangle center at (%d, %d)", center_point.x, center_point.y);
        cpoint = center_point;
    }

    if (!dst.empty()) cv::imshow("hsv_image", dst);
}

// Publish 3D coordinates to RViz
void publishCoordinateMarker(float x, float y, float z) {
    visualization_msgs::MarkerArray marker_array;

    // Red sphere at the detected point
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "camera_color_optical_frame";
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = "cube_coordinates";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = x;
    point_marker.pose.position.y = y;
    point_marker.pose.position.z = z;
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.02;
    point_marker.scale.y = 0.02;
    point_marker.scale.z = 0.02;
    point_marker.color.r = 1.0;
    point_marker.color.a = 1.0;
    point_marker.lifetime = ros::Duration(0.1);

    // Text label showing coordinates
    visualization_msgs::Marker text_marker;
    text_marker.header = point_marker.header;
    text_marker.ns = "cube_coordinates";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = x + 0.05;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = z + 0.05;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.02;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "X: " + std::to_string(x) + "\nY: " + std::to_string(y) + "\nZ: " + std::to_string(z);
    text_marker.lifetime = ros::Duration(0.1);

    marker_array.markers.push_back(point_marker);
    marker_array.markers.push_back(text_marker);
    marker_pub.publish(marker_array);
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
    int r = 10; // ROI radius
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
        depth = cv_ptr->image.clone();
        if (!depth.empty()) {
            depth_received = true;
            cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
            cv::circle(mask, cpoint, r, cv::Scalar(255), -1);
            
            cv::Mat depth_roi;
            depth.copyTo(depth_roi, mask);
            
            cv::Mat depth_float;
            depth_roi.convertTo(depth_float, CV_32F);
            depth_float.setTo(std::numeric_limits<float>::quiet_NaN(), depth_roi == 0);
            
            cv::Scalar mean_depth = cv::mean(depth_float, mask);
            
            if (!std::isnan(mean_depth[0])) {
                ROS_INFO("Depth at (%d,%d): %f mm", cpoint.x, cpoint.y, mean_depth[0]);
                
                if (camera_info_received) {
                    float fx = K.at<float>(0, 0);
                    float fy = K.at<float>(1, 1);
                    float cx = K.at<float>(0, 2);
                    float cy = K.at<float>(1, 2);
                    
                    float depth_m = mean_depth[0] / 1000.0; // mm -> m
                    float X = (cpoint.x - cx) * depth_m / fx;
                    float Y = (cpoint.y - cy) * depth_m / fy;
                    float Z = depth_m;
                    
                    ROS_INFO("3D Position: X=%.3fm, Y=%.3fm, Z=%.3fm", X, Y, Z);
                    publishCoordinateMarker(X, Y, Z);
                }
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Depth callback error: %s", e.what());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        image = cv_ptr->image.clone();
        if (!image.empty()) {
            image_received = true;
            bgr = image.clone();
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Image callback error: %s", e.what());
    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (!camera_info_received) {
        K = cv::Mat(3, 3, CV_32F);
        for (int i = 0; i < 9; ++i) K.at<float>(i/3, i%3) = msg->K[i];
        camera_info_received = true;
        ROS_INFO_STREAM("Camera Intrinsics:\n" << K);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_detection_node");
    ros::NodeHandle nh;
    
    std::string image_topic, camera_info_topic, depth_topic;
    nh.param<std::string>("image_topic_name", image_topic, "/camera/color/image_raw");
    nh.param<std::string>("camera_info_topic_name", camera_info_topic, "/camera/color/camera_info");
    nh.param<std::string>("depth_topic_name", depth_topic, "/camera/depth/image_raw");
    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cube_coordinates", 10);
    
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber info_sub = nh.subscribe(camera_info_topic, 1, cameraInfoCallback);
    ros::Subscriber depth_sub = nh.subscribe(depth_topic, 1, depthCallback);
    
    cv::namedWindow("origin_image", cv::WINDOW_GUI_EXPANDED);
    cv::namedWindow("hsv_image", cv::WINDOW_GUI_EXPANDED);
    cv::setMouseCallback("origin_image", onMouse, NULL);
    
    cv::createTrackbar("hmin", "hsv_image", &hmin, hmin_Max, callBack);
    cv::createTrackbar("hmax", "hsv_image", &hmax, hmax_Max, callBack);
    cv::createTrackbar("smin", "hsv_image", &smin, smin_Max, callBack);
    cv::createTrackbar("smax", "hsv_image", &smax, smax_Max, callBack);
    cv::createTrackbar("vmin", "hsv_image", &vmin, vmin_Max, callBack);
    cv::createTrackbar("vmax", "hsv_image", &vmax, vmax_Max, callBack);

    ros::Rate rate(30);
    while (ros::ok()) {
        if (image_received && !image.empty()) {
            cv::imshow("origin_image", image);
            callBack(0, 0);
        }
        if (cv::waitKey(1) == 27) break; // ESC to exit
        ros::spinOnce();
        rate.sleep();
    }
    
    cv::destroyAllWindows();
    return 0;
}