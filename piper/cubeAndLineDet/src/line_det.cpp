#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <line_segment.hpp>
// #include <line_fill.hpp>
#include <test_fitting.hpp>

CurveSegment::Parameters params;
// CurveReconstructor::Parameters line_fill_params;
CurveFitter3D::Parameters params_fiiting;

class LineDetector {
public:
    LineDetector(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize parameters with default values
        nh_.param<std::string>("image_topic_name", image_topic_, "/camera/color/image_raw");
        // nh_.param<std::string>("camera_info_topic_name", camera_info_topic_, "/camera/depth/camera_info");   
        // nh_.param<std::string>("depth_topic_name", depth_topic_, "/camera/depth/image_raw");
        nh_.param<std::string>("camera_info_topic_name", camera_info_topic_, "/camera/aligned_depth_to_color/camera_info");     //for realsense
        nh_.param<std::string>("depth_topic_name", depth_topic_, "/camera/aligned_depth_to_color/image_raw");    //for realsense
        
        
        // Camera parameters (initialize with default values)
        fx_ = 512.8;  // Focal length in x (pixels)
        fy_ = 483.1;  // Focal length in y (pixels)
        cx_ = 320.0;  // Principal point x (pixels)
        cy_ = 240.0;  // Principal point y (pixels)

        // Initialize HSV threshold parameters
        hmin_ = 0; hmin_max_ = 360;
        hmax_ = 60; hmax_max_ = 180;
        smin_ = 38; smin_max_ = 255;
        smax_ = 138; smax_max_ = 255;
        vmin_ = 205; vmin_max_ = 255;
        vmax_ = 255; vmax_max_ = 255;
        h_range_ = 30; s_range_ = 50; v_range_ = 50;
        
        // Setup ROS subscribers
        image_sub_ = nh_.subscribe(image_topic_, 1, &LineDetector::imageCallback, this);
        info_sub_ = nh_.subscribe(camera_info_topic_, 1, &LineDetector::cameraInfoCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic_, 1, &LineDetector::depthCallback, this);

        // Setup ROS publishers
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/line_points_markers", 1);
        marked_image_pub_ = nh_.advertise<sensor_msgs::Image>("/line_detection/marked_image", 1);
        line_path_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/line_path", 10);
        
        // Create windows and trackbars
        cv::namedWindow("origin_image", cv::WINDOW_GUI_EXPANDED);
        cv::namedWindow("hsv_image", cv::WINDOW_GUI_EXPANDED);
        cv::setMouseCallback("origin_image", &LineDetector::onMouse, this);
        
        createTrackbars();
    }

    ~LineDetector() {
        cv::destroyAllWindows();
    }
    
    void run() {
        ros::Rate rate(30);
        while (ros::ok()) {
            if (image_received_ && !image_.empty()) {
                processFrame();
            }
            
            int key = cv::waitKey(1);
            if (key == 27) {  // ESC key
                break;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // ROS members
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_, info_sub_, depth_sub_;
    ros::Publisher marker_pub_, marked_image_pub_, line_path_pub_;
    std::string image_topic_, camera_info_topic_, depth_topic_;
    
    // Image data
    cv::Mat K_, image_, depth_, bgr_, hsv_, dst_;
    bool image_received_ = false;
    bool depth_received_ = false;
    bool camera_info_received_ = false;
    
    // Processing variables
    cv::Point cpoint_;
    std::vector<cv::Point> center_of_line_;
    
    // Camera parameters
    double fx_, fy_, cx_, cy_;

    // HSV parameters
    int hmin_, hmax_, smin_, smax_, vmin_, vmax_;
    int hmin_max_, hmax_max_, smin_max_, smax_max_, vmin_max_, vmax_max_;
    int h_range_, s_range_, v_range_;

    // cv::Point3f pixelTo3D(const cv::Point& pixel, float depth) {
    //     if (depth <= 0) return cv::Point3f(0, 0, 0);
        
    //     float Z = depth / 1000.0f;  // Convert to meters
    //     float X = (pixel.x - cx_) * Z / fx_;
    //     float Y = (pixel.y - cy_) * Z / fy_;
    //     return cv::Point3f(X, Y, Z);
    // }

    // cv::Point3f pixelTo3D(const cv::Point& pixel, float depth, float depth_scale = 1.0f / 1000.0f) {
    //     // Check for invalid depth or uninitialized camera matrix
    //     if (depth <= 0 || !camera_info_received_ || K_.empty()) {
    //         return cv::Point3f(0, 0, 0);
    //     }

    //     // Extract camera intrinsics from K_
    //     const float fx = K_.at<float>(0, 0);
    //     const float fy = K_.at<float>(1, 1);
    //     const float cx = K_.at<float>(0, 2);
    //     const float cy = K_.at<float>(1, 2);

    //     // Check for invalid camera parameters
    //     if (fx <= 1e-6f || fy <= 1e-6f) {
    //         ROS_WARN_THROTTLE(1.0, "Invalid camera parameters (fx: %f, fy: %f)", fx, fy);
    //         return cv::Point3f(0, 0, 0);
    //     }

    //     // Convert to 3D point (in meters)
    //     const float Z = depth * depth_scale;
    //     const float X = (pixel.x - cx) * Z / fx;
    //     const float Y = (pixel.y - cy) * Z / fy;

    //     return cv::Point3f(X, Y, Z);
    // }


    /**
     * @brief 将像素坐标 + 深度值转换为 3D 点（相机坐标系）
     * @param pixel  输入像素坐标 (u, v)
     * @param depth  深度值（单位取决于 depth_scale，默认毫米）
     * @param depth_scale 深度缩放因子（默认 1/1000，即毫米转米）
     * @return 3D 点 (X, Y, Z)（单位：米），无效时返回 (0, 0, 0)
     */
    cv::Point3f pixelTo3D(const cv::Point& pixel, float depth, float depth_scale = 1.0f / 1000.0f) {
        // 1. 检查输入有效性
        if (depth <= 0 || !camera_info_received_ || K_.empty()) {
            ROS_WARN_THROTTLE(1.0, "Invalid input: depth=%f, K_ empty=%d", depth, K_.empty());
            return cv::Point3f(0, 0, 0);
        }

        // 2. 从相机矩阵 K_ 提取参数
        const float fx = K_.at<float>(0, 0);
        const float fy = K_.at<float>(1, 1);
        const float cx = K_.at<float>(0, 2);
        const float cy = K_.at<float>(1, 2);
        // const float fx = 512.9103088378906;
        // const float fy = 483.6195068359375;
        // const float cx = 320.9103088378906;
        // const float cy = 200.88180541992188;

        if (fx <= 1e-6f || fy <= 1e-6f) {
            ROS_ERROR_THROTTLE(1.0, "Invalid camera intrinsics: fx=%f, fy=%f", fx, fy);
            return cv::Point3f(0, 0, 0);
        }

        // 3. 使用矩阵运算计算 3D 点
        const float Z = depth * depth_scale;  // 转换为米
        cv::Mat pt2D = (cv::Mat_<float>(3, 1) << (pixel.x - cx) / fx,  // (u - cx)/fx
                                                (pixel.y - cy) / fy,  // (v - cy)/fy
                                                1.0f);                // 齐次坐标
        cv::Mat pt3D = Z * pt2D;  // 缩放至实际距离

        // 4. 返回结果
        return cv::Point3f(pt3D.at<float>(0),  // X
                        pt3D.at<float>(1),  // Y
                        pt3D.at<float>(2)); // Z
    }


    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                        const std::string& frame_id = "camera_color_optical_frame") {
        if (!cloud || cloud->empty()) {
            ROS_WARN("Input cloud is empty or null!");
            return;
        }

        // 创建一个 PointCloud2 消息
        sensor_msgs::PointCloud2 output;
        
        // 将 PCL 点云转换为 ROS PointCloud2 消息
        pcl::toROSMsg(*cloud, output);
        
        // 设置消息头
        output.header.stamp = ros::Time::now();
        output.header.frame_id = frame_id;
        
        // 发布消息
        line_path_pub_.publish(output);
        
        ROS_DEBUG("Published point cloud with %lu points", cloud->size());
    }

    visualization_msgs::MarkerArray createMarkerArray(const std::vector<cv::Point3f>& points_3d) {
        visualization_msgs::MarkerArray marker_array;
        
        // Clear previous markers
        visualization_msgs::Marker delete_markers;
        delete_markers.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_markers);
        
        // Create markers for each 3D point
        for (size_t i = 0; i < points_3d.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_color_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "line_points";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = points_3d[i].x;
            marker.pose.position.y = points_3d[i].y;
            marker.pose.position.z = points_3d[i].z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.005;  // 2cm diameter
            marker.scale.y = 0.005;
            marker.scale.z = 0.005;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0.5);
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

    visualization_msgs::MarkerArray createMarkerArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float r = 1, float g = 0, float b = 0, float duration = 1.0) {
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker delete_markers;
    delete_markers.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_markers);
    
    // Create markers for each 3D point
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = cloud->points[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_color_optical_frame";  // 你的坐标系
        marker.header.stamp = ros::Time::now();
        marker.ns = "filtered_points";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
        marker.pose.orientation.w = 1.0;  // 无旋转
        marker.scale.x = 0.01;  // 1cm diameter (adjust as needed)
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = r;    // 绿色
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;    // 不透明
        marker.lifetime = ros::Duration(duration);  // 0.5秒后自动消失
        
        marker_array.markers.push_back(marker);
    }
    
    return marker_array;
}
    
    void createTrackbars() {
        cv::createTrackbar("hmin", "hsv_image", &hmin_, hmin_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("hmax", "hsv_image", &hmax_, hmax_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("smin", "hsv_image", &smin_, smin_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("smax", "hsv_image", &smax_, smax_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("vmin", "hsv_image", &vmin_, vmin_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("vmax", "hsv_image", &vmax_, vmax_max_, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("h_range", "hsv_image", &h_range_, 100, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("s_range", "hsv_image", &s_range_, 100, &LineDetector::trackbarCallback, this);
        cv::createTrackbar("v_range", "hsv_image", &v_range_, 100, &LineDetector::trackbarCallback, this);
    }
    
    static void trackbarCallback(int, void* userdata) {
        LineDetector* detector = static_cast<LineDetector*>(userdata);
        detector->processFrame();
    }
    
    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        LineDetector* detector = static_cast<LineDetector*>(userdata);
        detector->handleMouseEvent(event, x, y, flags);
    }
    
    void handleMouseEvent(int event, int x, int y, int flags) {
        if (event == cv::EVENT_LBUTTONDOWN && !hsv_.empty()) {
            cv::Vec3b hsv_pixel = hsv_.at<cv::Vec3b>(y, x);
            int h = hsv_pixel[0];
            int s = hsv_pixel[1];
            int v = hsv_pixel[2];
            
            ROS_INFO("Clicked at (%d, %d) - HSV: (%d, %d, %d)", x, y, h, s, v);
            
            // Update HSV ranges based on clicked point
            hmin_ = std::max(0, h - h_range_);
            hmax_ = std::min(hmax_max_, h + h_range_);
            smin_ = std::max(0, s - s_range_);
            smax_ = std::min(smax_max_, s + s_range_);
            vmin_ = std::max(0, v - v_range_);
            vmax_ = std::min(vmax_max_, v + v_range_);
            
            // Update trackbars
            cv::setTrackbarPos("hmin", "hsv_image", hmin_);
            cv::setTrackbarPos("hmax", "hsv_image", hmax_);
            cv::setTrackbarPos("smin", "hsv_image", smin_);
            cv::setTrackbarPos("smax", "hsv_image", smax_);
            cv::setTrackbarPos("vmin", "hsv_image", vmin_);
            cv::setTrackbarPos("vmax", "hsv_image", vmax_);
            
            processFrame();
        }
    }
    
    void processFrame() {
        if (hsv_.empty()) return;
        
        // Output image
        dst_ = cv::Mat::zeros(image_.size(), image_.type());
        
        // Create mask based on HSV thresholds
        cv::Mat mask;
        cv::inRange(hsv_, cv::Scalar(hmin_, smin_, vmin_), cv::Scalar(hmax_, smax_, vmax_), mask);
        
        // Apply mask to original image
        bgr_.copyTo(dst_, mask);
        
        // Apply morphological operations
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_OPEN, cv::Size(3, 3));
        cv::morphologyEx(dst_, dst_, cv::MORPH_OPEN, kernel);
        cv::medianBlur(dst_, dst_, 3);
        
        // Convert to grayscale and threshold
        cv::Mat gray, binary;
        cv::cvtColor(dst_, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 0, 255, cv::THRESH_OTSU);
        cv::medianBlur(binary, binary, 9);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::Mat kernel_test = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(5, 5));
        cv::erode(binary, binary, kernel_test);
        cv::Mat kernel_test1 = cv::getStructuringElement(cv::MORPH_DILATE, cv::Size(7, 7));
        cv::dilate(binary, binary, kernel_test1);
        
        // Process binary image
        processBinaryImage(binary);

        // Convert line points to 3D and publish markers
        if (!center_of_line_.empty() && !depth_.empty() && depth_received_) {
            std::vector<cv::Point3f> points_3d;
            
            // Convert a subset of points (every 5th point for efficiency)
            for (size_t i = 0; i < center_of_line_.size(); i += 5) {
                const cv::Point& pt = center_of_line_[i];
                if (pt.y >= 0 && pt.y < depth_.rows && pt.x >= 0 && pt.x < depth_.cols) {
                    float depth = depth_.at<uint16_t>(pt.y, pt.x);
                    if (depth > 0) {
                        points_3d.push_back(pixelTo3D(pt, depth));
                    }
                }
            }
            
            if (!points_3d.empty()) {
                // visualization_msgs::MarkerArray marker_array = createMarkerArray(points_3d);
                // marker_pub_.publish(marker_array);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : points_3d) {
                cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z)); 
            }

            // 3. PCL 半径滤波 (移除离群点)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
            radius_filter.setInputCloud(cloud);
            radius_filter.setRadiusSearch(0.01);      // 搜索半径（单位：米，根据数据调整）
            radius_filter.setMinNeighborsInRadius(25);  // 邻域内最少点数（低于此值则视为离群点）
            radius_filter.filter(*cloud_filtered);
            // 4. 输出滤波前后的点数
            std::cout << "原始点数: " << cloud->size() << std::endl;
            std::cout << "滤波后点数: " << cloud_filtered->size() << std::endl;
            // visualization_msgs::MarkerArray markers = createMarkerArray(cloud_filtered, 1, 0, 0, 1);
            // marker_pub_.publish(markers);


            // ################曲线拟合与处理部分####################
                // 曲线提取

            pcl::PointCloud<pcl::PointXYZ>::Ptr line_seg(new pcl::PointCloud<pcl::PointXYZ>);
            params.cluster_tolerance = 0.05f;
            params.min_cluster_size = 60;
            CurveSegment segger(params);
            
            auto curves = segger.process(cloud_filtered);

            for (size_t i = 0; i < curves.size(); ++i) {
                std::cout << "Curve " << i << " has " << curves[i]->size() << " points." << std::endl;
            }
            // line_seg = mergeClouds(curves);
            // visualization_msgs::MarkerArray markers_test = createMarkerArray(line_seg, 0, 1, 0);
            // marker_pub_.publish(markers_test);

            // #################曲线连接##########################
            // CurveReconstructor connector(line_fill_params);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud = connector.process(cloud_filtered);
            // visualization_msgs::MarkerArray markers_test = createMarkerArray(result_cloud, 0, 1, 0);
            // marker_pub_.publish(markers_test);

            // ######################test fitting#######################
            params_fiiting.fitted_points = 300; // 设置拟合点数
            CurveFitter3D fitter(params_fiiting);
            pcl::PointCloud<pcl::PointXYZ>::Ptr result =  fitter.fitSingleCurve(curves);
            // visualization_msgs::MarkerArray markers_test = createMarkerArray(result, 0, 1, 0);
            // marker_pub_.publish(markers_test);
            publishPointCloud(result);

            
        }

        // Show results
        if (!dst_.empty()) {
            cv::imshow("origin_image", image_);
            cv::imshow("hsv_image", dst_);

            // Publish marked image
            cv_bridge::CvImage marked_img_msg;
            marked_img_msg.header.stamp = ros::Time::now();
            marked_img_msg.encoding = "bgr8";
            marked_img_msg.image = dst_.clone();
            marked_image_pub_.publish(marked_img_msg.toImageMsg());
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr mergeClouds(
            const std::vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>& cloud_vector)
        {
            // Create the output cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            // Iterate through all clouds in the vector
            for (const auto& cloud_ptr : cloud_vector) {
                if (cloud_ptr && !cloud_ptr->empty()) {
                    *merged_cloud += *cloud_ptr;  // Concatenate the point clouds
                }
            }
            
            return merged_cloud;
        }
    
    void processBinaryImage(cv::Mat& binary) {
        // Image thinning/skeletonization
        cv::Mat thinned = thinImage(binary);
        cv::Mat result = cv::Mat::zeros(thinned.size(), CV_8UC3);
        
        // Filter over-connected points
        filterOver(thinned);
        
        // Find key points
        std::vector<cv::Point> points = getPoints(thinned, 2, 10, 2);
        
        // Convert to color image for visualization
        thinned = thinned * 255;
        cv::cvtColor(thinned, result, cv::COLOR_GRAY2BGR);
        
        // Draw all points
        for (const auto& point : points) {
            cv::circle(result, point, 1, cv::Scalar(255, 0, 0), -1);
        }
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thinned, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
        if (!contours.empty()) {
            // Sort contours by area
            std::sort(contours.begin(), contours.end(), 
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) > cv::contourArea(b);
                });

            // 2. 过滤掉面积小于阈值的轮廓（例如面积 < 100 像素）
            const double area_threshold = 250.0;

            // 方法1：C++20 的 std::erase_if（推荐）
            #if __cplusplus >= 202002L
            std::erase_if(contours, 
                [area_threshold](const std::vector<cv::Point>& c) {
                    return cv::contourArea(c) < area_threshold;
                });
            
            // 方法2：手动遍历删除（兼容所有 C++ 版本）
            #else
            contours.erase(
                std::remove_if(contours.begin(), contours.end(),
                    [area_threshold](const std::vector<cv::Point>& c) {
                        return cv::contourArea(c) < area_threshold;
                    }),
                contours.end());
            #endif
            
            // Process largest contour
            // std::vector<cv::Point> largest_contour = contours[0];

            std::vector<cv::Point> largest_contour;
            for (const auto& contour : contours) {
                largest_contour.insert(largest_contour.end(), contour.begin(), contour.end());
            }

            center_of_line_.clear();
            
            // Filter points inside the largest contour
            try {
                // 检查轮廓是否有效
                if (largest_contour.empty()) {
                    throw std::runtime_error("Empty contour provided to pointPolygonTest");
                }

                for (const auto& point : points) {
                    // 检查点是否有效（可选）
                    if (point.x < 0 || point.y < 0) {
                        throw std::runtime_error("Invalid point coordinates");
                    }

                    double test_result;
                    try {
                        test_result = cv::pointPolygonTest(largest_contour, point, false);
                    } catch (const cv::Exception& e) {
                        // 捕获OpenCV内部异常并重新抛出更友好的消息
                        throw std::runtime_error(std::string("OpenCV error in pointPolygonTest: ") + e.what());
                    }

                    if (test_result != -1) {  // 严格在内部
                        center_of_line_.push_back(point);
                        cv::circle(result, point, 1, cv::Scalar(0, 0, 255), -1);
                    }
                }
                } catch (const std::exception& e) {
                    // 在这里处理异常，可以记录日志或采取其他恢复措施
                    std::cerr << "Error in processing points: " << e.what() << std::endl;
                    
                    // 如果你想继续向上层抛出异常，取消下面这行的注释
                    // throw;
                }
            
            // Draw contour on output image
            cv::drawContours(dst_, contours, 0, cv::Scalar(0, 0, 255), 1);
        }
        
        cv::imshow("result", result);
    }
    
    cv::Mat thinImage(const cv::Mat &src, const int maxIterations = -1) {
        assert(src.type() == CV_8UC1);
        cv::Mat dst;
        int width = src.cols;
        int height = src.rows;
        src.copyTo(dst);
        int count = 0;
        
        while (true) {
            count++;
            if (maxIterations != -1 && count > maxIterations) break;
            
            std::vector<uchar*> mFlag;
            
            // Mark points to delete
            for (int i = 0; i < height; ++i) {
                uchar* p = dst.ptr<uchar>(i);
                for (int j = 0; j < width; ++j) {
                    if (p[j] != 1) continue;
                    
                    // Get 8-connected neighbors
                    uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                    uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                    uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                    uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                    uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                    uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                    uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                    uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                    
                    if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && 
                        (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6) {
                        int ap = 0;
                        if (p2 == 0 && p3 == 1) ++ap;
                        if (p3 == 0 && p4 == 1) ++ap;
                        if (p4 == 0 && p5 == 1) ++ap;
                        if (p5 == 0 && p6 == 1) ++ap;
                        if (p6 == 0 && p7 == 1) ++ap;
                        if (p7 == 0 && p8 == 1) ++ap;
                        if (p8 == 0 && p9 == 1) ++ap;
                        if (p9 == 0 && p2 == 1) ++ap;

                        if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0) {
                            mFlag.push_back(p + j);
                        }
                    }
                }
            }

            // Delete marked points
            for (auto& p : mFlag) {
                *p = 0;
            }

            if (mFlag.empty()) break;
        }

        return dst;
    }
    
    void filterOver(cv::Mat thinSrc) {
        assert(thinSrc.type() == CV_8UC1);
        int width = thinSrc.cols;
        int height = thinSrc.rows;
        
        for (int i = 0; i < height; ++i) {
            uchar* p = thinSrc.ptr<uchar>(i);
            for (int j = 0; j < width; ++j) {
                if (p[j] != 1) continue;
                
                // Get 4-connected neighbors
                uchar p2 = (i == 0) ? 0 : *(p - thinSrc.step + j);
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + thinSrc.step + j);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                
                if (p2 + p4 + p6 + p8 >= 1) {
                    p[j] = 0;
                }
            }
        }
    }
    
    std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis = 4, 
                                    unsigned int thresholdMax = 6, unsigned int thresholdMin = 4) {
        assert(thinSrc.type() == CV_8UC1);
        int width = thinSrc.cols;
        int height = thinSrc.rows;
        std::vector<cv::Point> points;
        
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                if (thinSrc.at<uchar>(i, j) == 0) continue;
                
                int count = 0;
                for (int k = i - raudis; k < i + raudis + 1; k++) {
                    for (int l = j - raudis; l < j + raudis + 1; l++) {
                        if (k >= 0 && l >= 0 && k < height && l < width && thinSrc.at<uchar>(k, l) == 1) {
                            count++;
                        }
                    }
                }

                if (count > thresholdMax || count < thresholdMin) {
                    points.emplace_back(j, i);
                }
            }
        }
        
        return points;
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            image_ = cv_ptr->image.clone();
            if (!image_.empty()) {
                image_received_ = true;
                bgr_ = image_.clone();
                cv::cvtColor(bgr_, hsv_, cv::COLOR_BGR2HSV);
            }
        }
        catch (const cv_bridge::Exception &e) {
            ROS_ERROR("Image callback error: %s", e.what());
        }
    }
    
    void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
            depth_ = cv_ptr->image.clone();
            if (!depth_.empty()) {
                depth_received_ = true;
            }
        }
        catch (const cv_bridge::Exception &e) {
            ROS_ERROR("Depth callback error: %s", e.what());
        }
    }
    
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (!camera_info_received_) {
            K_ = cv::Mat(3, 3, CV_32F);
            for (int i = 0; i < 9; ++i)
                K_.at<float>(i/3, i%3) = msg->K[i];

            camera_info_received_ = true;
            ROS_INFO_STREAM("Camera matrix:\n" << K_);
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_detection_node");
    ros::NodeHandle nh("~");  // Private node handle for parameters
    
    LineDetector detector(nh);
    detector.run();
    
    return 0;
}