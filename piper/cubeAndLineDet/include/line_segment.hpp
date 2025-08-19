#ifndef CURVE_FITTER_3D_HPP
#define CURVE_FITTER_3D_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

class CurveSegment {
public:
    struct Parameters {
        // 聚类参数
        float cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
        
        // B样条参数
        int bspline_order;
        float bspline_smoothing;
        int bspline_refinement;
        
        // 缺失检测参数
        float gap_threshold_multiplier;
        
        // 交叉处理参数
        float intersection_search_radius;

        Parameters() :
            cluster_tolerance(0.5f),
            min_cluster_size(100),
            max_cluster_size(10000),
            bspline_order(3),
            bspline_smoothing(0.05f),
            bspline_refinement(1),
            gap_threshold_multiplier(8.0f),
            intersection_search_radius(0.02f) {}
    };

    CurveSegment(const Parameters& params = Parameters()) : params_(params) {}

    // 主处理函数
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> process(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> curves;
        
        if (!cloud || cloud->empty()) {
            return curves;
        }

        // 1. 分割不同曲线
        curves = segmentCurves(cloud);
        
        // 2. 分别处理每条曲线
        for (auto& curve : curves) {
            // fit3DBSpline(curve);
            // detectAndFillGaps(curve);
        }
        
        // 3. 处理曲线交叉
        // processCrossingCurves(curves);
        
        return curves;
    }

private:
    Parameters params_;

    // 曲线分割
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCurves(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> curves;
        
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_cluster_size);
        ec.setMaxClusterSize(params_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr curve(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, indices, *curve);
            curves.push_back(curve);
        }
        
        return curves;
    }

    // 三维B样条拟合
    void fit3DBSpline(pcl::PointCloud<pcl::PointXYZ>::Ptr& curve) {
        // 排序点云
        orderPointsAlongCurve(curve);
        
        // 准备Eigen数据
        Eigen::MatrixXd points(3, curve->size());
        for (size_t i = 0; i < curve->size(); ++i) {
            points(0, i) = curve->points[i].x;
            points(1, i) = curve->points[i].y;
            points(2, i) = curve->points[i].z;
        }
        
        // 生成参数t [0, 1]
        Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(curve->size(), 0.0, 1.0);
        
        // 拟合三维样条曲线
        typedef Eigen::Spline<double, 3> Spline3D;
        Spline3D spline = Eigen::SplineFitting<Spline3D>::Interpolate(
            points, 
            std::min<int>(curve->size()-1, params_.bspline_order),
            t);
        
        // 生成平滑曲线点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_curve(new pcl::PointCloud<pcl::PointXYZ>);
        int num_samples = curve->size() * params_.bspline_refinement;
        
        for (int i = 0; i < num_samples; ++i) {
            double t = static_cast<double>(i) / (num_samples - 1);
            Eigen::Vector3d point = spline(t);
            
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(point(0));
            pt.y = static_cast<float>(point(1));
            pt.z = static_cast<float>(point(2));
            fitted_curve->push_back(pt);
        }
        
        curve = fitted_curve;
    }

    // 缺失部分检测与插补
    void detectAndFillGaps(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (cloud->size() < 2) return;
        
        // 计算点间距
        std::vector<float> distances;
        for (size_t i = 1; i < cloud->size(); ++i) {
            distances.push_back(pcl::euclideanDistance(cloud->points[i-1], cloud->points[i]));
        }
        
        // 计算平均距离
        float mean_dist = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
        float threshold = params_.gap_threshold_multiplier * mean_dist;
        
        // 插补逻辑
        pcl::PointCloud<pcl::PointXYZ>::Ptr filled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < cloud->size(); ++i) {
            filled_cloud->push_back(cloud->points[i]);
            
            if (i < cloud->size() - 1) {
                float dist = pcl::euclideanDistance(cloud->points[i], cloud->points[i+1]);
                if (dist > threshold) {
                    int num_to_insert = static_cast<int>(dist / mean_dist) - 1;
                    for (int j = 1; j <= num_to_insert; ++j) {
                        float ratio = j / static_cast<float>(num_to_insert + 1);
                        pcl::PointXYZ new_point;
                        new_point.x = cloud->points[i].x + ratio * (cloud->points[i+1].x - cloud->points[i].x);
                        new_point.y = cloud->points[i].y + ratio * (cloud->points[i+1].y - cloud->points[i].y);
                        new_point.z = cloud->points[i].z + ratio * (cloud->points[i+1].z - cloud->points[i].z);
                        filled_cloud->push_back(new_point);
                    }
                }
            }
        }
        
        cloud = filled_cloud;
    }

    // 曲线交叉处理
    void processCrossingCurves(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& curves) {
        for (size_t i = 0; i < curves.size(); ++i) {
            for (size_t j = i+1; j < curves.size(); ++j) {
                if (checkCurvesIntersection(curves[i], curves[j])) {
                    adjustCurvesAtIntersection(curves[i], curves[j]);
                }
            }
        }
    }

    // 曲线交叉检测
    bool checkCurvesIntersection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& curve1,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& curve2) {
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(params_.intersection_search_radius);
        octree.setInputCloud(curve1);
        octree.addPointsFromInputCloud();
        
        for (const auto& pt : *curve2) {
            std::vector<int> indices;
            std::vector<float> sqr_distances;
            if (octree.radiusSearch(pt, params_.intersection_search_radius, indices, sqr_distances) > 0) {
                return true;
            }
        }
        return false;
    }

    // 曲线交叉调整
    void adjustCurvesAtIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr& curve1,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& curve2) {
        // 计算曲线1的主方向
        pcl::PCA<pcl::PointXYZ> pca1;
        pca1.setInputCloud(curve1);
        Eigen::Vector3f dir1 = pca1.getEigenVectors().col(0);
        
        // 在交叉点附近移动曲线2
        for (auto& pt : *curve2) {
            pt.x += 0.5f * params_.intersection_search_radius * dir1[0];
            pt.y += 0.5f * params_.intersection_search_radius * dir1[1];
            pt.z += 0.5f * params_.intersection_search_radius * dir1[2];
        }
    }

    // 点云沿曲线排序
    void orderPointsAlongCurve(pcl::PointCloud<pcl::PointXYZ>::Ptr& curve) {
        if (curve->size() < 3) return;
        
        // 使用PCA确定主方向
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(curve);
        Eigen::Vector3f centroid = pca.getMean().head<3>();
        Eigen::Vector3f eigenvector = pca.getEigenVectors().col(0); // 使用第一主成分
        
        // 沿主方向投影并排序
        std::vector<std::pair<float, size_t>> projections;
        for (size_t i = 0; i < curve->size(); ++i) {
            Eigen::Vector3f pt = curve->points[i].getVector3fMap() - centroid;
            float proj = pt.dot(eigenvector);
            projections.emplace_back(proj, i);
        }
        
        std::sort(projections.begin(), projections.end());
        
        // 重新排列点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr ordered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : projections) {
            ordered->push_back(curve->points[p.second]);
        }
        
        curve = ordered;
    }
};

#endif