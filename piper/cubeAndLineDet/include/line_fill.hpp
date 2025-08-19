#ifndef LINE_FIT_3D_HPP
#define LINE_FIT_3D_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
#include <vector>
#include <algorithm>

class CurveReconstructor {
public:
    struct Parameters {
        float cluster_tolerance = 0.1f;    // 曲线分割的容差
        float connection_threshold = 0.5f;  // 曲线连接阈值
        int min_curve_points = 30;         // 最小曲线点数
        float mls_search_radius = 0.5f;   // MLS平滑搜索半径
        int mls_poly_order = 2;            // MLS多项式阶数
        float max_gap_size = 0.5f;        // 最大插补间隙

        Parameters() :
            cluster_tolerance(0.5f),
            connection_threshold(0.1f),
            min_curve_points(20),
            mls_search_radius(0.05f),
            mls_poly_order(2),
            max_gap_size(0.15f){}
    };

    CurveReconstructor(const Parameters& params = Parameters()) : params_(params) {}

        pcl::PointCloud<pcl::PointXYZ>::Ptr process(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        
        // 1. 分割原始点云为曲线段
        auto curve_segments = extractCurveSegments(cloud);
        
        // 2. 平滑处理每条曲线段
        for (auto& segment : curve_segments) {
            smoothSegment(segment);
        }
        
        // 3. 连接相邻曲线段
        auto connected_curves = connectCurveSegments(curve_segments);
        
        // 4. 插补缺失部分
        auto filled_curves = fillCurveGaps(connected_curves);
        
        // 5. 合并所有曲线
        pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& curve : filled_curves) {
            *result += *curve;
        }
        
        return result;
    }

private:
    Parameters params_;

    // 曲线分割
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractCurveSegments(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;
        
        // 使用欧式聚类分割曲线
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_curve_points);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr segment(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, indices, *segment);
            segments.push_back(segment);
        }
        
        return segments;
    }

    // 曲线平滑
    void smoothSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr& segment) {
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(segment);
        mls.setSearchRadius(params_.mls_search_radius);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(params_.mls_poly_order);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZ>);
        mls.process(*smoothed);
        
        segment = smoothed;
    }

    // 曲线连接
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> connectCurveSegments(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments) {
        
        if (segments.size() <= 1) return segments;
        
        // 计算每条曲线的端点和切线方向
        std::vector<CurveEndInfo> end_infos;
        for (const auto& seg : segments) {
            end_infos.push_back(analyzeCurveEnds(seg));
        }
        
        // 尝试连接曲线
        std::vector<bool> connected(segments.size(), false);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> connected_curves;
        
        for (size_t i = 0; i < segments.size(); ++i) {
            if (connected[i]) continue;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_curve = segments[i];
            CurveEndInfo current_info = end_infos[i];
            
            // 寻找可以连接的曲线
            for (size_t j = i+1; j < segments.size(); ++j) {
                if (connected[j]) continue;
                
                CurveEndInfo other_info = end_infos[j];
                
                // 检查端点距离
                float dist1 = (current_info.end_point - other_info.start_point).norm();
                float dist2 = (current_info.start_point - other_info.end_point).norm();
                float dist3 = (current_info.end_point - other_info.end_point).norm();
                float dist4 = (current_info.start_point - other_info.start_point).norm();
                
                float min_dist = std::min({dist1, dist2, dist3, dist4});
                if (min_dist > params_.connection_threshold) continue;
                
                // 检查切线方向连续性
                bool reverse_other = false;
                Eigen::Vector3f dir1, dir2;
                
                if (min_dist == dist1) {
                    dir1 = current_info.end_tangent;
                    dir2 = other_info.start_tangent;
                } else if (min_dist == dist2) {
                    dir1 = current_info.start_tangent;
                    dir2 = other_info.end_tangent;
                    reverse_other = true;
                } else if (min_dist == dist3) {
                    dir1 = current_info.end_tangent;
                    dir2 = -other_info.end_tangent;
                    reverse_other = true;
                } else {
                    dir1 = current_info.start_tangent;
                    dir2 = -other_info.start_tangent;
                }
                
                float angle = dir1.dot(dir2);
                if (angle < 0.7) continue; // 方向差异太大(~45度)
                
                // 连接曲线
                pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
                
                if (min_dist == dist1) {
                    *merged = *current_curve + *segments[j];
                } else if (min_dist == dist2) {
                    *merged = *segments[j] + *current_curve;
                } else if (min_dist == dist3) {
                    pcl::PointCloud<pcl::PointXYZ> reversed = reverseCurve(segments[j]);
                    *merged = *current_curve + reversed;
                } else {
                    pcl::PointCloud<pcl::PointXYZ> reversed = reverseCurve(segments[j]);
                    *merged = reversed + *current_curve;
                }
                
                current_curve = merged;
                connected[j] = true;
                current_info = analyzeCurveEnds(merged);
            }
            
            connected_curves.push_back(current_curve);
            connected[i] = true;
        }
        
        return connected_curves;
    }

    // 曲线端点信息
    struct CurveEndInfo {
        Eigen::Vector3f start_point;
        Eigen::Vector3f end_point;
        Eigen::Vector3f start_tangent;
        Eigen::Vector3f end_tangent;
    };

    // 分析曲线端点
    CurveEndInfo analyzeCurveEnds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& curve) {
        CurveEndInfo info;
        
        // 排序点云
        orderPointsAlongCurve(curve);
        
        // 起点和终点
        info.start_point = curve->front().getVector3fMap();
        info.end_point = curve->back().getVector3fMap();
        
        // 计算起点切线方向(前5个点的平均方向)
        int num_tangent_points = std::min(5, (int)curve->size());
        Eigen::Vector3f start_dir = Eigen::Vector3f::Zero();
        for (int i = 1; i < num_tangent_points; ++i) {
            start_dir += (curve->points[i].getVector3fMap() - curve->points[i-1].getVector3fMap());
        }
        info.start_tangent = start_dir.normalized();
        
        // 计算终点切线方向(最后5个点的平均方向)
        Eigen::Vector3f end_dir = Eigen::Vector3f::Zero();
        for (int i = curve->size()-2; i >= (int)curve->size()-num_tangent_points-1 && i >= 0; --i) {
            end_dir += (curve->points[i].getVector3fMap() - curve->points[i+1].getVector3fMap());
        }
        info.end_tangent = end_dir.normalized();
        
        return info;
    }

    // 反转曲线点顺序
    pcl::PointCloud<pcl::PointXYZ> reverseCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& curve) {
        pcl::PointCloud<pcl::PointXYZ> reversed;
        reversed.reserve(curve->size());
        for (int i = curve->size()-1; i >= 0; --i) {
            reversed.push_back(curve->points[i]);
        }
        return reversed;
    }

        // 修正后的曲线插补方法
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fillCurveGaps(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& curves) {
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filled_curves;
        
        for (const auto& curve : curves) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filled(new pcl::PointCloud<pcl::PointXYZ>);
            
            // 排序点云
            orderPointsAlongCurve(curve);
            
            // 简单线性插补方法替代B样条
            for (size_t i = 1; i < curve->size(); ++i) {
                filled->push_back(curve->points[i-1]);
                
                float dist = pcl::euclideanDistance(curve->points[i-1], curve->points[i]);
                if (dist > params_.max_gap_size) {
                    // 计算需要插入的点数
                    int num_points_to_add = static_cast<int>(dist / params_.max_gap_size);
                    
                    // 线性插补
                    for (int j = 1; j <= num_points_to_add; ++j) {
                        float ratio = static_cast<float>(j) / (num_points_to_add + 1);
                        pcl::PointXYZ new_pt;
                        new_pt.x = curve->points[i-1].x + ratio * (curve->points[i].x - curve->points[i-1].x);
                        new_pt.y = curve->points[i-1].y + ratio * (curve->points[i].y - curve->points[i-1].y);
                        new_pt.z = curve->points[i-1].z + ratio * (curve->points[i].z - curve->points[i-1].z);
                        filled->push_back(new_pt);
                    }
                }
            }
            filled->push_back(curve->points.back());
            
            filled_curves.push_back(filled);
        }
        
        return filled_curves;
    }

    // 点云沿曲线排序
    void orderPointsAlongCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (cloud->empty()) return;
        
        // 使用最近邻排序
        pcl::PointCloud<pcl::PointXYZ>::Ptr ordered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        
        std::vector<bool> processed(cloud->size(), false);
        
        // 从第一个点开始
        ordered->push_back(cloud->points[0]);
        processed[0] = true;
        
        while (ordered->size() < cloud->size()) {
            pcl::PointXYZ last_point = ordered->back();
            
            // 找最近未处理的点
            std::vector<int> indices(2);
            std::vector<float> sqr_distances(2);
            kdtree.nearestKSearch(last_point, 2, indices, sqr_distances);
            
            int next_idx = -1;
            for (int idx : indices) {
                if (idx >= 0 && idx < cloud->size() && !processed[idx]) {
                    next_idx = idx;
                    break;
                }
            }
            
            if (next_idx == -1) {
                // 找不到未处理的邻近点，找最近的未处理点
                for (size_t i = 0; i < cloud->size(); ++i) {
                    if (!processed[i]) {
                        next_idx = i;
                        break;
                    }
                }
                if (next_idx == -1) break;
            }
            
            ordered->push_back(cloud->points[next_idx]);
            processed[next_idx] = true;
        }
        
        *cloud = *ordered;
    }
};

#endif