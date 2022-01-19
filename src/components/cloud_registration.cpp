#include "Test-Lidar-Algo/cloud_registration.hpp"

PCL_ICP::PCL_ICP() : icp_ptr(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>()) 
{
    float max_corr_dist = 1.2;
    float trans_eps = 0.01;
    float euc_fitness_eps = 0.36;
    int max_iter = 30;
    
    icp_ptr->setMaxCorrespondenceDistance(max_corr_dist);
    icp_ptr->setTransformationEpsilon(trans_eps);
    icp_ptr->setEuclideanFitnessEpsilon(euc_fitness_eps);
    icp_ptr->setMaximumIterations(max_iter);
}

bool PCL_ICP::set_input_target(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target)
{
    icp_ptr->setInputTarget(input_target);
    return true;
}

bool PCL_ICP::scan_match(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source, 
                          const Eigen::Matrix4d &predict_pose, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                              Eigen::Matrix4d &result_pose) 
{
    Eigen::Matrix4f predict_posef = predict_pose.cast<float>();
    Eigen::Matrix4f result_posef;
    icp_ptr->setInputSource(input_source);// 待匹配点云
    icp_ptr->align(*result_cloud_ptr, predict_posef);// 在predict_pose进行匹配
    result_posef = icp_ptr->getFinalTransformation();// 得到原始点云到目标点云的变换矩阵
    result_pose = result_posef.cast<double>();
    return true;
}
