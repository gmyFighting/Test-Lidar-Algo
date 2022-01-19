#include "Test-Lidar-Algo/voxel_filter.hpp"

VoxelFilter::VoxelFilter() {
    float leaf_size_x = 1.3;
    float leaf_size_y = 1.3;
    float leaf_size_z = 1.3;
    voxel_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z) {
    voxel_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

void VoxelFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, \
             pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) {
    voxel_filter.setInputCloud(input_cloud_ptr);
    voxel_filter.filter(*filtered_cloud_ptr);
}

// void VoxelFilter::set_filter_param(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z) {
// }
