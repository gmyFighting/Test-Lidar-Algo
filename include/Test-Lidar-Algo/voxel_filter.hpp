#ifndef _VOXEL_FILTER_HPP_
#define _VOXEL_FILTER_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

class VoxelFilter {
public:
    VoxelFilter();
    VoxelFilter(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z);

    void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr);

private:
    // bool set_filter_param(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
};




#endif
