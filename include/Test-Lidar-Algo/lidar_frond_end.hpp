#ifndef _LIDAR_FROND_END_HPP_
#define _LIDAR_FROND_END_HPP_

#include <ros/ros.h>
#include <deque>
#include <Eigen/Core>
#include "Test-Lidar-Algo/sensor_type.hpp"
#include "Test-Lidar-Algo/sensor_topic.hpp"
#include "Test-Lidar-Algo/voxel_filter.hpp"
#include "Test-Lidar-Algo/cloud_registration.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

class Frond_End {
public:
    // 关键帧(点云+位姿)
    struct Frame {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        CloudData cloud_data;
    };

    Frond_End(ros::NodeHandle &nh);
    void lidar_odom_update();
    void gnss_update();
    void read_data();
    bool has_data();
    bool data_is_valid();
    void fix_origin();
    bool pub_global_map();

private:
    Eigen::Matrix4d T_lidar2imu = Eigen::Matrix4d::Identity();
    // 功能组件
    GeographicLib::LocalCartesian geo_converter;
    VoxelFilter voxel_filter;
    PCL_ICP registration_method;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr;
    std::shared_ptr<ImuSubscriber> imu_sub_ptr;
    std::shared_ptr<GpsSubscriber> gnss_sub_ptr;

    CloudData current_cloud;
    IMUData current_imu;
    GNSSData current_gnss;

    // Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d lidar_odom_matrix = Eigen::Matrix4d::Identity();// lidar里程计位姿
    Eigen::Matrix4d gnss_odom_matrix = Eigen::Matrix4d::Identity();// gnss位姿
    Eigen::Matrix4d last_key_frame_pose = Eigen::Matrix4d::Identity();// 上一个有效关键帧

    Frame current_key_frame;// 当前点云关键帧(变换矩阵和点云数据)
    std::deque<Frame> local_map_frames;// 局部地图
    std::deque<Frame> global_map_frames;// 全局地图

    bool lidar_odom_inited = false;
    double key_frame_distance;
    
    bool update_map(const Frame &new_key_frame);
    bool key_frame_is_valid() {
        if (fabs(last_key_frame_pose(0,3) - current_key_frame.pose(0,3)) + 
            fabs(last_key_frame_pose(1,3) - current_key_frame.pose(1,3)) +
            fabs(last_key_frame_pose(2,3) - current_key_frame.pose(2,3)) > key_frame_distance) 
        {
            return true;
        }
        return false;
    }
};


#endif
