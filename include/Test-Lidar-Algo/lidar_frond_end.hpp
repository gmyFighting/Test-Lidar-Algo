#ifndef _LIDAR_FROND_END_HPP_
#define _LIDAR_FROND_END_HPP_

#include <ros/ros.h>
#include <deque>
#include <Eigen/Core>
#include "Test-Lidar-Algo/sensor_type.hpp"
#include "Test-Lidar-Algo/sensor_topic.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class Frond_End {
public:
    Frond_End(ros::NodeHandle &nh);
    void lidar_odom_update();
    void gnss_update();
    void read_data();
    bool has_data();
    void fix_origin();
    bool pub_global_map();

private:
    Eigen::Matrix4d T_lidar2imu = Eigen::Matrix4d::Identity();
    GeographicLib::LocalCartesian geo_converter;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr;
    std::shared_ptr<ImuSubscriber> imu_sub_ptr;
    std::shared_ptr<GpsSubscriber> gnss_sub_ptr;

    CloudData current_cloud;
    IMUData current_imu;
    GNSSData current_gnss;

    Eigen::Matrix4d lidar_odom_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d gnss_odom_matrix = Eigen::Matrix4d::Identity();
};



#endif
