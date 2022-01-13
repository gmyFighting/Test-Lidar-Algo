#ifndef _SENSOR_TOPIC_HPP_
#define _SENSOR_TOPIC_HPP_

#include <ros/ros.h>
#include <deque>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include "Test-Lidar-Algo/sensor_type.hpp"

class CloudSubscriber {
public:
    CloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, const size_t &buff_size);
    CloudSubscriber() = default;
    // void ParseData(std::deque<CloudData>& deque_cloud_data);
    std::deque<CloudData> buf_dq_;

private:
    ros::Subscriber subscriber_;
    void laserCloudInfoHandler(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
};

class GpsSubscriber {
public:
    GpsSubscriber(ros::NodeHandle &nh, const std::string &topic_name, const size_t &buff_size);
    GpsSubscriber() = default;

    std::deque<GNSSData> buf_dq_;
    // void ParseData(std::deque<GNSSData>& deque_gnss_data);

private:
    ros::Subscriber subscriber_;
    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr); 
};

class ImuSubscriber {
public:
    ImuSubscriber(ros::NodeHandle &nh, const std::string &topic_name, const size_t &buff_size);
    ImuSubscriber() = default;

    std::deque<IMUData> buf_dq_; 
    // void ParseData(std::deque<IMUData>& deque_imu_data);

private:
    void imuHandler(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    ros::Subscriber subscriber_;
};






#endif
