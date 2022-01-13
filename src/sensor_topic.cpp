// 传感器消息的发布订阅
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "Test-Lidar-Algo/sensor_topic.hpp"
#include "Test-Lidar-Algo/sensor_type.hpp"

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, const size_t &buff_size)
{
    subscriber_ = nh.subscribe(topic_name, buff_size, &CloudSubscriber::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
}

GpsSubscriber::GpsSubscriber(ros::NodeHandle &nh, const std::string &topic_name, const size_t &buff_size)
{
    subscriber_ = nh.subscribe(topic_name, buff_size, &GpsSubscriber::gpsHandler, this, ros::TransportHints().tcpNoDelay());
}

ImuSubscriber::ImuSubscriber(ros::NodeHandle& nh, const std::string &topic_name, const size_t &buff_size)
{
    subscriber_ = nh.subscribe(topic_name, buff_size, &ImuSubscriber::imuHandler, this, ros::TransportHints().tcpNoDelay());
}

void CloudSubscriber::laserCloudInfoHandler(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
    CloudData data;
    data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(data.cloud_ptr));
    buf_dq_.push_back(data);

    // std::cout << "find a cloud\n";
}

void GpsSubscriber::gpsHandler(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
{
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->longitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;// >0时有效
    // gnss_data.service = nav_sat_fix_ptr->status.service;

    buf_dq_.push_back(gnss_data);    
    // std::cout << "find a gps\n";
}

void ImuSubscriber::imuHandler(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    // 加速度
    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;
    // 角速度
    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;
    // 四元数
    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    buf_dq_.push_back(imu_data);    
    // std::cout << "find a imu\n";
}



