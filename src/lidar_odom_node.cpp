// lidar前端
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <memory>
#include "Test-Lidar-Algo/sensor_topic.hpp"
#include "Test-Lidar-Algo/sensor_type.hpp"
#include "Test-Lidar-Algo/lidar_frond_end.hpp"

Eigen::Matrix4d odometry_matrix;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_odom_node");
    ros::NodeHandle nh;

    std::shared_ptr<Frond_End> front_end_ptr = std::make_shared<Frond_End>(nh);

    bool origin_pos_inited = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        
        while (front_end_ptr->has_data()) {
            // 1. 读取imu gps 点云数据，利用点云时间戳同步imu/gps,若是第一帧做初始化
            front_end_ptr->read_data();

            // 2. 固定原点
            if (origin_pos_inited == false) {
                front_end_ptr->fix_origin();
                origin_pos_inited = true;
                rate.sleep();
            }
            // 3. 利用imu、gps生成gt && lidar前端流程
            // update_lidar_odom();
        }



        rate.sleep();
    }

    return 0;
}

