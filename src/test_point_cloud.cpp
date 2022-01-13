#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <memory>
#include <Eigen/Core>
#include "Test-Lidar-Algo/sensor_topic.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "Test-Lidar-Algo/sensor_type.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_point_cloud");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "points_raw", 100000);
    std::shared_ptr<ImuSubscriber> imu_sub_ptr = std::make_shared<ImuSubscriber>(nh, "imu_correct", 100000);
    std::shared_ptr<GpsSubscriber> gnss_sub_ptr = std::make_shared<GpsSubscriber>(nh, "gps/fix", 100000);
    // std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    // std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    // std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    // std::deque<CloudData> cloud_data_buff;
    // std::deque<IMUData> imu_data_buff;
    // std::deque<GNSSData> gnss_data_buff;

    // 根据KITTI cali文件获得lidar到imu的坐标变换
    // Eigen::MatrixXd T_lidar2imu = Eigen::MatrixXd::Identity(4, 4);
    // Eigen::Matrix3d R_imu2lidar;
    // Eigen::Vector3d t_imu2lidar(-8.086759e-01, 3.195559e-01, -7.997231e-01);

    // R_imu2lidar <<  9.999976e-01, 7.553071e-04, -2.035826e-03,
    //                -7.854027e-04, 9.998898e-01, -1.482298e-02,
    //                 2.024406e-03, 1.482454e-02,  9.998881e-01;
    // t_imu2lidar = -R_imu2lidar.transpose() * t_imu2lidar;
    // T_lidar2imu.block<3, 3>(0, 0) = R_imu2lidar.transpose();
    // T_lidar2imu.block<3, 1>(0, 3) = t_imu2lidar;
    // std::cout << T_lidar2imu << std::endl;
    // bool transform_received = false;
    // bool gnss_origin_position_inited = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        std::cout << cloud_sub_ptr->buf_dq_.size() << std::endl;
        // while (cloud_sub_ptr->buf_dq_.size() > 0 && imu_sub_ptr->buf_dq_.size() > 0 && gnss_sub_ptr->buf_dq_.size() > 0) {
        //     CloudData cloud_data = cloud_sub_ptr->buf_dq_.front();
        //     IMUData imu_data = imu_sub_ptr->buf_dq_.front();
        //     GNSSData gnss_data = gnss_sub_ptr->buf_dq_.front();
        //     std::cout << "cloud_data: " << cloud_data.time << std::endl;
        //     std::cout << "imu_data: " << imu_data.time << std::endl;
        //     std::cout << "gnss_data: " << gnss_data.time << std::endl;
        //     // double d_time = cloud_data.time - imu_data.time;
        //     // if (d_time < -0.05) {
        //     //     cloud_data_buff.pop_front();
        //     // } else if (d_time > 0.05) {
        //     //     imu_data_buff.pop_front();
        //     //     gnss_data_buff.pop_front();
        //     // } else {
        //     //     cloud_data_buff.pop_front();
        //     //     imu_data_buff.pop_front();
        //     //     gnss_data_buff.pop_front();

        //     //     Eigen::Matrix4f odometry_matrix;

        //     //     if (!gnss_origin_position_inited) {
        //     //         gnss_data.InitOriginPosition();
        //     //         gnss_origin_position_inited = true;
        //     //     }
        //     //     gnss_data.UpdateXYZ();
        //     //     odometry_matrix(0,3) = gnss_data.local_E;
        //     //     odometry_matrix(1,3) = gnss_data.local_N;
        //     //     odometry_matrix(2,3) = gnss_data.local_U;
        //     //     odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
        //     //     odometry_matrix *= lidar_to_imu;

        //     //     pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);

        //     //     cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
        //     //     odom_pub_ptr->Publish(odometry_matrix);
        //     // }
        // }

        rate.sleep();
    }

    return 0;
}

