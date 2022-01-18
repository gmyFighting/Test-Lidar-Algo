#include "Test-Lidar-Algo/lidar_frond_end.hpp"


Frond_End::Frond_End(ros::NodeHandle &nh)
{
    cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "points_raw", 100000);
    imu_sub_ptr = std::make_shared<ImuSubscriber>(nh, "imu_correct", 100000);
    gnss_sub_ptr = std::make_shared<GpsSubscriber>(nh, "gps/fix", 100000);
}

bool Frond_End::has_data()
{
    if (cloud_sub_ptr->buf_dq_.size() != 0 || gnss_sub_ptr->buf_dq_.size() != 0 || imu_sub_ptr->buf_dq_.size() != 0) {
        return false;
    }

    return true;
}

void Frond_End::read_data()
{
    current_cloud = cloud_sub_ptr->buf_dq_.front();
    current_imu = imu_sub_ptr->buf_dq_.front();
    current_gnss = gnss_sub_ptr->buf_dq_.front();
}

void Frond_End::fix_origin()
{
    // 1. 计算lidar2imu
    // 根据KITTI cali文件获得lidar到imu的坐标变换
    Eigen::Matrix3d R_imu2lidar;
    Eigen::Vector3d t_imu2lidar(-8.086759e-01, 3.195559e-01, -7.997231e-01);

    R_imu2lidar <<  9.999976e-01, 7.553071e-04, -2.035826e-03,
                   -7.854027e-04, 9.998898e-01, -1.482298e-02,
                    2.024406e-03, 1.482454e-02,  9.998881e-01;
    t_imu2lidar = -R_imu2lidar.transpose() * t_imu2lidar;
    T_lidar2imu.block<3, 3>(0, 0) = R_imu2lidar.transpose();
    T_lidar2imu.block<3, 1>(0, 3) = t_imu2lidar;

    // 2. 固定gnss原点
    geo_converter.Reset(current_gnss.latitude, current_gnss.longitude, current_gnss.altitude);

    // 3. 固定lidar里程计原点???
    
}

void Frond_End::gnss_update()
{
    geo_converter.Forward(current_gnss.latitude, current_gnss.longitude, current_gnss.altitude,\
                            current_gnss.local_E, current_gnss.local_N, current_gnss.local_U);
    gnss_odom_matrix(0,3) = current_gnss.local_E;
    gnss_odom_matrix(1,3) = current_gnss.local_N;
    gnss_odom_matrix(2,3) = current_gnss.local_U;
    // gnss_odom_matrix.block<3,3>(0,0) = imu_data.get_matrix();
    gnss_odom_matrix *= T_lidar2imu;       
}

void Frond_End::lidar_odom_update() 
{

    // 3.1 构建关键帧地图,将地图传入PCL-ICP
    // 3.2 将当前帧传入 PCL-ICP

    // 4. 得到位姿增量
}

   
