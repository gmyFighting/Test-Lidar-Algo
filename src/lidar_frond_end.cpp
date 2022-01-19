#include "Test-Lidar-Algo/lidar_frond_end.hpp"
#include <pcl/common/transforms.h>

Frond_End::Frond_End(ros::NodeHandle &nh)
{
    cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "points_raw", 100000);
    imu_sub_ptr = std::make_shared<ImuSubscriber>(nh, "imu_correct", 100000);
    gnss_sub_ptr = std::make_shared<GpsSubscriber>(nh, "gps/fix", 100000);
    key_frame_distance = 2.0;
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

bool Frond_End::data_is_valid()
{
    double dt = current_cloud.time - current_imu.time;
    if (dt < -0.05) {
        cloud_sub_ptr->buf_dq_.pop_front();
        return false;
    }

    if (dt > 0.05) {
        imu_sub_ptr->buf_dq_.pop_front();
        gnss_sub_ptr->buf_dq_.pop_front();
        return false;
    }

    cloud_sub_ptr->buf_dq_.pop_front();
    imu_sub_ptr->buf_dq_.pop_front();
    gnss_sub_ptr->buf_dq_.pop_front();

    return true;    
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
    // 1. 统一gnss/lidar odom初值
    if (!lidar_odom_inited) {
        lidar_odom_matrix = gnss_odom_matrix;
        current_key_frame.pose = gnss_odom_matrix;
        // init_matrix = gnss_odom_matrix；
        lidar_odom_inited = true;
    }

    current_key_frame.cloud_data.time = current_cloud.time;
    std::vector<int> tmp_indices;
    // 2. 去除点云中NaN并存入当前关键帧中
    pcl::removeNaNFromPointCloud(*current_cloud.cloud_ptr, *current_key_frame.cloud_data.cloud_ptr, tmp_indices);

    // 3. VoxelGrid滤波器:使用体素化网格的方法实现下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);// 未释放，改成make_shared
    voxel_filter.filter(current_key_frame.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4d last_pose = gnss_odom_matrix;
    static Eigen::Matrix4d predict_pose = gnss_odom_matrix;

    // 4. 地图初始化
    if (local_map_frames.size() == 0) {
        // current_key_frame.pose = init_matrix;
        update_map(current_key_frame);
        return;
    }

    // 5. 输入参数：待匹配点云, 预测转换矩阵(迭代初值), 变换后的点云(没用上), 待匹配点云到目标点云的变换矩阵
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_ptr;
    registration_method.scan_match(filtered_cloud_ptr, predict_pose, tmp_cloud_ptr, current_key_frame.pose);
    lidar_odom_matrix = current_key_frame.pose;

    // 6. 更新相邻两帧的相对运动
    Eigen::Matrix4d step_pose = last_pose.inverse() * current_key_frame.pose;
    predict_pose = current_key_frame.pose * step_pose;
    last_pose = current_key_frame.pose;

    // 7. 满足一定距离则将当前帧加入地图
    if (key_frame_is_valid()) {
        update_map(current_key_frame);
        last_key_frame_pose = current_key_frame.pose;
    }
}

// update local && global map
bool Frond_End::update_map(const Frame &new_key_frame)
{
    // // 把关键帧点云存储到硬盘里，节省内存
    // std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    // pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    // Frame key_frame = new_key_frame;
    // // 这一步的目的是为了把关键帧的点云保存下来
    // // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    // key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    // CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // // 更新局部地图
    // local_map_frames_.push_back(key_frame);
    // while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    //     local_map_frames_.pop_front();
    // }
    // local_map_ptr_.reset(new CloudData::CLOUD());
    // for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    //     pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
    //                              *transformed_cloud_ptr, 
    //                              local_map_frames_.at(i).pose);
    //     *local_map_ptr_ += *transformed_cloud_ptr;
    // }
    // has_new_local_map_ = true;

    // // 更新ndt匹配的目标点云
    // // 关键帧数量还比较少时不滤波，太稀疏影响匹配效果
    // if (local_map_frames.size() < 10) {
    //     registration_method.set_input_target(local_map_ptr_);// 设置目标点云(输入点云进行仿射变换得到目标点云)
    // } else {
    //     CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
    //     local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
    //     registration_method.set_input_target(filtered_local_map_ptr);
    // }

    // // 保存所有关键帧信息在容器里
    // // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    // key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    // global_map_frames_.push_back(key_frame);

    return true;
}

   
