#ifndef _CLOUD_REGISTRATION_HPP_
#define _CLOUD_REGISTRATION_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

class CloudRegistrationInterface {
  public:
    CloudRegistrationInterface() = default;
    // 基类的析构需设置为虚函数
    virtual ~CloudRegistrationInterface() = default;

    virtual bool set_input_target(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) = 0;
    virtual bool scan_match(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source, 
                            const Eigen::Matrix4d &predict_pose, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                              Eigen::Matrix4d &result_pose) = 0;
};

class PCL_ICP : public CloudRegistrationInterface {
public:
    PCL_ICP();

    bool set_input_target(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target) override;
    bool scan_match(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source, 
                            const Eigen::Matrix4d &predict_pose, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &result_cloud_ptr,
                              Eigen::Matrix4d &result_pose) override;
private:
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_ptr;
};




#endif
