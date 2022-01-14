#ifndef _SENSOR_TYPE_HPP_
#define _SENSOR_TYPE_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CloudData {
// public:
// using POINT = pcl::PointXYZ;
// using CLOUD = pcl::PointCloud<POINT>;
// using CLOUD_PTR = CLOUD::Ptr;
public:
    CloudData() : cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()) {}

    double time = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
};

class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    // int service = 0;

  private:
    // static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

//   public: 
//     void InitOriginPosition();
//     void UpdateXYZ();
};

class IMUData {
public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
    
    struct Orientation {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double w = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
  
public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3d get_matrix() {
      Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
      return q.matrix();
    }
};


#endif
