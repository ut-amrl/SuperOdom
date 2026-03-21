#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

namespace super_odometry::utils {

/// Rotate IMU data (acceleration, gyroscope, orientation) from the sensor's
/// native frame into a target frame using the provided rotation matrix.
///
/// The rotation matrix R should satisfy:  v_target = R * v_sensor
///
/// For orientation quaternion q_sensor (sensor-to-world), the result is:
///   q_target = q_sensor * R^T
/// because R^T converts target-frame axes back to sensor axes.
inline void rotate_imu_to_frame(sensor_msgs::msg::Imu& imu_msg,
                                 const Eigen::Matrix3d& rotation) {
    // Rotate linear acceleration
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);
    acc = rotation * acc;
    imu_msg.linear_acceleration.x = acc.x();
    imu_msg.linear_acceleration.y = acc.y();
    imu_msg.linear_acceleration.z = acc.z();

    // Rotate angular velocity
    Eigen::Vector3d gyr(imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z);
    gyr = rotation * gyr;
    imu_msg.angular_velocity.x = gyr.x();
    imu_msg.angular_velocity.y = gyr.y();
    imu_msg.angular_velocity.z = gyr.z();

    // Rotate orientation quaternion
    // q_sensor represents the rotation from sensor frame to world frame.
    // We want q_target: rotation from target frame to world frame.
    // q_target = q_sensor * R^T  (R^T maps target axes to sensor axes)
    Eigen::Quaterniond q(imu_msg.orientation.w,
                         imu_msg.orientation.x,
                         imu_msg.orientation.y,
                         imu_msg.orientation.z);
    if (q.norm() > 1e-12) {
        q.normalize();
        Eigen::Quaterniond q_rot(rotation);
        Eigen::Quaterniond q_new = q * q_rot.conjugate();
        q_new.normalize();

        imu_msg.orientation.x = q_new.x();
        imu_msg.orientation.y = q_new.y();
        imu_msg.orientation.z = q_new.z();
        imu_msg.orientation.w = q_new.w();
    }
}


/// Rotate a point cloud from the sensor frame into a target frame using the
/// provided rotation matrix.
template <typename PointT>
inline void rotate_pointcloud_to_frame(pcl::PointCloud<PointT>& cloud,
                                       const Eigen::Matrix3d& rotation) {
    for (auto& point : cloud.points) {
        Eigen::Vector3d xyz(point.x, point.y, point.z);
        xyz = rotation * xyz;
        point.x = static_cast<float>(xyz.x());
        point.y = static_cast<float>(xyz.y());
        point.z = static_cast<float>(xyz.z());
    }
}

} // namespace super_odometry::utils
