#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>

namespace super_odometry::utils {

inline void rotate_imu_to_frame(sensor_msgs::msg::Imu& imu_msg, const Eigen::Quaterniond& q_sensor_to_target) {
    Eigen::Quaterniond q_rot = q_sensor_to_target;
    if (q_rot.norm() <= 1e-12) {
        return;
    }
    q_rot.normalize();

    const Eigen::Matrix3d R = q_rot.toRotationMatrix();

    Eigen::Vector3d acc_sensor(imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z);
    Eigen::Vector3d gyr_sensor(imu_msg.angular_velocity.x,
                               imu_msg.angular_velocity.y,
                               imu_msg.angular_velocity.z);

    const Eigen::Vector3d acc_target = R * acc_sensor;
    const Eigen::Vector3d gyr_target = R * gyr_sensor;

    imu_msg.linear_acceleration.x = acc_target.x();
    imu_msg.linear_acceleration.y = acc_target.y();
    imu_msg.linear_acceleration.z = acc_target.z();

    imu_msg.angular_velocity.x = gyr_target.x();
    imu_msg.angular_velocity.y = gyr_target.y();
    imu_msg.angular_velocity.z = gyr_target.z();

    Eigen::Quaterniond q_body(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
    if (q_body.norm() > 1e-12) {
        q_body.normalize();
        Eigen::Quaterniond q_new = q_body * q_rot;
        q_new.normalize();

        imu_msg.orientation.x = q_new.x();
        imu_msg.orientation.y = q_new.y();
        imu_msg.orientation.z = q_new.z();
        imu_msg.orientation.w = q_new.w();
    }
}

} // namespace super_odometry::utils
