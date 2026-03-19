#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>

namespace super_odometry::utils {

inline Eigen::Vector3d rfu_to_flu(const Eigen::Vector3d& v_rfu) {
    return Eigen::Vector3d(v_rfu.y(), -v_rfu.x(), v_rfu.z());
}

inline void imu_rfu_to_flu(sensor_msgs::msg::Imu& imu_msg) {
    Eigen::Vector3d acc_rfu(imu_msg.linear_acceleration.x,
                            imu_msg.linear_acceleration.y,
                            imu_msg.linear_acceleration.z);
    Eigen::Vector3d gyr_rfu(imu_msg.angular_velocity.x,
                            imu_msg.angular_velocity.y,
                            imu_msg.angular_velocity.z);

    const Eigen::Vector3d acc_flu = rfu_to_flu(acc_rfu);
    const Eigen::Vector3d gyr_flu = rfu_to_flu(gyr_rfu);

    imu_msg.linear_acceleration.x = acc_flu.x();
    imu_msg.linear_acceleration.y = acc_flu.y();
    imu_msg.linear_acceleration.z = acc_flu.z();

    imu_msg.angular_velocity.x = gyr_flu.x();
    imu_msg.angular_velocity.y = gyr_flu.y();
    imu_msg.angular_velocity.z = gyr_flu.z();

    Eigen::Quaterniond q(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y,
                         imu_msg.orientation.z);
    if (q.norm() > 1e-12) {
        q.normalize();
        constexpr double kHalfPi = 1.57079632679489661923; // pi/2
        const Eigen::Quaterniond q_rfu_flu(Eigen::AngleAxisd(-kHalfPi, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q_new = q * q_rfu_flu;
        q_new.normalize();

        imu_msg.orientation.x = q_new.x();
        imu_msg.orientation.y = q_new.y();
        imu_msg.orientation.z = q_new.z();
        imu_msg.orientation.w = q_new.w();
    }
}

inline void rotate_imu_to_frame(sensor_msgs::msg::Imu& imu_msg, const Eigen::Matrix3d& rotation) {
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);
    Eigen::Vector3d gyr(imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z);

    acc = rotation * acc;
    gyr = rotation * gyr;

    imu_msg.linear_acceleration.x = acc.x();
    imu_msg.linear_acceleration.y = acc.y();
    imu_msg.linear_acceleration.z = acc.z();
    imu_msg.angular_velocity.x = gyr.x();
    imu_msg.angular_velocity.y = gyr.y();
    imu_msg.angular_velocity.z = gyr.z();

    Eigen::Quaterniond q(imu_msg.orientation.w, imu_msg.orientation.x,
                         imu_msg.orientation.y, imu_msg.orientation.z);
    if (q.norm() > 1e-12) {
        q.normalize();
        Eigen::Quaterniond q_rot(rotation);
        Eigen::Quaterniond q_new = q * q_rot.conjugate();
        q_new.normalize();
        imu_msg.orientation.w = q_new.w();
        imu_msg.orientation.x = q_new.x();
        imu_msg.orientation.y = q_new.y();
        imu_msg.orientation.z = q_new.z();
    }
}

} // namespace super_odometry::utils
