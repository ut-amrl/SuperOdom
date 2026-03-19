//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/config/parameter.h"

#include <unordered_map>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Define color escape codes for ~beautification~
#define RESET "\033[0m"
#define GREEN "\033[32m"   /* Green */
#define BLUE "\033[34m"    /* Blue */
#define BOLD "\033[1m"

std::string IMU_TOPIC;
std::string LASER_TOPIC;
std::string ODOM_TOPIC;
std::string DepthUP_TOPIC;
std::string DepthDown_TOPIC;
std::string ProjectName;

std::string WORLD_FRAME;
std::string WORLD_FRAME_ROT;
std::string SENSOR_FRAME;
std::string SENSOR_FRAME_ROT;
SensorType lidar_sensor;
SensorType imu_sensor;

Eigen::Matrix3d imu_laser_R;
Eigen::Vector3d imu_laser_T;
Transformd T_i_l;
Transformd T_l_i;
Transformd T_b_l;
Transformd T_l_b;
Transformd T_b_i;
Transformd T_i_b;
Transformd T_i_l_working;
Transformd T_l_i_working;
bool USE_BASE_FRAME_ROT_ALIGNMENT = false;

std::string IMU_FRAME;
std::string LIDAR_FRAME;
std::string BASE_FRAME;

float yaw_ratio;

float IMU_ACC_X_LIMIT;
float IMU_ACC_Y_LIMIT;
float IMU_ACC_Z_LIMIT;

bool USE_IMU_ROLL_PITCH;
bool SAVE_PLY;

std::string LIDAR_SENSOR;
std::string IMU_SENSOR;


template <typename T>
T readParam(rclcpp::Node::SharedPtr node, std::string name)
{
    T ans;
    // node->declare_parameter<T>(name);
    if (node->get_parameter(name, ans)) {
        RCLCPP_INFO(node->get_logger(),  "Loaded %s: ", name.c_str());
    }
    else {
        RCLCPP_ERROR(node->get_logger(), "Failed to load %s", name.c_str());
        rclcpp::shutdown();
    }
    return ans;
}

bool readCalibration(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[super_odometry] Reading extrinsics from TF (URDF)...");

    // Frame names already declared in readGlobalparam()
    yaw_ratio = node->get_parameter("yaw_ratio").as_double();

    RCLCPP_INFO(node->get_logger(), "imu_frame: %s, lidar_frame: %s, base_frame: %s",
                IMU_FRAME.c_str(), LIDAR_FRAME.c_str(), BASE_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "yaw_ratio: %f", yaw_ratio);

    // Create TF buffer and listener (spin_thread=true so it works before executor.spin())
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);

    auto lookup_tf = [&](const std::string &target, const std::string &source,
                         geometry_msgs::msg::TransformStamped &out_tf) -> bool {
        if (target == source) {
            out_tf.header.frame_id = target;
            out_tf.child_frame_id = source;
            out_tf.transform.rotation.w = 1.0;
            out_tf.transform.rotation.x = 0.0;
            out_tf.transform.rotation.y = 0.0;
            out_tf.transform.rotation.z = 0.0;
            out_tf.transform.translation.x = 0.0;
            out_tf.transform.translation.y = 0.0;
            out_tf.transform.translation.z = 0.0;
            return true;
        }
        try {
            out_tf = tf_buffer->lookupTransform(
                target, source, tf2::TimePointZero, std::chrono::seconds(10));
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(),
                        "Could not get TF %s -> %s: %s",
                        target.c_str(), source.c_str(), ex.what());
            return false;
        }
    };

    auto tf_to_transformd = [](const geometry_msgs::msg::TransformStamped &tf_msg) -> Transformd {
        const auto &rot = tf_msg.transform.rotation;
        const auto &trans = tf_msg.transform.translation;
        Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);
        return Transformd(
            q.normalized().toRotationMatrix(),
            Eigen::Vector3d(trans.x, trans.y, trans.z));
    };

    // Look up imu_frame -> lidar_frame transform (T_i_l)
    geometry_msgs::msg::TransformStamped tf_imu_lidar;
    if (!lookup_tf(IMU_FRAME, LIDAR_FRAME, tf_imu_lidar)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Could not get TF %s -> %s. Is robot_state_publisher running?",
                     IMU_FRAME.c_str(), LIDAR_FRAME.c_str());
        return false;
    }

    // Convert to Eigen
    const auto &rot = tf_imu_lidar.transform.rotation;
    const auto &trans = tf_imu_lidar.transform.translation;
    Eigen::Quaterniond q(rot.w, rot.x, rot.y, rot.z);
    imu_laser_R = q.normalized().toRotationMatrix();
    imu_laser_T = Eigen::Vector3d(trans.x, trans.y, trans.z);

    T_i_l = Transformd(imu_laser_R, imu_laser_T);
    T_l_i = T_i_l.inverse();

    // Log the result
    double roll, pitch, yaw;
    tf2::Quaternion tf2_q(rot.x, rot.y, rot.z, rot.w);
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), GREEN BOLD "Extrinsic from TF (%s -> %s):" RESET,
                IMU_FRAME.c_str(), LIDAR_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "  roll: %.2f deg, pitch: %.2f deg, yaw: %.2f deg",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "  translation: [%.6f, %.6f, %.6f]",
                imu_laser_T.x(), imu_laser_T.y(), imu_laser_T.z());
    RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "T_i_l Extrinsic:\n" RESET << T_i_l.matrix());
    RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "T_l_i Extrinsic:\n" RESET << T_l_i.matrix());

    // Base-frame alignment transforms (rotation-first sensor normalization)
    geometry_msgs::msg::TransformStamped tf_base_lidar;
    geometry_msgs::msg::TransformStamped tf_base_imu;
    const bool got_base_lidar = lookup_tf(BASE_FRAME, LIDAR_FRAME, tf_base_lidar);
    const bool got_base_imu = lookup_tf(BASE_FRAME, IMU_FRAME, tf_base_imu);

    if (got_base_lidar && got_base_imu) {
        T_b_l = tf_to_transformd(tf_base_lidar);
        T_l_b = T_b_l.inverse();
        T_b_i = tf_to_transformd(tf_base_imu);
        T_i_b = T_b_i.inverse();
        T_i_l_working = Transformd(
            Eigen::Matrix3d::Identity(),
            T_b_l.pos - T_b_i.pos);
        T_l_i_working = T_i_l_working.inverse();
        USE_BASE_FRAME_ROT_ALIGNMENT = true;

        RCLCPP_INFO(node->get_logger(),
                    GREEN BOLD "Base-frame rotation alignment enabled using TF (%s)." RESET,
                    BASE_FRAME.c_str());
        RCLCPP_INFO_STREAM(node->get_logger(),
                           GREEN BOLD "Working rectified T_i_l:\n" RESET
                               << T_i_l_working.matrix());
    } else {
        T_b_l = Transformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        T_l_b = T_b_l.inverse();
        T_b_i = Transformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        T_i_b = T_b_i.inverse();
        T_i_l_working = T_i_l;
        T_l_i_working = T_l_i;
        USE_BASE_FRAME_ROT_ALIGNMENT = false;
        RCLCPP_WARN(node->get_logger(),
                    "Base-frame TF lookup incomplete. Running without base-frame rotation alignment.");
    }

    return true;
}

bool readGlobalparam(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter<std::string>("imu_topic","imu/data");
    node->declare_parameter<std::string>("laser_topic","velodyne_points");
    node->declare_parameter<std::string>("odom_topic","integrated_to_init");
    node->declare_parameter<std::string>("depthup_topic","/rs_up/depth/cloud_filtered");
    node->declare_parameter<std::string>("depthdown_topic","/rs_down/depth/cloud_filtered");
    node->declare_parameter<std::string>("world_frame", "sensor_init");
    node->declare_parameter<std::string>("world_frame_rot", "sensor_init_rot");
    node->declare_parameter<std::string>("sensor_frame", "sensor");
    node->declare_parameter<std::string>("sensor_frame_rot", "sensor_rot");
    node->declare_parameter<std::string>("PROJECT_NAME", "");
    node->declare_parameter<std::string>("lidar_sensor", "livox");
    node->declare_parameter<std::string>("imu_sensor", "");
    node->declare_parameter<double>("imu_acc_x_limit", 0.5);
    node->declare_parameter<double>("imu_acc_y_limit", 0.2);
    node->declare_parameter<double>("imu_acc_z_limit", 0.4);
    node->declare_parameter<bool>("save_ply", false);
    node->declare_parameter<bool>("use_imu_roll_pitch", false);
    node->declare_parameter<std::string>("imu_frame", "imu_link");
    node->declare_parameter<std::string>("lidar_frame", "lidar_link");
    node->declare_parameter<std::string>("base_frame", "base_link");
    node->declare_parameter<double>("yaw_ratio", 0.0);

    
    LASER_TOPIC = node->get_parameter("laser_topic").as_string();
    IMU_TOPIC = node->get_parameter("imu_topic").as_string();
    ODOM_TOPIC = node->get_parameter("odom_topic").as_string();
    DepthUP_TOPIC = node->get_parameter("depthup_topic").as_string();
    DepthDown_TOPIC = node->get_parameter("depthdown_topic").as_string();
    WORLD_FRAME = node->get_parameter("world_frame").as_string();
    WORLD_FRAME_ROT = node->get_parameter("world_frame_rot").as_string();
    SENSOR_FRAME = node->get_parameter("sensor_frame").as_string();
    SENSOR_FRAME_ROT = node->get_parameter("sensor_frame_rot").as_string();
    ProjectName = node->get_parameter("PROJECT_NAME").as_string();
    LIDAR_SENSOR = node->get_parameter("lidar_sensor").as_string();
    IMU_SENSOR = node->get_parameter("imu_sensor").as_string();
    if (IMU_SENSOR.empty()) {
        IMU_SENSOR = LIDAR_SENSOR;
    }
    USE_IMU_ROLL_PITCH = node->get_parameter("use_imu_roll_pitch").as_bool();
    SAVE_PLY = node->get_parameter("save_ply").as_bool();
    IMU_ACC_X_LIMIT = node->get_parameter("imu_acc_x_limit").as_double();
    IMU_ACC_Y_LIMIT = node->get_parameter("imu_acc_y_limit").as_double();
    IMU_ACC_Z_LIMIT = node->get_parameter("imu_acc_z_limit").as_double();
    IMU_FRAME = node->get_parameter("imu_frame").as_string();
    LIDAR_FRAME = node->get_parameter("lidar_frame").as_string();
    BASE_FRAME = node->get_parameter("base_frame").as_string();
    //check whether sensor is support 
    const std::unordered_map<std::string, SensorType> sensorTypeMap = {
        {"velodyne", SensorType::VELODYNE},
        {"ouster", SensorType::OUSTER},
        {"livox", SensorType::LIVOX},
        {"vectornav_enu", SensorType::VECTORNAV_ENU}
    };

    if (sensorTypeMap.find(LIDAR_SENSOR) == sensorTypeMap.end()) {
        RCLCPP_ERROR(node->get_logger(), "Unsupported sensor type: %s", LIDAR_SENSOR.c_str());
        return false;
    }
    if (sensorTypeMap.find(IMU_SENSOR) == sensorTypeMap.end()) {
        RCLCPP_ERROR(node->get_logger(), "Unsupported IMU sensor type: %s", IMU_SENSOR.c_str());
        return false;
    }

    lidar_sensor = sensorTypeMap.at(LIDAR_SENSOR);
    imu_sensor = sensorTypeMap.at(IMU_SENSOR);
    
    RCLCPP_INFO(node->get_logger(), "LASER_TOPIC %s", LASER_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_TOPIC %s", IMU_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "ODOM_TOPIC %s", ODOM_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "DepthUP_TOPIC %s", DepthUP_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "DepthDown_TOPIC %s", DepthDown_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "WORLD_FRAME %s", WORLD_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "WORLD_FRAME_ROT %s", WORLD_FRAME_ROT.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR_FRAME %s", SENSOR_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR_FRAME_ROT %s", SENSOR_FRAME_ROT.c_str());
    RCLCPP_INFO(node->get_logger(), "ProjectName %s", ProjectName.c_str());
    RCLCPP_INFO(node->get_logger(), "LIDAR_SENSOR %s", LIDAR_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_SENSOR %s", IMU_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "USE_IMU_ROLL_PITCH %d", USE_IMU_ROLL_PITCH);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);

    return true;
}
