//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/config/parameter.h"

#include <unordered_map>

// Define color escape codes for ~beautification~
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"
#define ITALIC "\033[3m"

std::string IMU_TOPIC;
std::string LASER_TOPIC;
std::string ODOM_TOPIC;
std::string DepthUP_TOPIC;
std::string DepthDown_TOPIC;
std::string ProjectName;

std::string WORLD_FRAME;
std::string WORLD_FRAME_ROT;
std::string LIDAR_FRAME_RECTIFIED;
std::string BASE_LINK_FRAME;
std::string IMU_FRAME_NAME;
std::string LIDAR_FRAME_NAME;
std::string IMU_FRAME_RECTIFIED;
bool USE_TF_ALIGNMENT = false;
Eigen::Quaterniond Q_IMU_TO_BASE = Eigen::Quaterniond::Identity();
Eigen::Quaterniond Q_LIDAR_TO_BASE = Eigen::Quaterniond::Identity();
Eigen::Vector3d T_BASE_IMU = Eigen::Vector3d::Zero();
Eigen::Vector3d T_BASE_LIDAR = Eigen::Vector3d::Zero();
SensorType lidar_sensor;
SensorType imu_sensor;

int PROVIDE_IMU_LASER_EXTRINSIC;

Eigen::Matrix3d imu_laser_R;

Eigen::Vector3d imu_laser_T;

Eigen::Vector3d imu_laser_offset;

Eigen::Matrix3d cam_laser_R;

Eigen::Vector3d cam_laser_T;

Eigen::Matrix3d imu_camera_R;

Eigen::Vector3d imu_camera_T;

Transformd Tcam_lidar;

Transformd T_i_c;

Transformd T_i_l;

Transformd T_l_i;

Transformd T_ouster_sensor;

Eigen::Matrix3d ouster_sensor_R;

Eigen::Vector3d ouster_sensor_T;

float lidar_imu_offset_roll;

float up_realsense_roll;

float up_realsense_pitch;

float up_realsense_yaw;

float up_realsense_x;

float up_realsense_y;

float up_realsense_z;

float down_realsense_roll;

float down_realsense_pitch;

float down_realsense_yaw;

float down_realsense_x;

float down_realsense_y;

float down_realsense_z;

float yaw_ratio;

float IMU_ACC_X_LIMIT;

float IMU_ACC_Y_LIMIT;

float IMU_ACC_Z_LIMIT;

double IMU_MIN_DT;
bool USE_IMU_ROLL_PITCH;
bool LOG_IMU_ROLL_PITCH_ICP = false;

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


template <typename T>
T declareOrGetParam(rclcpp::Node::SharedPtr node, const std::string &name, const T &default_value)
{
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }

    T value = default_value;
    if (!node->get_parameter(name, value)) {
        value = default_value;
    }
    return value;
}



bool readCalibration(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[super_odometry] read parameter");

    PROVIDE_IMU_LASER_EXTRINSIC = 1;
    USE_TF_ALIGNMENT = declareOrGetParam<bool>(node, "use_tf_alignment", false);
    BASE_LINK_FRAME = declareOrGetParam<std::string>(node, "base_link_frame", "base_link");
    IMU_FRAME_NAME = declareOrGetParam<std::string>(node, "imu_frame", "imu_link");
    LIDAR_FRAME_NAME = declareOrGetParam<std::string>(node, "lidar_frame", "lidar_link");
    IMU_FRAME_RECTIFIED = declareOrGetParam<std::string>(node, "imu_frame_rectified", "imu_frame_rectified");
    const std::vector<double> imu_laser_rotation_offset_deg =
        declareOrGetParam<std::vector<double>>(node, "imu_laser_rotation_offset_deg", {0.0, 0.0, 0.0});

    up_realsense_roll = declareOrGetParam<double>(node, "up_realsense_roll", 0.0);
    up_realsense_pitch = declareOrGetParam<double>(node, "up_realsense_pitch", 0.0);
    up_realsense_yaw = declareOrGetParam<double>(node, "up_realsense_yaw", 0.0);
    up_realsense_x = declareOrGetParam<double>(node, "up_realsense_x", 0.0);
    up_realsense_y = declareOrGetParam<double>(node, "up_realsense_y", 0.0);
    up_realsense_z = declareOrGetParam<double>(node, "up_realsense_z", 0.0);

    down_realsense_roll = declareOrGetParam<double>(node, "down_realsense_roll", 0.0);
    down_realsense_pitch = declareOrGetParam<double>(node, "down_realsense_pitch", 0.0);
    down_realsense_yaw = declareOrGetParam<double>(node, "down_realsense_yaw", 0.0);
    down_realsense_x = declareOrGetParam<double>(node, "down_realsense_x", 0.0);
    down_realsense_y = declareOrGetParam<double>(node, "down_realsense_y", 0.0);
    down_realsense_z = declareOrGetParam<double>(node, "down_realsense_z", 0.0);
    yaw_ratio = declareOrGetParam<double>(node, "yaw_ratio", 0.0);

    RCLCPP_INFO(node->get_logger(), "PROVIDE_IMU_LASER_EXTRINSIC: %d", PROVIDE_IMU_LASER_EXTRINSIC);
    RCLCPP_INFO(node->get_logger(), "up realsense extrinsic to velodyne (RPYXYZ): %f, %f, %f, %f, %f, %f",
                up_realsense_roll,
                up_realsense_pitch,
                up_realsense_yaw,
                up_realsense_x,
                up_realsense_y,
                up_realsense_z);

    RCLCPP_INFO(node->get_logger(), "down realsense extrinsic to velodyne (RPYXYZ): %f, %f, %f, %f, %f, %f",
                down_realsense_roll,
                down_realsense_pitch,
                down_realsense_yaw,
                down_realsense_x,
                down_realsense_y,
                down_realsense_z);

    RCLCPP_INFO(node->get_logger(), "yaw ratio: %f", yaw_ratio);

    if (imu_laser_rotation_offset_deg.size() != 3) {
        RCLCPP_ERROR(node->get_logger(), "imu_laser_rotation_offset_deg must contain exactly 3 values.");
        return false;
    }

    imu_laser_offset = Eigen::Vector3d(
        imu_laser_rotation_offset_deg[0],
        imu_laser_rotation_offset_deg[1],
        imu_laser_rotation_offset_deg[2]);

    if (!USE_TF_ALIGNMENT) {
        RCLCPP_ERROR(
            node->get_logger(),
            "[super_odometry] This branch requires use_tf_alignment=true. Calibration-file fallback has been removed.");
        return false;
    }

    imu_laser_R = Eigen::Matrix3d::Identity();
    imu_laser_T = Eigen::Vector3d::Zero();

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

    bool got_imu_tf = false;
    bool got_lidar_tf = false;
    Eigen::Vector3d t_base_imu = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_base_lidar = Eigen::Vector3d::Zero();

    int wait_log_counter = 0;
    while (rclcpp::ok() && (!got_imu_tf || !got_lidar_tf)) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node);

        if (!got_imu_tf) {
            try {
                auto tf_base_imu = tf_buffer->lookupTransform(BASE_LINK_FRAME, IMU_FRAME_NAME, tf2::TimePointZero);
                const auto &r = tf_base_imu.transform.rotation;
                const auto &t = tf_base_imu.transform.translation;
                Q_IMU_TO_BASE = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
                t_base_imu = Eigen::Vector3d(t.x, t.y, t.z);
                T_BASE_IMU = t_base_imu;
                got_imu_tf = true;
                RCLCPP_INFO(node->get_logger(), "[super_odometry] Received TF %s -> %s", BASE_LINK_FRAME.c_str(), IMU_FRAME_NAME.c_str());
            } catch (const tf2::TransformException &) {
            }
        }

        if (!got_lidar_tf) {
            try {
                auto tf_base_lidar = tf_buffer->lookupTransform(BASE_LINK_FRAME, LIDAR_FRAME_NAME, tf2::TimePointZero);
                const auto &r = tf_base_lidar.transform.rotation;
                const auto &t = tf_base_lidar.transform.translation;
                Q_LIDAR_TO_BASE = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized();
                t_base_lidar = Eigen::Vector3d(t.x, t.y, t.z);
                T_BASE_LIDAR = t_base_lidar;
                got_lidar_tf = true;
                RCLCPP_INFO(node->get_logger(), "[super_odometry] Received TF %s -> %s", BASE_LINK_FRAME.c_str(), LIDAR_FRAME_NAME.c_str());
            } catch (const tf2::TransformException &) {
            }
        }

        ++wait_log_counter;
        if ((!got_imu_tf || !got_lidar_tf) && wait_log_counter >= 10) {
            wait_log_counter = 0;
            RCLCPP_WARN(
                node->get_logger(),
                "[super_odometry] Waiting for TFs %s -> %s and %s -> %s before startup continues...",
                BASE_LINK_FRAME.c_str(),
                IMU_FRAME_NAME.c_str(),
                BASE_LINK_FRAME.c_str(),
                LIDAR_FRAME_NAME.c_str());
        }
    }

    if (!rclcpp::ok()) {
        return false;
    }

    // Match the earlier utamrl_alphatruck TF-derived translation path by
    // taking the IMU/LiDAR origin delta directly from the TF frame origins.
    imu_laser_T = t_base_imu - t_base_lidar;
    RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "[super_odometry] TF-derived imu_laser_T: " RESET << imu_laser_T.transpose());

    T_i_l = Transformd(imu_laser_R, imu_laser_T);
    T_l_i = T_i_l.inverse();

    double roll, pitch, yaw;
    tf2::Quaternion orientation_pre(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(), T_i_l.rot.w());
    tf2::Matrix3x3(orientation_pre).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), BLUE"\n previous roll: %f previous pitch: %f previous yaw: %f" RESET,
                roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

    tf2::Quaternion IMU_LASER_R_offset;
    IMU_LASER_R_offset.setRPY(imu_laser_offset[0] * M_PI / 180,
                              imu_laser_offset[1] * M_PI / 180,
                              imu_laser_offset[2] * M_PI / 180);

    tf2::Quaternion IMU_LASER_R(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(), T_i_l.rot.w());
    tf2::Quaternion IMU_LASER = IMU_LASER_R_offset * IMU_LASER_R;
    Eigen::Quaterniond imu_laser_rot(IMU_LASER.w(), IMU_LASER.x(), IMU_LASER.y(), IMU_LASER.z());

    T_i_l.rot = imu_laser_rot;
    T_l_i = T_i_l.inverse();
    imu_laser_R = T_i_l.rot.toRotationMatrix();

    RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "T_i_l Extrinsic : \n" << T_i_l.matrix());
    RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "T_l_i Extrinsic : \n" << T_l_i.matrix());

    double updated_roll, updated_pitch, updated_yaw;
    tf2::Quaternion orientation_curr(IMU_LASER.x(), IMU_LASER.y(), IMU_LASER.z(), IMU_LASER.w());
    tf2::Matrix3x3(orientation_curr).getRPY(updated_roll, updated_pitch, updated_yaw);

    RCLCPP_INFO(node->get_logger(), GREEN BOLD"\n updated roll: %f updated pitch: %f updated yaw: %f" RESET,
                updated_roll * 180 / M_PI, updated_pitch * 180 / M_PI, updated_yaw * 180 / M_PI);

    ouster_sensor_R << -1, 0,  0,
                    0, -1, 0,
                    0,  0,  1;

    ouster_sensor_T << 0, 0, 0.036180;

    T_ouster_sensor = Transformd(ouster_sensor_R, ouster_sensor_T);

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
    node->declare_parameter<std::string>("lidar_frame_rectified", "lidar_frame_rectified");
    node->declare_parameter<std::string>("imu_frame_rectified", "imu_frame_rectified");
    node->declare_parameter<std::string>("PROJECT_NAME", "");
    node->declare_parameter<std::string>("lidar_sensor", "livox");
    node->declare_parameter<std::string>("imu_sensor", "");
    node->declare_parameter<double>("imu_acc_x_limit", 0.5);
    node->declare_parameter<double>("imu_acc_y_limit", 0.2);
    node->declare_parameter<double>("imu_acc_z_limit", 0.4);
    node->declare_parameter<bool>("save_ply", false);
    node->declare_parameter<bool>("use_imu_roll_pitch", false);
    node->declare_parameter<bool>("log_imu_roll_pitch_icp", false);
    node->declare_parameter<double>("imu_min_dt", 0.004);

    
    LASER_TOPIC = node->get_parameter("laser_topic").as_string();
    IMU_TOPIC = node->get_parameter("imu_topic").as_string();
    ODOM_TOPIC = node->get_parameter("odom_topic").as_string();
    DepthUP_TOPIC = node->get_parameter("depthup_topic").as_string();
    DepthDown_TOPIC = node->get_parameter("depthdown_topic").as_string();
    WORLD_FRAME = node->get_parameter("world_frame").as_string();
    WORLD_FRAME_ROT = node->get_parameter("world_frame_rot").as_string();
    LIDAR_FRAME_RECTIFIED = node->get_parameter("lidar_frame_rectified").as_string();
    IMU_FRAME_RECTIFIED = node->get_parameter("imu_frame_rectified").as_string();
    ProjectName = node->get_parameter("PROJECT_NAME").as_string();
    LIDAR_SENSOR = node->get_parameter("lidar_sensor").as_string();
    IMU_SENSOR = node->get_parameter("imu_sensor").as_string();
    if (IMU_SENSOR.empty()) {
        IMU_SENSOR = LIDAR_SENSOR;
    }
    USE_IMU_ROLL_PITCH = node->get_parameter("use_imu_roll_pitch").as_bool();
    LOG_IMU_ROLL_PITCH_ICP = node->get_parameter("log_imu_roll_pitch_icp").as_bool();
    IMU_MIN_DT = node->get_parameter("imu_min_dt").as_double();
    SAVE_PLY = node->get_parameter("save_ply").as_bool();
    IMU_ACC_X_LIMIT = node->get_parameter("imu_acc_x_limit").as_double();
    IMU_ACC_Y_LIMIT = node->get_parameter("imu_acc_y_limit").as_double();
    IMU_ACC_Z_LIMIT = node->get_parameter("imu_acc_z_limit").as_double();
    //check whether sensor is support 
    const std::unordered_map<std::string, SensorType> sensorTypeMap = {
        {"velodyne", SensorType::VELODYNE},
        {"ouster", SensorType::OUSTER},
        {"livox", SensorType::LIVOX},
        {"vectornav", SensorType::VECTORNAV}
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
    RCLCPP_INFO(node->get_logger(), "LIDAR_FRAME_RECTIFIED %s", LIDAR_FRAME_RECTIFIED.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_FRAME_RECTIFIED %s", IMU_FRAME_RECTIFIED.c_str());
    RCLCPP_INFO(node->get_logger(), "ProjectName %s", ProjectName.c_str());
    RCLCPP_INFO(node->get_logger(), "LIDAR_SENSOR %s", LIDAR_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_SENSOR %s", IMU_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "USE_IMU_ROLL_PITCH %d", USE_IMU_ROLL_PITCH);
    RCLCPP_INFO(node->get_logger(), "LOG_IMU_ROLL_PITCH_ICP %d", LOG_IMU_ROLL_PITCH_ICP);
    RCLCPP_INFO(node->get_logger(), "IMU_MIN_DT %f", IMU_MIN_DT);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);

    return true;
}
