//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/config/parameter.h"

#include <unordered_map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
std::string SENSOR_FRAME;
std::string SENSOR_FRAME_ROT;
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

bool USE_IMU_ROLL_PITCH;

bool SAVE_PLY;

std::string LIDAR_SENSOR;
std::string IMU_SENSOR;

// --- TF-based frame alignment globals ---
bool USE_TF_ALIGNMENT = false;
std::string BASE_FRAME;
std::string IMU_FRAME;
std::string LIDAR_FRAME;

Eigen::Matrix3d R_base_imu = Eigen::Matrix3d::Identity();
Eigen::Matrix3d R_base_lidar = Eigen::Matrix3d::Identity();
Eigen::Vector3d t_base_imu = Eigen::Vector3d::Zero();
Eigen::Vector3d t_base_lidar = Eigen::Vector3d::Zero();

Transformd T_i_l_working;


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
    // When using TF alignment, skip calibration file entirely
    if (USE_TF_ALIGNMENT) {
        RCLCPP_INFO(node->get_logger(), GREEN BOLD "[super_odometry] Using TF-based alignment — skipping calibration file" RESET);

        // Set extrinsics from TF-derived values
        // T_i_l_working has identity rotation (both frames aligned to base)
        // and translation = lidar_pos - imu_pos in base frame
        imu_laser_R = Eigen::Matrix3d::Identity();
        imu_laser_T = t_base_lidar - t_base_imu;
        imu_laser_offset = Eigen::Vector3d::Zero();

        T_i_l = T_i_l_working;
        T_l_i = T_i_l.inverse();

        PROVIDE_IMU_LASER_EXTRINSIC = 1;
        yaw_ratio = 0.0;

        // Zero out realsense params
        up_realsense_roll = up_realsense_pitch = up_realsense_yaw = 0.0f;
        up_realsense_x = up_realsense_y = up_realsense_z = 0.0f;
        down_realsense_roll = down_realsense_pitch = down_realsense_yaw = 0.0f;
        down_realsense_x = down_realsense_y = down_realsense_z = 0.0f;

        RCLCPP_INFO_STREAM(node->get_logger(), GREEN BOLD "T_i_l_working (TF-based): \n" RESET << T_i_l_working.matrix());

        ouster_sensor_R << -1, 0,  0,
                        0, -1, 0,
                        0,  0,  1;
        ouster_sensor_T << 0, 0, 0.036180;
        T_ouster_sensor = Transformd(ouster_sensor_R, ouster_sensor_T);

        return true;
    }

    // --- Legacy calibration file path ---
    RCLCPP_INFO(node->get_logger(), "[super_odometry] read parameter (calibration file)");
    std::string calib_file;
    calib_file = node->declare_parameter("calibration_file", std::string(""));
    RCLCPP_INFO(node->get_logger(), "[super_odometry] calib_file: %s", calib_file.c_str());
    cv::FileStorage fsSettings(calib_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return false;
    }
    PROVIDE_IMU_LASER_EXTRINSIC = node->declare_parameter("provide_imu_laser_extrinsic", true);
    RCLCPP_INFO(node->get_logger(), "PROVIDE_IMU_LASER_EXTRINSIC: %d", PROVIDE_IMU_LASER_EXTRINSIC);

    up_realsense_roll = fsSettings["up_realsense_roll"];
    up_realsense_pitch = fsSettings["up_realsense_pitch"];
    up_realsense_yaw = fsSettings["up_realsense_yaw"];
    up_realsense_x = fsSettings["up_realsense_x"];
    up_realsense_y = fsSettings["up_realsense_y"];
    up_realsense_z = fsSettings["up_realsense_z"];

    down_realsense_roll = fsSettings["down_realsense_roll"];
    down_realsense_pitch = fsSettings["down_realsense_pitch"];
    down_realsense_yaw = fsSettings["down_realsense_yaw"];
    down_realsense_x = fsSettings["down_realsense_x"];
    down_realsense_y = fsSettings["down_realsense_y"];
    down_realsense_z = fsSettings["down_realsense_z"];
    
    yaw_ratio=fsSettings["yaw_ratio"];

    RCLCPP_INFO(node->get_logger(), "yaw ratio: %f", yaw_ratio);
    
    if (PROVIDE_IMU_LASER_EXTRINSIC)
    {
        cv::Mat cv_R, cv_T;
        cv::Mat imu_laser_rotation_offset;
        fsSettings["imu_laser_rotation_offset"] >> imu_laser_rotation_offset;
        fsSettings["extrinsicRotation_imu_laser"] >> cv_R;
        fsSettings["extrinsicTranslation_imu_laser"] >> cv_T;
        cv::cv2eigen(cv_R, imu_laser_R);
        cv::cv2eigen(cv_T, imu_laser_T);
        cv::cv2eigen(imu_laser_rotation_offset, imu_laser_offset);

        T_i_l = Transformd(imu_laser_R, imu_laser_T);
        T_l_i = T_i_l.inverse();   

        double roll, pitch, yaw;
        tf2::Quaternion orientation_pre(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(), T_i_l.rot.w());
        tf2::Matrix3x3(orientation_pre).getRPY(roll, pitch, yaw);
        RCLCPP_INFO(node->get_logger(), BLUE"\n previous roll: %f previous pitch: %f previous yaw: %f" RESET, roll *180/M_PI, pitch *180/M_PI, yaw *180/M_PI); 

        tf2::Quaternion IMU_LASER_R_offset;
        IMU_LASER_R_offset.setRPY(imu_laser_offset[0]* M_PI / 180, imu_laser_offset[1] * M_PI / 180, 
                            imu_laser_offset[2]* M_PI / 180);

        tf2::Quaternion IMU_LASER_R(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(),
                                                        T_i_l.rot.w());
        tf2::Quaternion IMU_LASER = IMU_LASER_R_offset * IMU_LASER_R;
        Eigen::Quaterniond imu_laser_rot;             
        imu_laser_rot = Eigen::Quaterniond(IMU_LASER.w(), IMU_LASER.x(), IMU_LASER.y(),
                                                      IMU_LASER.z());
         
        T_i_l.rot=imu_laser_rot;
        T_l_i = T_i_l.inverse(); 
        imu_laser_R=T_i_l.rot.toRotationMatrix();
        
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_i_l Extrinsic : \n" << T_i_l.matrix());
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_l_i Extrinsic : \n" << T_l_i.matrix()); 
    }
    else
    {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation_camera_laser"] >> cv_R;
        fsSettings["extrinsicTranslation_camera_laser"] >> cv_T;
        cv::cv2eigen(cv_R, cam_laser_R);
        cv::cv2eigen(cv_T, cam_laser_T);

        Tcam_lidar = Transformd(cam_laser_R, cam_laser_T);

        fsSettings["extrinsicRotation_imu_camera"] >> cv_R;
        fsSettings["extrinsicTranslation_imu_camera"] >> cv_T;

        cv::cv2eigen(cv_R, imu_camera_R);
        cv::cv2eigen(cv_T, imu_camera_T);
        Eigen::Quaterniond Q(imu_camera_R);
        imu_camera_R = Q.normalized();

        T_i_c = Transformd(imu_camera_R, imu_camera_T);

        T_i_l = T_i_c * Tcam_lidar;
        T_l_i = T_i_l.inverse();

        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_i_l Extrinsic : \n" << T_i_l.matrix());
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_l_i Extrinsic : \n" << T_l_i.matrix());
    }

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
    // TF alignment params
    node->declare_parameter<bool>("use_tf_alignment", false);
    node->declare_parameter<std::string>("base_link_frame", "base_link");
    node->declare_parameter<std::string>("imu_frame", "imu_link");
    node->declare_parameter<std::string>("lidar_frame", "lidar_link");

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

    // TF alignment
    USE_TF_ALIGNMENT = node->get_parameter("use_tf_alignment").as_bool();
    BASE_FRAME = node->get_parameter("base_link_frame").as_string();
    IMU_FRAME = node->get_parameter("imu_frame").as_string();
    LIDAR_FRAME = node->get_parameter("lidar_frame").as_string();

    // Sensor type mapping
    const std::unordered_map<std::string, SensorType> sensorTypeMap = {
        {"velodyne", SensorType::VELODYNE},
        {"ouster", SensorType::OUSTER},
        {"livox", SensorType::LIVOX},
        {"vectornav_enu", SensorType::VECTORNAV_ENU}
    };

    if (sensorTypeMap.find(LIDAR_SENSOR) == sensorTypeMap.end()) {
        RCLCPP_ERROR(node->get_logger(), "Unsupported lidar sensor type: %s", LIDAR_SENSOR.c_str());
        return false;
    }
    if (sensorTypeMap.find(IMU_SENSOR) == sensorTypeMap.end()) {
        RCLCPP_ERROR(node->get_logger(), "Unsupported IMU sensor type: %s", IMU_SENSOR.c_str());
        return false;
    }
    lidar_sensor = sensorTypeMap.at(LIDAR_SENSOR);
    imu_sensor = sensorTypeMap.at(IMU_SENSOR);

    // --- TF Lookup ---
    if (USE_TF_ALIGNMENT) {
        RCLCPP_INFO(node->get_logger(), CYAN BOLD "[TF Alignment] Looking up transforms: %s → %s, %s → %s" RESET,
                    BASE_FRAME.c_str(), IMU_FRAME.c_str(), BASE_FRAME.c_str(), LIDAR_FRAME.c_str());

        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Wait for transforms to become available
        bool got_imu_tf = false, got_lidar_tf = false;
        for (int attempt = 0; attempt < 50 && (!got_imu_tf || !got_lidar_tf); ++attempt) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(node);

            if (!got_imu_tf) {
                try {
                    auto tf_base_imu = tf_buffer->lookupTransform(BASE_FRAME, IMU_FRAME, tf2::TimePointZero);
                    auto& r = tf_base_imu.transform.rotation;
                    auto& t = tf_base_imu.transform.translation;
                    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
                    q.normalize();
                    R_base_imu = q.toRotationMatrix();
                    t_base_imu = Eigen::Vector3d(t.x, t.y, t.z);
                    got_imu_tf = true;
                    RCLCPP_INFO(node->get_logger(), GREEN "[TF] Got %s → %s transform" RESET, BASE_FRAME.c_str(), IMU_FRAME.c_str());
                } catch (const tf2::TransformException&) {}
            }

            if (!got_lidar_tf) {
                try {
                    auto tf_base_lidar = tf_buffer->lookupTransform(BASE_FRAME, LIDAR_FRAME, tf2::TimePointZero);
                    auto& r = tf_base_lidar.transform.rotation;
                    auto& t = tf_base_lidar.transform.translation;
                    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
                    q.normalize();
                    R_base_lidar = q.toRotationMatrix();
                    t_base_lidar = Eigen::Vector3d(t.x, t.y, t.z);
                    got_lidar_tf = true;
                    RCLCPP_INFO(node->get_logger(), GREEN "[TF] Got %s → %s transform" RESET, BASE_FRAME.c_str(), LIDAR_FRAME.c_str());
                } catch (const tf2::TransformException&) {}
            }
        }

        if (!got_imu_tf || !got_lidar_tf) {
            RCLCPP_ERROR(node->get_logger(), RED "[TF] Failed to get TF transforms after 5s! Make sure robot_state_publisher is running." RESET);
            return false;
        }

        // Compute T_i_l_working: translation-only (identity rotation)
        // In the rectified frames, both sensors are rotation-aligned to base,
        // so the extrinsic is purely translational.
        Eigen::Vector3d t_i_l_working = t_base_lidar - t_base_imu;
        T_i_l_working = Transformd(Eigen::Matrix3d::Identity(), t_i_l_working);

        RCLCPP_INFO_STREAM(node->get_logger(), CYAN BOLD "\n[TF] R_base_imu:\n" RESET << R_base_imu);
        RCLCPP_INFO_STREAM(node->get_logger(), CYAN BOLD "\n[TF] R_base_lidar:\n" RESET << R_base_lidar);
        RCLCPP_INFO_STREAM(node->get_logger(), CYAN BOLD "\n[TF] t_base_imu: " RESET << t_base_imu.transpose());
        RCLCPP_INFO_STREAM(node->get_logger(), CYAN BOLD "\n[TF] t_base_lidar: " RESET << t_base_lidar.transpose());
        RCLCPP_INFO_STREAM(node->get_logger(), CYAN BOLD "\n[TF] T_i_l_working:\n" RESET << T_i_l_working.matrix());
    }

    RCLCPP_INFO(node->get_logger(), "LASER_TOPIC %s", LASER_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_TOPIC %s", IMU_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "ODOM_TOPIC %s", ODOM_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "WORLD_FRAME %s", WORLD_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR_FRAME %s", SENSOR_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "ProjectName %s", ProjectName.c_str());
    RCLCPP_INFO(node->get_logger(), "LIDAR_SENSOR %s", LIDAR_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_SENSOR %s", IMU_SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "USE_TF_ALIGNMENT %d", USE_TF_ALIGNMENT);
    RCLCPP_INFO(node->get_logger(), "USE_IMU_ROLL_PITCH %d", USE_IMU_ROLL_PITCH);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);

    return true;
}

// ---- Publish rectified static TF frames ----
// These frames share the same origin as the original sensor frames
// but are rotation-aligned to base_link.
void publishRectifiedWorkingFrames(rclcpp::Node::SharedPtr node) {
    if (!USE_TF_ALIGNMENT) return;

    static auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    auto makeRectifiedTransform = [&](const std::string& parent,
                                      const std::string& child,
                                      const Eigen::Vector3d& translation) {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = node->now();
        ts.header.frame_id = parent;
        ts.child_frame_id = child;
        ts.transform.translation.x = translation.x();
        ts.transform.translation.y = translation.y();
        ts.transform.translation.z = translation.z();
        // Identity rotation (aligned to base)
        ts.transform.rotation.w = 1.0;
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = 0.0;
        return ts;
    };

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(makeRectifiedTransform(BASE_FRAME, IMU_FRAME + "_rect", t_base_imu));
    transforms.push_back(makeRectifiedTransform(BASE_FRAME, LIDAR_FRAME + "_rect", t_base_lidar));
    static_broadcaster->sendTransform(transforms);

    // Update SENSOR_FRAME to use the rectified lidar frame for odometry output
    SENSOR_FRAME = LIDAR_FRAME + "_rect";
    SENSOR_FRAME_ROT = LIDAR_FRAME + "_rect_rot";

    RCLCPP_INFO(node->get_logger(), GREEN BOLD "[TF] Published rectified frames: %s_rect, %s_rect" RESET,
                IMU_FRAME.c_str(), LIDAR_FRAME.c_str());
}