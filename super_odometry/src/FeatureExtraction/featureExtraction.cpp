//
// Created by shibo zhao on 2020-09-27.
//

#include <super_odometry/FeatureExtraction/featureExtraction.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstdint>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/buffer.h>
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

#include "super_odometry/utils/imu_frame_utils.h"

namespace super_odometry {
    
    featureExtraction::featureExtraction(const rclcpp::NodeOptions & options)
    : Node("feature_extraction_node", options) {
    }

    void featureExtraction::initInterface() {      
        //! Callback Groups
        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_;

        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();  // Use BEST_EFFORT reliability
        imu_qos.keep_last(10);  // Keep last 10 messages

        rclcpp::QoS laser_qos(10);
        laser_qos.best_effort();  // Use BEST_EFFORT reliability
        laser_qos.keep_last(2);  // Keep last 10 messages

        if(!readGlobalparam(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read calibration. Exiting...");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "calibration");
        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
            return;
        }
        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[super_odometry::featureExtraction] Could not read parameters. Exiting...");
            rclcpp::shutdown();
            return;
        }
        rectified_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishRectifiedSensorFrames();
         
        RCLCPP_WARN(this->get_logger(), "config_.skipFrame: %d", config_.skipFrame);
        RCLCPP_INFO(this->get_logger(), "scan line number %d \n", config_.N_SCANS);      
        RCLCPP_INFO(this->get_logger(), "use imu roll and pitch %d \n", config_.use_imu_roll_pitch);

        if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64 && config_.N_SCANS != 4 && config_.N_SCANS != 128)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne, livox, ouster with 16, 32, 64 or 128 scan line! and livox mid 360");
            rclcpp::shutdown();
            return;
        }

        if (config_.lidar_sensor == SensorType::VELODYNE || config_.lidar_sensor == SensorType::OUSTER) {
            subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(LASER_TOPIC, laser_qos, 
                    std::bind(&featureExtraction::laserCloudHandler, this,
                    std::placeholders::_1), sub_options);
        } else if (config_.lidar_sensor == SensorType::LIVOX) {
            subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(LASER_TOPIC, laser_qos,
                    std::bind(&featureExtraction::livoxHandler, this,
                    std::placeholders::_1), sub_options);
        } //TODO: add this to config

        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, imu_qos, 
            std::bind(&featureExtraction::imu_Handler, this,
                        std::placeholders::_1), sub_options);

        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_TOPIC, 10, 
            std::bind(&featureExtraction::visual_odom_Handler, this,
                        std::placeholders::_1), sub_options);

        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/velodyne_cloud_2", 2);

        pubLaserFeatureInfo = this->create_publisher<super_odometry_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2);

        pubBobPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/bob_points", 2);

        pubPlannerPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/planner_points", 2);

        pubEdgePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            ProjectName+"/edge_points", 2);

        pubElevationMap = this->create_publisher<grid_map_msgs::msg::GridMap>(
            ProjectName+"/elevation_map", 2);

        // Subscribe to SLAM odometry for reliable world-frame position.
        // featureExtraction's t_w_original_l comes from IMU-only integration which
        // has no position (Imu struct carries only orientation), so it stays near zero.
        // The laser_mapping_node publishes the corrected pose on laser_odometry.
        auto slam_odom_cb = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(slam_pose_mutex_);
            slam_pos_ = Eigen::Vector3d(
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
            slam_rot_ = Eigen::Quaterniond(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
            has_slam_pose_ = true;
        };
        subSlamOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName + "/laser_odometry", 10, slam_odom_cb);

        // lio_prediction is published at IMU rate and gives high-rate pose updates.
        // Sharing the same callback keeps slam_pos_ fresh between lidar scans so the
        // high-rate timer can slide the map center accurately.
        subLioPrediction_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ProjectName + "/lio_prediction", 10, slam_odom_cb);

        // High-rate republish: move the map center to the latest pose and republish.
        // Data stays the same; only the window position changes until the next scan.
        if (config_.elevation_map_publish_rate > 0.0) {
            const auto period_ms = static_cast<int>(1000.0 / config_.elevation_map_publish_rate);
            elevation_map_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                [this]() {
                    std::lock_guard<std::mutex> map_lock(latest_map_mutex_);
                    if (latest_elevation_map_.getSize().prod() == 0) return;

                    Eigen::Vector3d pos;
                    {
                        std::lock_guard<std::mutex> pose_lock(slam_pose_mutex_);
                        pos = slam_pos_;
                    }
                    latest_elevation_map_.move(grid_map::Position(pos.x(), pos.y()));

                    auto msg = grid_map::GridMapRosConverter::toMessage(latest_elevation_map_);
                    msg->header.stamp = this->now();
                    pubElevationMap->publish(*msg);
                });
        }

        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < config_.N_SCANS; i++)
            {
                auto tmp_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "laser_scanid_" + std::to_string(i), 2);
                pubEachScan.push_back(tmp_publisher_);
            }
        }

        delay_count_ = 0;
        if (config_.imu_dt <= 0.0f) {
            RCLCPP_WARN(this->get_logger(),
                        "[super_odometry::featureExtraction] imu_dt must be > 0; falling back to 0.005");
            config_.imu_dt = 0.005f;
        }
        m_imuPeriod = config_.imu_dt;
        if (imu_Init && config_.imu_dt > 0.0f) {
            imu_Init->imu_frequency = 1.0 / static_cast<double>(config_.imu_dt);
        }
    }

    bool featureExtraction::readParameters()
    {          

        this->declare_parameter<int>("feature_extraction_node.scan_line", 4);
        this->declare_parameter<int>("feature_extraction_node.mapping_skip_frame", 1);
        this->declare_parameter<double>("feature_extraction_node.blindFront", 0.1);
        this->declare_parameter<double>("feature_extraction_node.blindBack", -1.0);
        this->declare_parameter<double>("feature_extraction_node.blindLeft", 0.1);
        this->declare_parameter<double>("feature_extraction_node.blindRight", -0.1);
        this->declare_parameter<bool>("feature_extraction_node.use_dynamic_mask", false);
        this->declare_parameter<bool>("feature_extraction_node.use_imu_roll_pitch", false);
        this->declare_parameter<float>("feature_extraction_node.min_range", 0.2);
        this->declare_parameter<float>("feature_extraction_node.max_range", 130.0);
        this->declare_parameter<int>("feature_extraction_node.filter_point_size", 3);
        this->declare_parameter<int>("feature_extraction_node.provide_point_time", 1);
        this->declare_parameter<double>("feature_extraction_node.voxel_leaf_size", 0.0);
        this->declare_parameter<bool>("feature_extraction_node.debug_view", false);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_x_limit", 1.0);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_y_limit", 1.0);
        this->declare_parameter<double>("feature_extraction_node.imu_acc_z_limit", 1.0);
        this->declare_parameter<std::string>("feature_extraction_node.lidar_sensor", "livox");
        this->declare_parameter<std::string>("feature_extraction_node.imu_sensor", "livox");
        this->declare_parameter<float>("imu_dt", 0.005);
        this->declare_parameter<bool>("elevation_map.enabled", true);
        this->declare_parameter<double>("elevation_map.resolution", 0.5);
        this->declare_parameter<double>("elevation_map.size", 20.0);
        this->declare_parameter<double>("elevation_map.height_cutoff", 3.0);
        this->declare_parameter<double>("elevation_map.height_min", -3.0);
        this->declare_parameter<int>("elevation_map.fill_passes", 3);
        this->declare_parameter<int>("elevation_map.buffer_size", 5);
        this->declare_parameter<bool>("elevation_map.equal_weight", false);
        this->declare_parameter<double>("elevation_map.publish_rate", 0.0);
        this->declare_parameter<bool>("elevation_map.gaussian_blur", false);
        this->declare_parameter<int>("elevation_map.gaussian_kernel_size", 3);
        this->declare_parameter<bool>("elevation_map.yaw_filter", false);
        this->declare_parameter<double>("elevation_map.yaw_min", -M_PI);
        this->declare_parameter<double>("elevation_map.yaw_max",  M_PI);

                
        config_.N_SCANS = this->get_parameter("feature_extraction_node.scan_line").as_int();
        config_.skipFrame = this->get_parameter("feature_extraction_node.mapping_skip_frame").as_int();
        config_.box_size.blindFront = this->get_parameter("feature_extraction_node.blindFront").as_double();
        config_.box_size.blindBack = this->get_parameter("feature_extraction_node.blindBack").as_double();
        config_.box_size.blindLeft = this->get_parameter("feature_extraction_node.blindLeft").as_double();
        config_.box_size.blindRight = this->get_parameter("feature_extraction_node.blindRight").as_double();
        // config_.use_imu_roll_pitch = this->get_parameter("feature_extraction_node.use_imu_roll_pitch").as_bool();
        config_.min_range = this->get_parameter("feature_extraction_node.min_range").as_double();
        config_.max_range = this->get_parameter("feature_extraction_node.max_range").as_double();
        config_.filter_point_size = this->get_parameter("feature_extraction_node.filter_point_size").as_int();
        config_.provide_point_time = this->get_parameter("feature_extraction_node.provide_point_time").as_int();
        config_.voxel_leaf_size = this->get_parameter("feature_extraction_node.voxel_leaf_size").as_double();
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;

        if (!USE_TF_ALIGNMENT) {
            RCLCPP_ERROR(
                this->get_logger(),
                "[featureExtraction] This branch requires use_tf_alignment=true to derive LiDAR mount correction from TF.");
            return false;
        }

        tf2::Quaternion orientation_tf(Q_LIDAR_TO_BASE.x(), Q_LIDAR_TO_BASE.y(), Q_LIDAR_TO_BASE.z(), Q_LIDAR_TO_BASE.w());
        double roll_rad = 0.0;
        double pitch_rad = 0.0;
        double yaw_rad = 0.0;
        tf2::Matrix3x3(orientation_tf).getRPY(roll_rad, pitch_rad, yaw_rad);

        // The existing c04bc56 leveling path applies level_R = mount_R.transpose().
        // To preserve that behavior, convert the TF orientation into the legacy
        // mount-angle convention expected by the transpose-based correction.
        roll_deg = -roll_rad * 180.0 / M_PI;
        pitch_deg = -pitch_rad * 180.0 / M_PI;
        yaw_deg = -yaw_rad * 180.0 / M_PI;

        RCLCPP_INFO(
            this->get_logger(),
            "[featureExtraction] Using TF-derived legacy lidar mount RPY from %s -> %s: roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
            BASE_LINK_FRAME.c_str(),
            LIDAR_FRAME_NAME.c_str(),
            roll_deg,
            pitch_deg,
            yaw_deg);

        config_.lidar_mount_roll_rad = roll_deg * M_PI / 180.0;
        config_.lidar_mount_pitch_rad = pitch_deg * M_PI / 180.0;
        config_.lidar_mount_yaw_rad = yaw_deg * M_PI / 180.0;
        config_.use_dynamic_mask = this->get_parameter("feature_extraction_node.use_dynamic_mask").as_bool(); 
        config_.debug_view_enabled = this->get_parameter("feature_extraction_node.debug_view").as_bool();
        config_.imu_acc_x_limit = this->get_parameter("feature_extraction_node.imu_acc_x_limit").as_double();
        config_.imu_acc_y_limit = this->get_parameter("feature_extraction_node.imu_acc_y_limit").as_double();
        config_.imu_acc_z_limit = this->get_parameter("feature_extraction_node.imu_acc_z_limit").as_double();
        config_.imu_dt = this->get_parameter("imu_dt").as_double();
        config_.elevation_map_enabled        = this->get_parameter("elevation_map.enabled").as_bool();
        config_.elevation_map_resolution     = this->get_parameter("elevation_map.resolution").as_double();
        config_.elevation_map_size           = this->get_parameter("elevation_map.size").as_double();
        config_.elevation_map_height_cutoff  = this->get_parameter("elevation_map.height_cutoff").as_double();
        config_.elevation_map_height_min     = this->get_parameter("elevation_map.height_min").as_double();
        config_.elevation_map_fill_passes    = this->get_parameter("elevation_map.fill_passes").as_int();
        config_.elevation_map_buffer_size    = this->get_parameter("elevation_map.buffer_size").as_int();
        config_.elevation_map_equal_weight   = this->get_parameter("elevation_map.equal_weight").as_bool();
        config_.elevation_map_publish_rate   = this->get_parameter("elevation_map.publish_rate").as_double();
        config_.elevation_map_gaussian_blur  = this->get_parameter("elevation_map.gaussian_blur").as_bool();
        config_.elevation_map_gaussian_kernel_size =
            this->get_parameter("elevation_map.gaussian_kernel_size").as_int();
        config_.elevation_map_yaw_filter     = this->get_parameter("elevation_map.yaw_filter").as_bool();
        config_.elevation_map_yaw_min        = this->get_parameter("elevation_map.yaw_min").as_double();
        config_.elevation_map_yaw_max        = this->get_parameter("elevation_map.yaw_max").as_double();
        config_.use_imu_roll_pitch = USE_IMU_ROLL_PITCH;
        config_.imu_acc_x_limit = IMU_ACC_X_LIMIT;
        config_.imu_acc_y_limit = IMU_ACC_Y_LIMIT;
        config_.imu_acc_z_limit = IMU_ACC_Z_LIMIT;

        if (LIDAR_SENSOR == "livox") {
            config_.lidar_sensor = SensorType::LIVOX;
        } else if (LIDAR_SENSOR == "velodyne") {
            config_.lidar_sensor = SensorType::VELODYNE;
        } else if (LIDAR_SENSOR == "ouster") {
            config_.lidar_sensor = SensorType::OUSTER;
        } 

        if (IMU_SENSOR == "vectornav") {
            config_.imu_sensor = SensorType::VECTORNAV;
        } else if (IMU_SENSOR == "livox") {
            config_.imu_sensor = SensorType::LIVOX;
        } else if (IMU_SENSOR == "velodyne") {
            config_.imu_sensor = SensorType::VELODYNE;
        } else if (IMU_SENSOR == "ouster") {
            config_.imu_sensor = SensorType::OUSTER;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown IMU sensor type: %s", IMU_SENSOR.c_str());
            return false;
        }

        return true;
    }

    void featureExtraction::publishRectifiedSensorFrames() {
        if (!rectified_tf_broadcaster_) {
            return;
        }

        geometry_msgs::msg::TransformStamped lidar_tf;
        lidar_tf.header.stamp = this->now();
        lidar_tf.header.frame_id = LIDAR_FRAME_NAME;
        lidar_tf.child_frame_id = LIDAR_FRAME_RECTIFIED;
        lidar_tf.transform.translation.x = 0.0;
        lidar_tf.transform.translation.y = 0.0;
        lidar_tf.transform.translation.z = 0.0;
        const Eigen::Quaterniond q_base_to_lidar = Q_LIDAR_TO_BASE.inverse();
        lidar_tf.transform.rotation.x = q_base_to_lidar.x();
        lidar_tf.transform.rotation.y = q_base_to_lidar.y();
        lidar_tf.transform.rotation.z = q_base_to_lidar.z();
        lidar_tf.transform.rotation.w = q_base_to_lidar.w();

        geometry_msgs::msg::TransformStamped imu_tf;
        imu_tf.header.stamp = this->now();
        imu_tf.header.frame_id = IMU_FRAME_NAME;
        imu_tf.child_frame_id = IMU_FRAME_RECTIFIED;
        imu_tf.transform.translation.x = 0.0;
        imu_tf.transform.translation.y = 0.0;
        imu_tf.transform.translation.z = 0.0;
        const Eigen::Quaterniond q_base_to_imu = Q_IMU_TO_BASE.inverse();
        imu_tf.transform.rotation.x = q_base_to_imu.x();
        imu_tf.transform.rotation.y = q_base_to_imu.y();
        imu_tf.transform.rotation.z = q_base_to_imu.z();
        imu_tf.transform.rotation.w = q_base_to_imu.w();

        std::vector<geometry_msgs::msg::TransformStamped> rectified_transforms{lidar_tf, imu_tf};
        rectified_tf_broadcaster_->sendTransform(rectified_transforms);

        RCLCPP_INFO(this->get_logger(),
                    "[featureExtraction] Published URDF-linked rectified sensor frames %s -> %s and %s -> %s",
                    LIDAR_FRAME_NAME.c_str(), LIDAR_FRAME_RECTIFIED.c_str(),
                    IMU_FRAME_NAME.c_str(), IMU_FRAME_RECTIFIED.c_str());
    }



    template <typename Meas>
    bool featureExtraction::synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                                     MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf)
    {

        if (lidarBuf.getSize() == 0 or measureBuf.getSize() == 0)
            return false;

        double lidar_start_time;
        lidarBuf.getFirstTime(lidar_start_time);

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
        lidarBuf.getFirstMeas(lidar_msg);

        double lidar_end_time = lidar_start_time + lidar_msg->back().time;

        // obtain the current imu message
        double meas_start_time=0;
        measureBuf.getFirstTime(meas_start_time);

        double meas_end_time=0;
        measureBuf.getLastTime(meas_end_time);

        if (meas_end_time <= lidar_end_time) // make sure imu message arrives after lidar message
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "meas_end_time < lidar_end_time; waiting for newer IMU/odom data before processing this lidar frame");
            RCLCPP_WARN(this->get_logger(), "meas_end_time %f < %f lidar_end_time", meas_end_time, lidar_end_time);
            RCLCPP_WARN(this->get_logger(), "Keeping the front lidar frame buffered until sensor data catches up");

            return false;
        }

        if (meas_start_time >= lidar_start_time)          
        {
            RCLCPP_WARN(this->get_logger(), "throw laser scan, only should happen at the beginning");
            lidarBuf.clean(lidar_start_time);
            RCLCPP_WARN(this->get_logger(), "removed the lidarBuf size % d, measureBuf size % d ", lidarBuf.getSize(), measureBuf.getSize());
            RCLCPP_WARN(this->get_logger(), "meas_start_time: %f > lidar_start_time: %f ", meas_start_time, lidar_start_time);
            return false;
        }
        else
        {
            return true;
        }
        
    }


    

    template<typename BufferType>
    void featureExtraction::removePointDistortion(
        double lidar_start_time, 
        double lidar_end_time,
        MapRingBuffer<BufferType> &buffer,
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {
        // Step 1: Define how to extract pose based on buffer type
        auto extractPose = [](const BufferType& data) -> Transformd {
            Transformd pose;
            if constexpr (std::is_same_v<BufferType, Imu::Ptr>) {
                // For IMU data: only rotation, zero translation
                pose.rot = data->q_w_i;
                pose.pos = Eigen::Vector3d::Zero();
            } else {
                // For VIO data: both rotation and translation
                pose.rot = Eigen::Quaterniond(
                    data->pose.pose.orientation.w,
                    data->pose.pose.orientation.x,
                    data->pose.pose.orientation.y,
                    data->pose.pose.orientation.z
                );
                pose.pos = Eigen::Vector3d(
                    data->pose.pose.position.x,
                    data->pose.pose.position.y,
                    data->pose.pose.position.z
                );
            }
            return pose;
        };

        
         // Step 2: Get interpolated poses directly
        auto getInterpolatedPoseAtTime = [&buffer, &extractPose](double timestamp) -> Transformd {
        auto after_ptr = buffer.measMap_.upper_bound(timestamp);
        if (after_ptr->first < 0.0001) {
            after_ptr = buffer.measMap_.begin();
        }

        if (after_ptr == buffer.measMap_.begin()) {
            return extractPose(after_ptr->second);
        }

        auto before_ptr = std::prev(after_ptr);
        double ratio = (timestamp - before_ptr->first) / 
                      (after_ptr->first - before_ptr->first);

        Transformd before_pose = extractPose(before_ptr->second);
        Transformd after_pose = extractPose(after_ptr->second);

        Transformd result;
        result.rot = before_pose.rot.slerp(ratio, after_pose.rot);
        result.pos = (1 - ratio) * before_pose.pos + ratio * after_pose.pos;
        return result;
    };

    // Step 3: Get start pose
    Transformd start_pose = getInterpolatedPoseAtTime(lidar_start_time);

    // Step 4: Calculate initial transform
    Transformd T_w_original(start_pose.rot, start_pose.pos);
    bool is_imu_data = std::is_same_v<BufferType, Imu::Ptr>;
    Transformd T_w_original_sensor = is_imu_data ? 
                                    T_w_original * T_i_l : 
                                    T_w_original;

    q_w_original_l = T_w_original_sensor.rot;
    t_w_original_l = T_w_original_sensor.pos;

    // Step 5: Process each point
    for (auto &point : lidar_msg->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        double point_time = point.time + lidar_start_time;
        Transformd point_pose = getInterpolatedPoseAtTime(point_time);
        
        // Transform point
        Transformd T_w_current(point_pose.rot, point_pose.pos);
        Transformd T_original_current = T_w_original.inverse() * T_w_current;
        Transformd T_final = is_imu_data ? 
                            T_l_i * T_original_current * T_i_l : 
                            T_original_current;

        Eigen::Vector3d pt(point.x, point.y, point.z);
        pt = T_final * pt;
            
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        }
    }

    

    // Helper functions for clarity and reusability
    template<typename BufferType>
    Transformd featureExtraction::getInterpolatedPose(
        double timestamp, 
        MapRingBuffer<BufferType> &buffer,
        const std::function<Transformd(const BufferType&)>& extractPose)
    {
        auto after_ptr = buffer.measMap_.upper_bound(timestamp);
        if (after_ptr->first < 0.0001) {
            after_ptr = buffer.measMap_.begin();
        }

        if (after_ptr == buffer.measMap_.begin()) {
            return extractPose(after_ptr->second);
        }

        auto before_ptr = std::prev(after_ptr);
        double ratio = (timestamp - before_ptr->first) / 
                    (after_ptr->first - before_ptr->first);

        Transformd before_pose = extractPose(before_ptr->second);
        Transformd after_pose = extractPose(after_ptr->second);

        Transformd result;
        result.rot = before_pose.rot.slerp(ratio, after_pose.rot);
        result.pos = (1 - ratio) * before_pose.pos + ratio * after_pose.pos;
        return result;
    }

    bool featureExtraction::isPointValid(const point_os::PointcloudXYZITR& point) {
        return std::isfinite(point.x) && 
            std::isfinite(point.y) && 
            std::isfinite(point.z);
    }

    Eigen::Vector3d featureExtraction::transformPoint(
        const point_os::PointcloudXYZITR& point,
        const Transformd& T_w_original,
        const Transformd& point_pose,
        bool is_imu_data)
    {
        Transformd T_w_current(point_pose.rot, point_pose.pos);
        Transformd T_original_current = T_w_original.inverse() * T_w_current;

        Transformd T_final = is_imu_data ? 
                            T_l_i * T_original_current * T_i_l : 
                            T_original_current;

        Eigen::Vector3d pt(point.x, point.y, point.z);
        return T_final * pt;
    }

    void featureExtraction::updatePointPosition(point_os::PointcloudXYZITR& point, const Eigen::Vector3d& new_pos) {
        point.x = new_pos.x();
        point.y = new_pos.y();
        point.z = new_pos.z();
    }


    template <typename Point>
    sensor_msgs::msg::PointCloud2
    featureExtraction::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, typename pcl::PointCloud<Point>::Ptr thisCloud,
                                    rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        return tempCloud;
    }

    void featureExtraction::publishTopic(double lidar_start_time, 
                                         pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr laser_no_distortion_points,
                                         pcl::PointCloud<PointType>::Ptr edgePoints,
                                         pcl::PointCloud<PointType>::Ptr plannerPoints, 
                                         pcl::PointCloud<PointType>::Ptr depthPoints,
                                         Eigen::Quaterniond q_w_original_l)
    {
        FeatureHeader.frame_id = WORLD_FRAME;
        FeatureHeader.stamp = rclcpp::Time(lidar_start_time*1e9);
        laserFeature.header = FeatureHeader;
        laserFeature.imu_available = false;
        laserFeature.odom_available = false;

      
        laserFeature.cloud_nodistortion = publishCloud<point_os::PointcloudXYZITR>(pubLaserCloud, laser_no_distortion_points, FeatureHeader.stamp, LIDAR_FRAME_RECTIFIED);
        laserFeature.cloud_corner = publishCloud<PointType>(pubEdgePoints, edgePoints, FeatureHeader.stamp, LIDAR_FRAME_RECTIFIED);
        laserFeature.cloud_surface = publishCloud<PointType>(pubPlannerPoints, plannerPoints, FeatureHeader.stamp, LIDAR_FRAME_RECTIFIED);
        laserFeature.cloud_realsense=publishCloud<PointType>(pubBobPoints, depthPoints, FeatureHeader.stamp, LIDAR_FRAME_RECTIFIED);
       
        laserFeature.initial_quaternion_x = q_w_original_l.x();
        laserFeature.initial_quaternion_y = q_w_original_l.y();
        laserFeature.initial_quaternion_z = q_w_original_l.z();
        laserFeature.initial_quaternion_w= q_w_original_l.w();

        laserFeature.initial_pose_x = t_w_original_l.x();
        laserFeature.initial_pose_y = t_w_original_l.y();
        laserFeature.initial_pose_z = t_w_original_l.z();

        laserFeature.imu_available = true;
        laserFeature.sensor = 0; 
        pubLaserFeatureInfo->publish(laserFeature);
    }

    void featureExtraction::extractFeatures(
        double lidar_start_time,
        const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr& lidar_msg,
        const Eigen::Quaterniond& quaternion)
    {
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_filtered = lidar_msg;
        if (config_.voxel_leaf_size > 1e-6) {
            lidar_filtered.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
            pcl::VoxelGrid<point_os::PointcloudXYZITR> voxel;
            voxel.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
            voxel.setInputCloud(lidar_msg);
            voxel.filter(*lidar_filtered);
        }

        if (std::abs(config_.lidar_mount_roll_rad) > 1e-12 ||
            std::abs(config_.lidar_mount_pitch_rad) > 1e-12 ||
            std::abs(config_.lidar_mount_yaw_rad) > 1e-12) {
            const Eigen::Matrix3d mount_R =
                (Eigen::AngleAxisd(config_.lidar_mount_yaw_rad, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(config_.lidar_mount_pitch_rad, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(config_.lidar_mount_roll_rad, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            const Eigen::Matrix3d level_R = mount_R.transpose();

            for (auto& pt : lidar_filtered->points) {
                Eigen::Vector3d p(pt.x, pt.y, pt.z);
                p = level_R * p;
                pt.x = static_cast<float>(p.x());
                pt.y = static_cast<float>(p.y());
                pt.z = static_cast<float>(p.z());
            }
        }

        pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
        plannerPoints->reserve(lidar_filtered->points.size());
        pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
        edgePoints->reserve(lidar_filtered->points.size());
        pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
        bobPoints->reserve(lidar_filtered->points.size());

        uniformFeatureExtraction(lidar_filtered, plannerPoints, config_.filter_point_size, config_.min_range, config_.max_range);

        buildAndPublishElevationMap(lidar_filtered, lidar_start_time);

        publishTopic(lidar_start_time, lidar_filtered, edgePoints, plannerPoints, bobPoints, quaternion);
    }


    bool featureExtraction::undistortionAndFeatureExtraction()      
    {
        LASER_IMU_SYNC_SCCUESS = synchronize_measurements<Imu::Ptr>(imuBuf, lidarBuf);
        LASER_CAMERA_SYNC_SUCCESS = synchronize_measurements<nav_msgs::msg::Odometry::SharedPtr>(visualOdomBuf, lidarBuf);

        if (frameCount > 100 and LASER_CAMERA_SYNC_SUCCESS == true)
            LASER_CAMERA_SYNC_SUCCESS = true;
        else
            LASER_CAMERA_SYNC_SUCCESS = false;

        if ((LASER_IMU_SYNC_SCCUESS == true or LASER_CAMERA_SYNC_SUCCESS == true) and lidarBuf.getSize() > 0)
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;32m----> Both IMU ,VIO laserscan are synchronized!.\033[0m");
                removePointDistortion<nav_msgs::msg::Odometry::SharedPtr>(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == false and LASER_CAMERA_SYNC_SUCCESS == true)
            {
                removePointDistortion<nav_msgs::msg::Odometry::SharedPtr>(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            }

            if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == false)
            {
                // RCLCPP_INFO(this->get_logger(), "\033[1;32m----> IMU and laserscan is synchronized!.\033[0m");
                removePointDistortion<Imu::Ptr>(lidar_start_time, lidar_end_time, imuBuf, lidar_msg);
            }

            // Extract features and publish
            extractFeatures(lidar_start_time, lidar_msg, q_w_original_l);

            LASER_CAMERA_SYNC_SUCCESS = false;
            LASER_IMU_SYNC_SCCUESS = false;
            return true;
        }
        else if (imuBuf.empty())
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);
            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            RCLCPP_INFO(this->get_logger(), "\033[1;32m----> no IMU data, running LiDAR Odometry only.\033[0m");
            Eigen::Quaterniond default_quaternion = Eigen::Quaterniond::Identity();
            
            // Extract features and publish with default quaternion
            extractFeatures(lidar_start_time, lidar_msg, default_quaternion);
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "sync unsuccessful, keeping scan buffered until newer IMU/odom data arrives");
        }

        return false;
    }

    void featureExtraction::buildAndPublishElevationMap(
        const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr& points, double lidar_start_time)
    {
        if (!config_.elevation_map_enabled) return;

        const float resolution = config_.elevation_map_resolution;
        const float size       = config_.elevation_map_size;
        const float height_max = config_.elevation_map_height_cutoff;
        const float height_min = config_.elevation_map_height_min;

        // Use SLAM pose for reliable world-frame position.
        // t_w_original_l comes from IMU-only integration which carries no position
        // (Imu struct has orientation only), so it stays near zero.
        // Fall back to it only before the first SLAM pose arrives.
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        {
            std::lock_guard<std::mutex> lock(slam_pose_mutex_);
            if (has_slam_pose_) {
                R = slam_rot_.toRotationMatrix();
                t = slam_pos_;
            } else {
                R = q_w_original_l.toRotationMatrix();
                t = t_w_original_l;
            }
        }

        // --- Build this scan's raw elevation map ---
        grid_map::GridMap raw_map({"elevation"});
        raw_map.setFrameId(WORLD_FRAME);
        raw_map.setGeometry(grid_map::Length(size, size), resolution,
                            grid_map::Position(t.x(), t.y()));
        raw_map["elevation"].setConstant(NAN);

        for (const auto& pt : points->points) {
            // Azimuth (yaw) filter in sensor frame — mirrors FastDEM's cropAngle.
            // yaw_min <= yaw_max: keep [yaw_min, yaw_max] (normal sector).
            // yaw_min >  yaw_max: keep outside (yaw_max, yaw_min) (wrap-around sector).
            if (config_.elevation_map_yaw_filter) {
                const float yaw = std::atan2(pt.y, pt.x);
                const bool in_range = (config_.elevation_map_yaw_min <= config_.elevation_map_yaw_max)
                    ? (yaw >= config_.elevation_map_yaw_min && yaw <= config_.elevation_map_yaw_max)
                    : (yaw >= config_.elevation_map_yaw_min || yaw <= config_.elevation_map_yaw_max);
                if (!in_range) continue;
            }

            const Eigen::Vector3d p_world = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
            if (p_world.z() > height_max || p_world.z() < height_min) continue;
            grid_map::Position pos(p_world.x(), p_world.y());
            if (!raw_map.isInside(pos)) continue;
            grid_map::Index idx;
            raw_map.getIndex(pos, idx);
            float& cell = raw_map.at("elevation", idx);
            if (std::isnan(cell) || static_cast<float>(p_world.z()) < cell)
                cell = static_cast<float>(p_world.z());
        }

        // --- Buffer management ---
        // Keep the last N raw maps. Each is in world frame with its own center,
        // so position tracking is implicit in the grid_map geometry.
        elevation_map_buffer_.push_back(raw_map);
        while (static_cast<int>(elevation_map_buffer_.size()) > config_.elevation_map_buffer_size)
            elevation_map_buffer_.pop_front();

        // --- Weighted temporal merge ---
        // Linear weights: oldest frame = weight 1, newest = weight N.
        // All maps are in world frame so we can query any position directly.
        grid_map::GridMap merged({"elevation"});
        merged.setFrameId(WORLD_FRAME);
        merged.setGeometry(grid_map::Length(size, size), resolution,
                           grid_map::Position(t.x(), t.y()));
        merged["elevation"].setConstant(NAN);

        const int N = static_cast<int>(elevation_map_buffer_.size());
        for (grid_map::GridMapIterator it(merged); !it.isPastEnd(); ++it) {
            grid_map::Position pos;
            merged.getPosition(*it, pos);

            float weighted_sum = 0.0f;
            float weight_sum   = 0.0f;
            for (int i = 0; i < N; ++i) {
                // Equal weight: every frame counts the same.
                // Linear weight: oldest=1, newest=N — newer frames dominate.
                const float w = config_.elevation_map_equal_weight
                                ? 1.0f
                                : static_cast<float>(i + 1);
                const auto& buf_map = elevation_map_buffer_[i];
                if (!buf_map.isInside(pos)) continue;
                grid_map::Index buf_idx;
                buf_map.getIndex(pos, buf_idx);
                const float val = buf_map.at("elevation", buf_idx);
                if (!std::isnan(val)) {
                    weighted_sum += w * val;
                    weight_sum   += w;
                }
            }
            if (weight_sum > 0.0f)
                merged.at("elevation", *it) = weighted_sum / weight_sum;
        }

        // --- Gap filling: propagate known values into unknown cells ---
        if (config_.elevation_map_fill_passes > 0) {
            auto& layer = merged["elevation"];
            const int rows = layer.rows();
            const int cols = layer.cols();
            Eigen::MatrixXf buf(rows, cols);
            for (int pass = 0; pass < config_.elevation_map_fill_passes; ++pass) {
                buf = layer;
                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        if (!std::isnan(layer(i, j))) continue;
                        float sum = 0.0f; int count = 0;
                        for (int di = -1; di <= 1; ++di) {
                            for (int dj = -1; dj <= 1; ++dj) {
                                if (di == 0 && dj == 0) continue;
                                const int ni = i + di, nj = j + dj;
                                if (ni < 0 || ni >= rows || nj < 0 || nj >= cols) continue;
                                if (!std::isnan(layer(ni, nj))) { sum += layer(ni, nj); ++count; }
                            }
                        }
                        if (count > 0) buf(i, j) = sum / static_cast<float>(count);
                    }
                }
                layer = buf;
            }
        }

        if (config_.elevation_map_gaussian_blur) {
            auto& layer = merged["elevation"];
            const int rows = layer.rows();
            const int cols = layer.cols();
            int kernel_size = config_.elevation_map_gaussian_kernel_size;
            if (kernel_size < 1) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "elevation_map.gaussian_kernel_size=%d is invalid; using 1",
                    kernel_size);
                kernel_size = 1;
            }
            if (kernel_size % 2 == 0) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "elevation_map.gaussian_kernel_size=%d must be odd; using %d",
                    kernel_size, kernel_size + 1);
                ++kernel_size;
            }
            if (kernel_size > 1) {
                using RowMajorMatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

                RowMajorMatrixXf values(rows, cols);
                RowMajorMatrixXf weights = RowMajorMatrixXf::Zero(rows, cols);
                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        const float val = layer(i, j);
                        if (std::isnan(val)) {
                            values(i, j) = 0.0f;
                            continue;
                        }
                        values(i, j) = val;
                        weights(i, j) = 1.0f;
                    }
                }

                RowMajorMatrixXf blurred_values = RowMajorMatrixXf::Zero(rows, cols);
                RowMajorMatrixXf blurred_weights = RowMajorMatrixXf::Zero(rows, cols);

                cv::Mat values_mat(rows, cols, CV_32FC1, values.data());
                cv::Mat weights_mat(rows, cols, CV_32FC1, weights.data());
                cv::Mat blurred_values_mat(rows, cols, CV_32FC1, blurred_values.data());
                cv::Mat blurred_weights_mat(rows, cols, CV_32FC1, blurred_weights.data());

                cv::GaussianBlur(values_mat, blurred_values_mat, cv::Size(kernel_size, kernel_size), 0.0, 0.0, cv::BORDER_REPLICATE);
                cv::GaussianBlur(weights_mat, blurred_weights_mat, cv::Size(kernel_size, kernel_size), 0.0, 0.0, cv::BORDER_REPLICATE);

                for (int i = 0; i < rows; ++i) {
                    for (int j = 0; j < cols; ++j) {
                        const float weight = blurred_weights(i, j);
                        layer(i, j) = (weight > 1e-6f) ? (blurred_values(i, j) / weight) : NAN;
                    }
                }
            }
        }

        // Store for the high-rate timer which slides the center between scans.
        {
            std::lock_guard<std::mutex> lock(latest_map_mutex_);
            latest_elevation_map_ = merged;
        }

        auto msg = grid_map::GridMapRosConverter::toMessage(merged);
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_start_time * 1e9));
        pubElevationMap->publish(*msg);
    }

    void featureExtraction::uniformFeatureExtraction(const pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &pc_in, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf, int skip_num, float min_range, float max_range)
    {   
        const float min_range_sq = min_range * min_range;
        const float max_range_sq = max_range > 0.0f ? max_range * max_range : std::numeric_limits<float>::infinity();
        for (uint i=1; i <(int)pc_in->points.size(); i+=skip_num)
        {   
            pcl::PointXYZI point;
            point.x=pc_in->points[i].x;
            point.y=pc_in->points[i].y;
            point.z=pc_in->points[i].z;
            point.intensity=pc_in->points[i].time;

            const float range_sq = pc_in->points[i].x * pc_in->points[i].x +
                                   pc_in->points[i].y * pc_in->points[i].y +
                                   pc_in->points[i].z * pc_in->points[i].z;

            if (((abs(pc_in->points[i].x - pc_in->points[i-1].x) > 1e-7)
                || (abs(pc_in->points[i].y - pc_in->points[i-1].y) > 1e-7)
                || (abs(pc_in->points[i].z - pc_in->points[i-1].z) > 1e-7))
                && range_sq > min_range_sq
                && range_sq < max_range_sq)
            {
                pc_out_surf->push_back(point);
            }
        
        }
        
    }

    ImuMeasurement featureExtraction::parseImuMessage(const sensor_msgs::msg::Imu& msg) {
        ImuMeasurement measurement;
        measurement.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
        measurement.accel << msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z;
        measurement.gyr << msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z;
        measurement.orientation = Eigen::Quaterniond(msg.orientation.w,
                                                msg.orientation.x,
                                                msg.orientation.y,
                                                msg.orientation.z);
        return measurement;
    }

    double featureExtraction::calculateDeltaTime(double current_timestamp) {
        double lastImuTime = 0.0;
        double dt = m_imuPeriod;
        
        if(imuBuf.getLastTime(lastImuTime)) {
            dt = current_timestamp - lastImuTime;
            if(abs(dt - m_imuPeriod) > m_imuPeriod * IMU_TIME_LENIENCY) {
                dt = m_imuPeriod; // IMU timestamp jumped - quietly assume normal delta t
            }
        }
        return dt;
    }

    Imu::Ptr featureExtraction::createImuData(const ImuMeasurement& measurement) {
        Imu::Ptr imudata = std::make_shared<Imu>();
        imudata->time = measurement.timestamp;
        
        // Handle Livox sensor specific processing
        if(IMU_INIT && config_.imu_sensor == SensorType::LIVOX) {
            double gravity = imu_Init->gravity_norm;
            Eigen::Vector3d gyr = imu_Init->imu_laser_R_Gravity * measurement.gyr;
            Eigen::Vector3d accel = imu_Init->imu_laser_R_Gravity * measurement.accel;
            imudata->acc = accel * gravity / imu_Init->acc_mean.norm();
        } else {
            imudata->acc = measurement.accel;
        }
        
        imudata->gyr = measurement.gyr;
        return imudata;
    }

    void featureExtraction::updateImuOrientation(Imu::Ptr& imudata) {
        if (!imuBuf.empty()) {
            const auto& last_imu = imuBuf.measMap_.rbegin()->second;
            const double dt = imudata->time - last_imu->time;
            
            Eigen::Vector3d delta_angle = dt * 0.5 * (imudata->gyr + last_imu->gyr);
            Eigen::Quaterniond delta_r = Sophus::SO3d::exp(delta_angle).unit_quaternion();
            
            imudata->q_w_i = last_imu->q_w_i * delta_r;
            imudata->q_w_i.normalize();
        } else if (config_.use_imu_roll_pitch) {
            tf2::Quaternion orientation_curr(imudata->q_w_i.x(),
                                        imudata->q_w_i.y(),
                                        imudata->q_w_i.z(),
                                        imudata->q_w_i.w());
            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
            
            tf2::Quaternion yaw_quat;
            yaw_quat.setRPY(0, 0, -yaw);
            tf2::Quaternion first_orientation = yaw_quat * orientation_curr;
            
            imudata->q_w_i = Eigen::Quaterniond(first_orientation.w(),
                                            first_orientation.x(),
                                            first_orientation.y(),
                                            first_orientation.z());
        }
    }

    void featureExtraction::imuInitialization(double timestamp) {
        double lidar_first_time = 0;
        if(lidarBuf.getFirstTime(lidar_first_time) && 
            timestamp > lidar_first_time + LIDAR_MESSAGE_TIME + 0.05) {
            
            double first_time = 0.0;
            imuBuf.getFirstTime(first_time);
            
            if (timestamp - first_time > 1.0 && !IMU_INIT) {
                imu_Init->imuInit(imuBuf);
                IMU_INIT = true;
                imuBuf.clean(timestamp);
                RCLCPP_INFO(this->get_logger(), "IMU Initialization Process Finish!");
            }
        }
    }

    void featureExtraction::imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
        m_buf.lock();

        const double imuTime = msg_in->header.stamp.sec + msg_in->header.stamp.nanosec * 1e-9;
        double lastImuTime = 0.0;
        if (imuBuf.getLastTime(lastImuTime)) {
            const double dt = imuTime - lastImuTime;
            if (dt <= 0.0 || dt < IMU_MIN_DT) {
                m_buf.unlock();
                return;
            }
        }
        
        sensor_msgs::msg::Imu imu_msg = *msg_in;
        if (USE_TF_ALIGNMENT) {
            utils::rotate_imu_to_frame(imu_msg, Q_IMU_TO_BASE);
        }
        auto measurement = parseImuMessage(imu_msg);
        
        calculateDeltaTime(measurement.timestamp);
        
        auto imudata = createImuData(measurement);
        
        updateImuOrientation(imudata);
        
        imuBuf.addMeas(imudata, measurement.timestamp);
        
        // only do it at the beginning
        imuInitialization(measurement.timestamp);
        
        m_buf.unlock();
    }

    void featureExtraction::visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry)
    {
        m_buf.lock();
        visualOdomBuf.addMeas(visualOdometry, visualOdometry->header.stamp.sec + visualOdometry->header.stamp.nanosec*1e-9);
        m_buf.unlock();
    }

    void featureExtraction::assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_)
    {
        size_t cloud_size = laserCloudIn_ptr_->size();
        RCLCPP_DEBUG(this->get_logger(), "\n\ninput cloud size: %zu \n",cloud_size);
        pointCloudwithTime.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloudwithTime->reserve(cloud_size);

        point_os::PointcloudXYZITR point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point.x = laserCloudIn_ptr_->points[i].x;
            point.y = laserCloudIn_ptr_->points[i].y;
            point.z = laserCloudIn_ptr_->points[i].z;
            point.intensity = laserCloudIn_ptr_->points[i].intensity;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (config_.N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 32)
            {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = config_.N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                // ROS_BREAK();
            }

            point.ring = scanID;
            float rel_time = (columnTime * int(i / config_.N_SCANS) + laserTime * (i % config_.N_SCANS)) / scanPeriod;
            float pointTime = rel_time * scanPeriod;
            point.time = pointTime;
            pointCloudwithTime->push_back(point);
        }
    }

    void featureExtraction::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {  
        // Check if we should process this frame based on skip count
        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return;

        m_buf.lock();

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());
        
        tmpOusterCloudIn.reset(new pcl::PointCloud<point_os::OusterPointXYZIRT>());

        if (config_.provide_point_time)
        {

            if (config_.lidar_sensor == SensorType::VELODYNE)
            {
                pcl::fromROSMsg(*laserCloudMsg, *pointCloud);

            }
            else if (config_.lidar_sensor == SensorType::OUSTER)
            {
                // Convert to Velodyne format
                pcl::fromROSMsg(*laserCloudMsg, *tmpOusterCloudIn);
                pointCloud->points.resize(tmpOusterCloudIn->size());
                pointCloud->is_dense = tmpOusterCloudIn->is_dense;

                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = pointCloud->points[i];
                    utils::transformOusterPoints(&src, &dst, T_ouster_sensor);  // Convert the ouster points from ouster frame to sensor frame
                    dst.time = src.t * 1e-9f;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"Unknown sensor type: %d", int(lidar_sensor));
                rclcpp::shutdown();
            }
        }
        else
        {
            pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_ptr_);
            assignTimeforPointCloud(laserCloudIn_ptr_);
            pointCloud = pointCloudwithTime;
        }

        manageLidarBuffer(pointCloud, laserCloudMsg->header.stamp.sec + laserCloudMsg->header.stamp.nanosec * 1e-9);

        if(IMU_INIT==true or imuBuf.empty())
        {   
            const bool processed_scan = undistortionAndFeatureExtraction();
            if (processed_scan) {
                double lidar_first_time;
                if (lidarBuf.getFirstTime(lidar_first_time)) {
                    lidarBuf.clean(lidar_first_time);
                }
            }
        }

        m_buf.unlock();
    }


    // void featureExtraction::livoxHandler(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
    // {   
    //     frameCount = frameCount + 1;
    //     if (frameCount % config_.skipFrame != 0)
    //         return; 

    //     m_buf.lock();
        
    //     pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
    //         new pcl::PointCloud<point_os::PointcloudXYZITR>());
        
    //     pointCloud->points.resize(msg->point_num);

    //     Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    //     if (!imuBuf.empty()) {
    //         rotation_matrix = imu_Init->imu_laser_R_Gravity;
    //     } 
        
    //     if(config_.provide_point_time) {     
    //         for (uint i=0; i < msg->point_num; i++) {
    //             if ((msg->points[i].line < config_.N_SCANS) &&
    //                 ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {   
    //                 Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
    //                 Eigen::Vector3d transformed_point = rotation_matrix * point;
    //                 pointCloud->points[i].x = transformed_point.x();
    //                 pointCloud->points[i].y = transformed_point.y();
    //                 pointCloud->points[i].z = transformed_point.z();
    //                 pointCloud->points[i].intensity = msg->points[i].reflectivity;
    //                 pointCloud->points[i].time = msg->points[i].offset_time / float(1000000000);
    //                 pointCloud->points[i].ring = msg->points[i].line;
    //             }
    //         }
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "Please check yaml or livox driver to provide the timestamp for each point");
    //         rclcpp::shutdown();
    //     }

    //     manageLidarBuffer(pointCloud, msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);

    //     if(IMU_INIT==true or imuBuf.empty())
    //     {   
    //         undistortionAndFeatureExtraction();
    //         double lidar_first_time;
    //         lidarBuf.getFirstTime(lidar_first_time);
    //         lidarBuf.clean(lidar_first_time);
    //     }

    //     m_buf.unlock();
    // }

    void featureExtraction::livoxHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0) {
            return;
        }

        m_buf.lock();

        if (!config_.provide_point_time) {
            RCLCPP_ERROR(this->get_logger(),
                         "livox_pcl2 requires per-point timestamps (set provide_point_time=1 and ensure the driver publishes the `timestamp` field).");
            rclcpp::shutdown();
            return;
        }

        if (msg->is_bigendian) {
            RCLCPP_ERROR(this->get_logger(), "Big-endian PointCloud2 is not supported.");
            rclcpp::shutdown();
            return;
        }

        const std::size_t num_points = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloud->points.reserve(num_points);
        pointCloud->is_dense = false;

        auto has_field = [&](const char* name) {
            for (const auto& f : msg->fields) {
                if (f.name == name) {
                    return true;
                }
            }
            return false;
        };
        if (!has_field("x") || !has_field("y") || !has_field("z") ||
            !has_field("intensity") || !has_field("tag") || !has_field("line") || !has_field("timestamp")) {
            RCLCPP_ERROR(this->get_logger(),
                         "livox_pcl2 expects fields: x,y,z,intensity,tag,line,timestamp (got a different PointCloud2 layout).");
            rclcpp::shutdown();
            return;
        }

        const double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        const std::int64_t stamp_ns =
            static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(msg->header.stamp.nanosec);

        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        if (!imuBuf.empty()) {
            rotation_matrix = imu_Init->imu_laser_R_Gravity;
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_tag(*msg, "tag");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(*msg, "line");
        sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp(*msg, "timestamp");

        for (std::size_t i = 0; i < num_points;
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag, ++iter_line, ++iter_timestamp) {
            const uint8_t line = *iter_line;
            if (line >= static_cast<uint8_t>(config_.N_SCANS)) {
                continue;
            }
            const uint8_t tag = *iter_tag;
            if (!(((tag & 0x30) == 0x10) || ((tag & 0x30) == 0x00))) {
                continue;
            }

            const std::int64_t point_time_ns = static_cast<std::int64_t>(*iter_timestamp);
            double point_time_sec = (point_time_ns - stamp_ns) * 1e-9;
            if (point_time_sec < 0.0) {
                point_time_sec = 0.0;
            }

            Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
            Eigen::Vector3d transformed_point = rotation_matrix * point;

            point_os::PointcloudXYZITR out;
            out.x = static_cast<float>(transformed_point.x());
            out.y = static_cast<float>(transformed_point.y());
            out.z = static_cast<float>(transformed_point.z());
            out.intensity = *iter_intensity;
            out.time = static_cast<float>(point_time_sec);
            out.ring = line;
            pointCloud->points.push_back(out);
        }

        if (pointCloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "livox_pcl2: received an empty/invalid cloud (no points passed filters).");
            m_buf.unlock();
            return;
        }

        pointCloud->width = static_cast<uint32_t>(pointCloud->points.size());
        pointCloud->height = 1;

        manageLidarBuffer(pointCloud, stamp_sec);

        if(IMU_INIT==true or imuBuf.empty())
        {   
            const bool processed_scan = undistortionAndFeatureExtraction();
            if (processed_scan) {
                double lidar_first_time;
                if (lidarBuf.getFirstTime(lidar_first_time)) {
                    lidarBuf.clean(lidar_first_time);
                }
            }
        }

        m_buf.unlock();
    }

    void featureExtraction::manageLidarBuffer(
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud, 
        double timestamp)
    {
        // Check buffer size and drop oldest frames if necessary
        std::size_t curLidarBufferSize = lidarBuf.getSize();
        
        while (curLidarBufferSize >= 50) {
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
            RCLCPP_WARN(this->get_logger(), "Lidar buffer too large, dropping frame");
            curLidarBufferSize = lidarBuf.getSize();
        }
        
        // Add measurement to buffer
        lidarBuf.addMeas(pointCloud, timestamp);
    }

} // namespace super_odometry
