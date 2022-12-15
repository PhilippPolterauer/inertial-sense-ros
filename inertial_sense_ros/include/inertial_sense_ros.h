#pragma once

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include "InertialSense.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "inertial_sense_msgs/msg/gps.hpp"
#include "data_sets.h"
#include "inertial_sense_msgs/msg/gps_info.hpp"
#include "inertial_sense_msgs/msg/pre_int_imu.hpp"
#include "inertial_sense_msgs/srv/firmware_update.hpp"
#include "inertial_sense_msgs/srv/ref_lla_update.hpp"
#include "inertial_sense_msgs/msg/rtk_rel.hpp"
#include "inertial_sense_msgs/msg/rtk_info.hpp"
#include "inertial_sense_msgs/msg/gnss_ephemeris.hpp"
#include "inertial_sense_msgs/msg/glonass_ephemeris.hpp"
#include "inertial_sense_msgs/msg/gnss_observation.hpp"
#include "inertial_sense_msgs/msg/gnss_obs_vec.hpp"
#include "inertial_sense_msgs/msg/inl2_states.hpp"
#include "inertial_sense_msgs/msg/didins1.hpp"
#include "inertial_sense_msgs/msg/didins2.hpp"
#include "inertial_sense_msgs/msg/didins4.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ISConstants.h"
// #include "geometry/xform.h"

#define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
#define LEAP_SECONDS 18           // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
#define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)
#define FIRMWARE_VERSION_CHAR0 1
#define FIRMWARE_VERSION_CHAR1 9
#define FIRMWARE_VERSION_CHAR2 0

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple)                               \
    IS_.BroadcastBinaryData(DID, __periodmultiple,                                          \
                            [this](InertialSense *i, p_data_t *data, int pHandle)           \
                            {                                                               \
                                /* ROS_INFO("Got message %d", DID);*/                       \
                                this->__cb_fun(DID, reinterpret_cast<__type *>(data->buf)); \
                            })

class InertialSenseROS //: SerialListener
{
    using TriggerReq = std_srvs::srv::Trigger::Request::SharedPtr;
    using TriggerRes = std_srvs::srv::Trigger::Response::SharedPtr;
    using RefllaReq = inertial_sense_msgs::srv::RefLLAUpdate::Request::SharedPtr;
    using RefllaRes = inertial_sense_msgs::srv::RefLLAUpdate::Response::SharedPtr;
    using FirmwareUpdateReq = inertial_sense_msgs::srv::FirmwareUpdate::Request::ConstSharedPtr;
    using FirmwareUpdateRes = inertial_sense_msgs::srv::FirmwareUpdate::Response::SharedPtr;

public:
    typedef enum
    {
        NMEA_GPGGA = 0x01,
        NMEA_GPGLL = 0x02,
        NMEA_GPGSA = 0x04,
        NMEA_GPRMC = 0x08,
        NMEA_SER0 = 0x01,
        NMEA_SER1 = 0x02
    } NMEA_message_config_t;

    InertialSenseROS(YAML::Node paramNode = YAML::Node(YAML::NodeType::Undefined), bool configFlashParameters = true);
    void callback(p_data_t *data);
    void update();

    void load_params_srv();
    void load_params_yaml(YAML::Node node);
    template <typename Type>
    bool get_node_param_yaml(YAML::Node node, const std::string key, Type &val);
    template <typename Derived1>
    bool get_node_vector_yaml(YAML::Node node, const std::string key, int size, Derived1 &val);
    void connect();
    bool firmware_compatiblity_check();
    void set_navigation_dt_ms();
    void configure_flash_parameters();
    void configure_rtk();
    void connect_rtk_client(const std::string &RTK_correction_protocol, const std::string &RTK_server_IP, const int RTK_server_port);
    void start_rtk_server(const std::string &RTK_server_IP, const int RTK_server_port);

    void configure_data_streams(bool startup);
    void configure_data_streams();
    void configure_ascii_output();
    void start_log();

    template <typename T>
    void get_vector_flash_config(std::string param_name, uint32_t size, T &data);
    // void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
    void get_flash_config();
    void reset_device();
    void flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg);
    bool flashConfigStreaming_ = false;
    // Serial Port Configuration
    std::string port_ = "/dev/ttyACM0";
    int baudrate_ = 921600;
    bool initialized_;
    bool log_enabled_ = false;
    bool covariance_enabled_ = false;

    std::string frame_id_ = "body";

    // ROS Stream handling
    template <typename T> struct ros_stream_t
    {
        // typedef rclcpp::Publisher<T>::SharedPtr pubT;
        bool enabled = false;
        typename rclcpp::Publisher<T>::SharedPtr (pub);
        int period_multiple = 1;
    };

    typedef struct
    {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_msgs::msg::RTKRel>::SharedPtr pub_rel;
        rclcpp::Publisher<inertial_sense_msgs::msg::RTKInfo>::SharedPtr pub_info;
        int period_multiple = 1;
    } ros_stream_rtk_t;

    typedef struct
    {
        bool enabled = false;
        rclcpp::Publisher<inertial_sense_msgs::msg::GNSSObsVec>::SharedPtr pub_obs;
        rclcpp::Publisher<inertial_sense_msgs::msg::GNSSEphemeris>::SharedPtr pub_eph;
        rclcpp::Publisher<inertial_sense_msgs::msg::GlonassEphemeris>::SharedPtr pub_geph;
        int period_multiple = 1;
    } ros_stream_gpsraw_t;

    tf2_ros::TransformBroadcaster br;
    bool publishTf_ = true;
    geometry_msgs::msg::TransformStamped transform_NED_;
    geometry_msgs::msg::TransformStamped transform_ENU_;
    geometry_msgs::msg::TransformStamped transform_ECEF_;
    enum
    {
        NED,
        ENU
    } ltcf;

    rclcpp::Logger logger_;
    rclcpp::PublisherBase::SharedPtr did_ins_1_pub_;
    rclcpp::PublisherBase::SharedPtr did_ins_2_pub_;
    rclcpp::PublisherBase::SharedPtr odom_ins_ned_pub_;
    rclcpp::PublisherBase::SharedPtr odom_ins_ecef_pub_;
    rclcpp::PublisherBase::SharedPtr odom_ins_enu_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr strobe_pub_;
    rclcpp::TimerBase::SharedPtr obs_bundle_timer_;
    rclcpp::Time last_obs_time_1_;
    rclcpp::Time last_obs_time_2_;
    rclcpp::Time last_obs_time_base_;
    rclcpp::TimerBase::SharedPtr data_stream_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    inertial_sense_msgs::msg::GNSSObsVec gps1_obs_Vec_;
    inertial_sense_msgs::msg::GNSSObsVec gps2_obs_Vec_;
    inertial_sense_msgs::msg::GNSSObsVec base_obs_Vec_;

    bool rtk_connecting_ = false;
    int RTK_connection_attempt_limit_ = 1;
    int RTK_connection_attempt_backoff_ = 2;
    int rtk_traffic_total_byte_count_ = 0;
    int rtk_data_transmission_interruption_count_ = 0;
    bool rtk_connectivity_watchdog_enabled_ = true;
    float rtk_connectivity_watchdog_timer_frequency_ = 1;
    int rtk_data_transmission_interruption_limit_ = 5;
    std::string RTK_server_mount_ = "";
    std::string RTK_server_username_ = "";
    std::string RTK_server_password_ = "";
    std::string RTK_correction_protocol_ = "RTCM3";
    std::string RTK_server_IP_ = "127.0.0.1";
    int RTK_server_port_ = 7777;
    bool RTK_rover_ = false;
    bool RTK_rover_radio_enable_ = false;
    bool RTK_base_USB_ = false;
    bool RTK_base_serial_ = false;
    bool RTK_base_TCP_ = false;
    bool GNSS_Compass_ = false;

    std::string gps1_type_ = "F9P";
    std::string gps1_topic_ = "gps1";
    std::string gps2_type_ = "F9P";
    std::string gps2_topic_ = "gps2";

    rclcpp::TimerBase::SharedPtr rtk_connectivity_watchdog_timer_;
    void start_rtk_connectivity_watchdog_timer();
    void stop_rtk_connectivity_watchdog_timer();
    void rtk_connectivity_watchdog_timer_callback();

    void INS1_callback(eDataIDs DID, const ins_1_t *const msg);
    void INS2_callback(eDataIDs DID, const ins_2_t *const msg);
    void INS4_callback(eDataIDs DID, const ins_4_t *const msg);
    void INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg);
    void INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t *const msg);
    void odom_ins_ned_callback(eDataIDs DID, const ins_2_t *const msg);
    void odom_ins_ecef_callback(eDataIDs DID, const ins_2_t *const msg);
    void odom_ins_enu_callback(eDataIDs DID, const ins_2_t *const msg);
    void GPS_info_callback(eDataIDs DID, const gps_sat_t *const msg);
    void mag_callback(eDataIDs DID, const magnetometer_t *const msg);
    void baro_callback(eDataIDs DID, const barometer_t *const msg);
    void preint_IMU_callback(eDataIDs DID, const pimu_t *const msg);
    void strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg);
    void diagnostics_callback();
    void GPS_pos_callback(eDataIDs DID, const gps_pos_t *const msg);
    void GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg);
    void GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg);
    void GPS_obs_callback(eDataIDs DID, const obsd_t *const msg, int nObs);
    void GPS_obs_bundle_timer_callback();
    void GPS_eph_callback(eDataIDs DID, const eph_t *const msg);
    void GPS_geph_callback(eDataIDs DID, const geph_t *const msg);
    void RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg);
    void RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg);

    float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;
    uint diagnostic_fix_type_;

    ros_stream_t<inertial_sense_msgs::msg::DIDINS1> DID_INS_1_;
    ros_stream_t<inertial_sense_msgs::msg::DIDINS2> DID_INS_2_;
    ros_stream_t<inertial_sense_msgs::msg::DIDINS4> DID_INS_4_;
    ros_stream_t<inertial_sense_msgs::msg::INL2States> INL2_states_;
    ros_stream_t<nav_msgs::msg::Odometry> odom_ins_ned_;
    ros_stream_t<nav_msgs::msg::Odometry> odom_ins_ecef_;
    ros_stream_t<nav_msgs::msg::Odometry> odom_ins_enu_;
    ros_stream_t<sensor_msgs::msg::Imu> IMU_;
    ros_stream_t<sensor_msgs::msg::MagneticField> mag_;
    ros_stream_t<sensor_msgs::msg::FluidPressure> baro_;
    ros_stream_t<inertial_sense_msgs::msg::PreIntIMU> preint_IMU_;
    ros_stream_t<diagnostic_msgs::msg::DiagnosticArray> diagnostics_;
    ros_stream_t<inertial_sense_msgs::msg::GPS> GPS1_;
    ros_stream_t<inertial_sense_msgs::msg::GPSInfo> GPS1_info_;
    ros_stream_gpsraw_t GPS1_raw_;
    ros_stream_t<inertial_sense_msgs::msg::GPS> GPS2_;
    ros_stream_t<inertial_sense_msgs::msg::GPSInfo> GPS2_info_;
    ros_stream_gpsraw_t GPS2_raw_;
    ros_stream_gpsraw_t GPS_base_raw_;
    ros_stream_rtk_t RTK_pos_;
    ros_stream_rtk_t RTK_cmp_;
    ros_stream_t<sensor_msgs::msg::NavSatFix> NavSatFix_;
    bool NavSatFixConfigured = false;
    int gps_raw_period_multiple = 1;
    int gps_info_period_multiple = 1;

    bool ins1Streaming_ = false;
    bool ins2Streaming_ = false;
    bool ins4Streaming_ = false;
    bool inl2StatesStreaming_ = false;
    bool insCovarianceStreaming_ = false;
    bool magStreaming_ = false;
    bool baroStreaming_ = false;
    bool preintImuStreaming_ = false;
    bool imuStreaming_ = false;
    bool strobeInStreaming_ = false;
    bool diagnosticsStreaming_ = false;
    // NOTE: that GPS streaming flags are applicable for all GPS devices/receivers
    bool gps1PosStreaming_ = false;
    bool gps1VelStreaming_ = false;
    bool gps2PosStreaming_ = false;
    bool gps2VelStreaming_ = false;
    bool gps1RawStreaming_ = false;
    bool gps2RawStreaming_ = false;
    bool gps1InfoStreaming_ = false;
    bool gps2InfoStreaming_ = false;
    bool rtkPosMiscStreaming_ = false;
    bool rtkPosRelStreaming_ = false;
    bool rtkCmpMiscStreaming_ = false;
    bool rtkCmpRelStreaming_ = false;
    bool data_streams_enabled_ = false;

    // Services
    rclcpp::ServiceBase::SharedPtr mag_cal_srv_;
    rclcpp::ServiceBase::SharedPtr multi_mag_cal_srv_;
    rclcpp::ServiceBase::SharedPtr firmware_update_srv_;
    rclcpp::ServiceBase::SharedPtr refLLA_set_current_srv_;
    rclcpp::ServiceBase::SharedPtr refLLA_set_value_srv_;
    void set_current_position_as_refLLA(TriggerReq req, TriggerRes res);
    void set_refLLA_to_value(RefllaReq req, RefllaRes res);
    void perform_mag_cal_srv_callback(TriggerReq req, TriggerRes res);
    void perform_multi_mag_cal_srv_callback(TriggerReq req, TriggerRes res);
    void update_firmware_srv_callback(FirmwareUpdateReq req, FirmwareUpdateRes res);

    void publishGPS1();
    void publishGPS2();

    enum PositionCovarianceType
    {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN = 3
    };

    enum NavSatFixStatusFixType
    {
        STATUS_NO_FIX = -1,  // unable to fix position
        STATUS_FIX = 0,      // unaugmented fix
        STATUS_SBAS_FIX = 1, // with satellite-based augmentation
        STATUS_GBAS_FIX = 2  // with ground-based augmentation
    };

    // Bits defining which Global Navigation Satellite System signals were used by the receiver.
    enum NavSatFixService
    {
        SERVICE_GPS = 0x1,
        SERVICE_GLONASS = 0x2,
        SERVICE_COMPASS = 0x4, // includes BeiDou.
        SERVICE_GALILEO = 0x8
    };

    rclcpp::Time now();

    /**
     * @brief ros_time_from_week_and_tow
     * Get current ROS time from week and tow
     * @param week Weeks since January 6th, 1980
     * @param timeofweek Time of week (since Sunday morning) in seconds, GMT
     * @return equivalent ros::Time
     */
    rclcpp::Time ros_time_from_week_and_tow(const uint32_t week, const double timeofweek);

    /**
     * @brief ros_time_from_start_time
     * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
     * @return equivalent rclcpp::Time
     */
    rclcpp::Time ros_time_from_start_time(const double time);

    /**
     * @brief ros_time_from_tow
     * Get equivalent ros time from tow and internal week counter
     * @param tow Time of Week (seconds)
     * @return equivalent rclcpp::Time
     */
    rclcpp::Time ros_time_from_tow(const double tow);

    double tow_from_ros_time(const rclcpp::Time &rt);
    rclcpp::Time ros_time_from_gtime(const uint64_t sec, double subsec);

    double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS
                               //  If this number is 0, then we have not yet got a fix
    uint64_t GPS_week_ = 0;    // Week number to start of GPS_towOffset_ in GPS time
    // Time sync variables
    double INS_local_offset_ = 0.0;  // Current estimate of the uINS start time in ROS time seconds
    bool got_first_message_ = false; // Flag to capture first uINS start time guess

    /**
     * @brief LD2Cov
     * Transform array of covariance lower diagonals itno the full covariance matrix
     * @param LD array of lower diagonals
     * @param Cov full covariance matrix
     * @param width size (width or height) of the covariance matrix
     */
    void LD2Cov(const float *LD, float *Cov, int width);

    /**
     * @brief rotMatB2R
     * Make a rotation matrix body-to-reference from quaternion
     * @param quat attitude quaternion (rotation from the reference frame to body)
     * @param R rotation matrix body-to-reference
     */
    void rotMatB2R(const ixVector4 quat, ixMatrix3 R);

    /**
     * @brief transform_6x6_covariance
     * Transform covariance matrix due to the change of coordinates, such that
     * the fisrt 3 coordinates are rotated by R1 and the last 3 coordinates are rotated by R2
     * @param Pout output covariance matrix (in the new coordinates)
     * @param Pin  input covariance matrix (in the old coordinates)
     * @param R1   rotation matrix describing transformation of the first 3 coordinates
     * @param R2   rotation matrix describing transformation of the last 3 coordinates
     */
    void transform_6x6_covariance(float Pout[36], float Pin[36], ixMatrix3 R1, ixMatrix3 R2);

    // Data to hold on to in between callbacks
    double lla_[3];
    double ecef_[3];
    sensor_msgs::msg::Imu imu_msg;
    nav_msgs::msg::Odometry ned_odom_msg;
    nav_msgs::msg::Odometry ecef_odom_msg;
    nav_msgs::msg::Odometry enu_odom_msg;
    sensor_msgs::msg::NavSatFix NavSatFix_msg;
    inertial_sense_msgs::msg::GPS gps1_msg;
    geometry_msgs::msg::Vector3Stamped gps1_velecef;
    inertial_sense_msgs::msg::GPSInfo gps_info_msg;
    inertial_sense_msgs::msg::GPS gps2_msg;
    geometry_msgs::msg::Vector3Stamped gps2_velecef;
    inertial_sense_msgs::msg::GPSInfo gps2_info_msg;
    inertial_sense_msgs::msg::INL2States inl2_states_msg;
    inertial_sense_msgs::msg::DIDINS1 did_ins_1_msg;
    inertial_sense_msgs::msg::DIDINS2 did_ins_2_msg;
    inertial_sense_msgs::msg::DIDINS4 did_ins_4_msg;
    inertial_sense_msgs::msg::PreIntIMU preintIMU_msg;

    float poseCov[36], twistCov[36];

    rclcpp::Node nh_;
    rclcpp::Node nh_private_;

    // Connection to the uINS
    InertialSense IS_;

    // Flash parameters

    int navigation_dt_ms_ = 4;
    float insRotation_[3] = {0, 0, 0};
    float insOffset_[3] = {0, 0, 0};
    float gps1AntOffset_[3] = {0, 0, 0};
    float gps2AntOffset_[3] = {0, 0, 0};
    double refLla_[3] = {0, 0, 0};
    float magInclination_ = 0;
    float magDeclination_ = 0;
    int insDynModel_ = INS_DYN_MODEL_AIRBORNE_4G;
    bool refLLA_known = false;
    int ioConfig_ = 39624800; // F9P RUG2 RTK CMP: 0x025ca060
};
