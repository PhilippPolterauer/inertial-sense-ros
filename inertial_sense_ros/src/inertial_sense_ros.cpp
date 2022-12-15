#include "inertial_sense_ros.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <ISPose.h>
#include <ISEarth.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ISMatrix.h"
#include "ISEarth.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

template <typename FunctionT>
auto bind_membercb_service(FunctionT f, InertialSenseROS *obj)
{
    return std::bind(f, obj, std::placeholders::_1, std::placeholders::_2);
}

InertialSenseROS::InertialSenseROS(YAML::Node paramNode, bool configFlashParameters) : nh_("inertial_sense"),
                                                                                       nh_private_("~"),
                                                                                       logger_(nh_.get_logger()),
                                                                                       initialized_(false),
                                                                                       br(nh_)
{
    // rtk_connectivity_watchdog_timer_()

    if (paramNode.IsDefined())
    {
        load_params_yaml(paramNode);
        RCLCPP_INFO(logger_, "Using Yaml file parameters.\n\n");
    }
    else
    {
        load_params_srv();
        RCLCPP_INFO(logger_, "Using parameter server.\n\n");
    }
    connect();

    // Check protocol and firmware version
    if (!firmware_compatiblity_check())
    {
        RCLCPP_FATAL(logger_, "Protocol version of ROS node does not match device protocol!");
    }

    // Start Up ROS service servers
    refLLA_set_current_srv_ = nh_.create_service<inertial_sense_msgs::srv::RefLLAUpdate>("set_refLLA_current", std::bind(&InertialSenseROS::set_current_position_as_refLLA, this, std::placeholders::_1, std::placeholders::_2));
    refLLA_set_value_srv_ = nh_.create_service<inertial_sense_msgs::srv::RefLLAUpdate>("set_refLLA_value", std::bind(&InertialSenseROS::set_refLLA_to_value, this, std::placeholders::_1, std::placeholders::_2));
    mag_cal_srv_ = nh_.create_service<std_srvs::srv::Trigger>("single_axis_mag_cal", bind_membercb_service(&InertialSenseROS::perform_mag_cal_srv_callback, this));
    multi_mag_cal_srv_ = nh_.create_service<std_srvs::srv::Trigger>("multi_axis_mag_cal", bind_membercb_service(&InertialSenseROS::perform_multi_mag_cal_srv_callback, this));
    // firmware_update_srv_ = nh_.advertiseService("firmware_update", &InertialSenseROS::update_firmware_srv_callback, this);
    // data_stream_timer_ = nh_.create_wall_timer(std::chrono::duration(1*std::chrono_literals::s()), configure_data_streams, this); // 2 Hz
    if (diagnostics_.enabled)
    {
        diagnostics_.pub = nh_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
        diagnostics_timer_ = nh_.create_wall_timer(0.5s, [this]()
                                                   { diagnostics_callback(); }); // 2 Hz
    }

    IS_.StopBroadcasts(true);
    configure_data_streams(true);
    configure_rtk();
    IS_.SavePersistent();

    if (configFlashParameters)
    { // Set uINS flash parameters after everything thing else so uINS flash write processor stall doesn't interfere.
        configure_flash_parameters();
    }

    if (log_enabled_)
    {
        start_log(); // Start log should always happen last.
    }

    //  configure_ascii_output(); //does not work right now

    initialized_ = true;
}

void InertialSenseROS::load_params_yaml(YAML::Node node)
{
    RCLCPP_INFO(logger_, ("Load YAML server"));
    get_node_param_yaml(node, "port", port_);
    get_node_param_yaml(node, "navigation_dt_ms", navigation_dt_ms_);
    get_node_param_yaml(node, "baudrate", baudrate_);
    get_node_param_yaml(node, "frame_id", frame_id_);
    get_node_param_yaml(node, "stream_DID_INS_1", DID_INS_1_.enabled);
    get_node_param_yaml(node, "ins1_period_multiple", DID_INS_1_.period_multiple);
    get_node_param_yaml(node, "stream_DID_INS_2", DID_INS_2_.enabled);
    get_node_param_yaml(node, "ins2_period_multiple", DID_INS_2_.period_multiple);
    get_node_param_yaml(node, "stream_DID_INS_4", DID_INS_4_.enabled);
    get_node_param_yaml(node, "ins4_period_multiple", DID_INS_4_.period_multiple);
    get_node_param_yaml(node, "stream_odom_ins_ned", odom_ins_ned_.enabled);
    get_node_param_yaml(node, "odom_ins_ned_period_multiple", odom_ins_ned_.period_multiple);
    get_node_param_yaml(node, "stream_odom_ins_enu", odom_ins_enu_.enabled);
    get_node_param_yaml(node, "odom_ins_enu_period_multiple", odom_ins_enu_.period_multiple);
    get_node_param_yaml(node, "stream_odom_ins_ecef", odom_ins_ecef_.enabled);
    get_node_param_yaml(node, "odom_ins_ecef_period_multiple", odom_ins_ecef_.period_multiple);
    get_node_param_yaml(node, "stream_covariance_data", covariance_enabled_);
    get_node_param_yaml(node, "stream_INL2_states", INL2_states_.enabled);
    get_node_param_yaml(node, "INL2_states_period_multiple", INL2_states_.period_multiple);
    get_node_param_yaml(node, "stream_IMU", IMU_.enabled);
    get_node_param_yaml(node, "imu_period_multiple", IMU_.period_multiple);
    get_node_param_yaml(node, "stream_GPS1", GPS1_.enabled);
    get_node_param_yaml(node, "gps1_period_multiple", GPS1_.period_multiple);
    get_node_param_yaml(node, "stream_GPS2", GPS2_.enabled);
    get_node_param_yaml(node, "gps2_period_multiple", GPS2_.period_multiple);
    get_node_param_yaml(node, "stream_GPS_raw", GPS1_raw_.enabled);
    get_node_param_yaml(node, "stream_GPS_raw", GPS1_raw_.enabled);
    get_node_param_yaml(node, "gps_raw_period_multiple", gps_raw_period_multiple);
    get_node_param_yaml(node, "stream_GPS_info", GPS1_info_.enabled);
    get_node_param_yaml(node, "stream_GPS_info", GPS2_info_.enabled);
    get_node_param_yaml(node, "gps_info_period_multiple", gps_info_period_multiple);
    get_node_param_yaml(node, "GPS1_type", gps1_type_);
    get_node_param_yaml(node, "GPS1_topic", gps1_topic_);
    get_node_param_yaml(node, "GPS2_type", gps2_type_);
    get_node_param_yaml(node, "GPS2_topic", gps2_topic_);
    get_node_param_yaml(node, "stream_NavSatFix", NavSatFix_.enabled);
    get_node_param_yaml(node, "NavSatFix_period_multiple", NavSatFix_.period_multiple);
    get_node_param_yaml(node, "stream_mag", mag_.enabled);
    get_node_param_yaml(node, "mag_period_multiple", mag_.period_multiple);
    get_node_param_yaml(node, "stream_baro", baro_.enabled);
    get_node_param_yaml(node, "baro_period_multiple", baro_.period_multiple);
    get_node_param_yaml(node, "stream_preint_IMU", preint_IMU_.enabled);
    get_node_param_yaml(node, "preint_imu_period_multiple", preint_IMU_.period_multiple);
    get_node_param_yaml(node, "stream_diagnostics", diagnostics_.enabled);
    get_node_param_yaml(node, "diagnostics_period_multiple", diagnostics_.period_multiple);
    get_node_param_yaml(node, "publishTf", publishTf_);
    get_node_param_yaml(node, "enable_log", log_enabled_);
    get_node_param_yaml(node, "ioConfig", ioConfig_);
    get_node_param_yaml(node, "RTK_server_mount", RTK_server_mount_);
    get_node_param_yaml(node, "RTK_server_username", RTK_server_username_);
    get_node_param_yaml(node, "RTK_server_password", RTK_server_password_);
    get_node_param_yaml(node, "RTK_connection_attempt_limit", RTK_connection_attempt_limit_);
    get_node_param_yaml(node, "RTK_connection_attempt_backoff", RTK_connection_attempt_backoff_);
    //     // default is false for legacy compatibility
    get_node_param_yaml(node, "RTK_connectivity_watchdog_enabled", rtk_connectivity_watchdog_enabled_);
    get_node_param_yaml(node, "RTK_connectivity_watchdog_timer_frequency", rtk_connectivity_watchdog_timer_frequency_);
    get_node_param_yaml(node, "RTK_data_transmission_interruption_limit", rtk_data_transmission_interruption_limit_);
    get_node_param_yaml(node, "RTK_correction_protocol", RTK_correction_protocol_);
    get_node_param_yaml(node, "RTK_server_IP", RTK_server_IP_);
    get_node_param_yaml(node, "RTK_server_port", RTK_server_port_);
    get_node_param_yaml(node, "RTK_rover", RTK_rover_);
    get_node_param_yaml(node, "RTK_pos_period_multiple", RTK_pos_.period_multiple);
    get_node_param_yaml(node, "RTK_rover_radio_enable", RTK_rover_radio_enable_);
    get_node_param_yaml(node, "RTK_base_USB", RTK_base_USB_);
    get_node_param_yaml(node, "RTK_base_serial", RTK_base_serial_);
    get_node_param_yaml(node, "RTK_base_TCP", RTK_base_TCP_);
    get_node_param_yaml(node, "GNSS_Compass", GNSS_Compass_);
    get_node_param_yaml(node, "RTK_cmp_period_multiple", RTK_cmp_.period_multiple);
    get_node_param_yaml(node, "inclination", magInclination_);
    get_node_param_yaml(node, "declination", magDeclination_);
    get_node_param_yaml(node, "dynamic_model", insDynModel_);

    // Params with arrays
    get_node_vector_yaml(node, "INS_rpy_radians", 3, insRotation_);
    get_node_vector_yaml(node, "INS_xyz", 3, insOffset_);
    get_node_vector_yaml(node, "GPS_ant1_xyz", 3, gps1AntOffset_);
    get_node_vector_yaml(node, "GPS_ant2_xyz", 3, gps2AntOffset_);
    get_node_vector_yaml(node, "GPS_ref_lla", 3, refLla_);
}

void InertialSenseROS::load_params_srv()
{
    RCLCPP_INFO(logger_, ("Load Param Server"));
    printf("\n\nLoading Params");
    nh_private_.get_parameter("port", port_);
    nh_private_.get_parameter("navigation_dt_ms", navigation_dt_ms_);
    nh_private_.get_parameter("baudrate", baudrate_);
    nh_private_.get_parameter("frame_id", frame_id_);
    nh_private_.get_parameter_or("stream_DID_INS_1", DID_INS_1_.enabled, true);
    nh_private_.get_parameter("ins1_period_multiple", DID_INS_1_.period_multiple);
    nh_private_.get_parameter("stream_DID_INS_2", DID_INS_2_.enabled);
    nh_private_.get_parameter("ins2_period_multiple", DID_INS_2_.period_multiple);
    nh_private_.get_parameter("stream_DID_INS_4", DID_INS_4_.enabled);
    nh_private_.get_parameter("ins4_period_multiple", DID_INS_4_.period_multiple);
    nh_private_.get_parameter("stream_odom_ins_ned", odom_ins_ned_.enabled);
    nh_private_.get_parameter("odom_ins_ned_period_multiple", odom_ins_ned_.period_multiple);
    nh_private_.get_parameter("stream_odom_ins_enu", odom_ins_enu_.enabled);
    nh_private_.get_parameter("odom_ins_enu_period_multiple", odom_ins_enu_.period_multiple);
    nh_private_.get_parameter("stream_odom_ins_ecef", odom_ins_ecef_.enabled);
    nh_private_.get_parameter("odom_ins_ecef_period_multiple", odom_ins_ecef_.period_multiple);
    nh_private_.get_parameter("stream_covariance_data", covariance_enabled_);
    nh_private_.get_parameter("stream_INL2_states", INL2_states_.enabled);
    nh_private_.get_parameter("INL2_states_period_multiple", INL2_states_.period_multiple);
    nh_private_.get_parameter("stream_IMU", IMU_.enabled);
    nh_private_.get_parameter("imu_period_multiple", IMU_.period_multiple);
    nh_private_.get_parameter_or("stream_GPS1", GPS1_.enabled, true);
    nh_private_.get_parameter("gps1_period_multiple", GPS1_.period_multiple);
    nh_private_.get_parameter_or("stream_GPS2", GPS2_.enabled, true);
    nh_private_.get_parameter("gps2_period_multiple", GPS2_.period_multiple);
    nh_private_.get_parameter("stream_GPS_raw", GPS1_raw_.enabled);
    nh_private_.get_parameter("stream_GPS_raw", GPS2_raw_.enabled);
    nh_private_.get_parameter("gps_raw_period_multiple", gps_raw_period_multiple);
    nh_private_.get_parameter("stream_GPS1_info", GPS1_info_.enabled);
    nh_private_.get_parameter("stream_GPS2_info", GPS2_info_.enabled);
    nh_private_.get_parameter("gps_info_period_multiple", gps_info_period_multiple);
    nh_private_.get_parameter("GPS1_topic", gps1_topic_);
    nh_private_.get_parameter("GPS2_topic", gps2_topic_);
    nh_private_.get_parameter("stream_NavSatFix", NavSatFix_.enabled);
    nh_private_.get_parameter("NavSatFix_period_multiple", NavSatFix_.period_multiple);
    nh_private_.get_parameter("stream_mag", mag_.enabled);
    nh_private_.get_parameter("mag_period_multiple", mag_.period_multiple);
    nh_private_.get_parameter("stream_baro", baro_.enabled);
    nh_private_.get_parameter("baro_period_multiple", baro_.period_multiple);
    nh_private_.get_parameter("stream_preint_IMU", preint_IMU_.enabled);
    nh_private_.get_parameter("preint_imu_period_multiple", preint_IMU_.period_multiple);
    nh_private_.get_parameter("stream_diagnostics", diagnostics_.enabled);
    nh_private_.get_parameter("diagnostics_period_multiple", diagnostics_.period_multiple);
    nh_private_.get_parameter("publishTf", publishTf_);
    nh_private_.get_parameter("ioConfig", ioConfig_);
    nh_private_.get_parameter("enable_log", log_enabled_);
    nh_private_.get_parameter("RTK_server_mount", RTK_server_mount_);
    nh_private_.get_parameter("RTK_server_username", RTK_server_username_);
    nh_private_.get_parameter("RTK_server_password", RTK_server_password_);
    nh_private_.get_parameter("RTK_connection_attempt_limit", RTK_connection_attempt_limit_);
    nh_private_.get_parameter("RTK_connection_attempt_backoff", RTK_connection_attempt_backoff_);
    // default is false for legacy compatibility
    nh_private_.get_parameter("RTK_connectivity_watchdog_enabled", rtk_connectivity_watchdog_enabled_);
    nh_private_.get_parameter("RTK_connectivity_watchdog_timer_frequency", rtk_connectivity_watchdog_timer_frequency_);
    nh_private_.get_parameter("RTK_data_transmission_interruption_limit", rtk_data_transmission_interruption_limit_);
    nh_private_.get_parameter("RTK_correction_protocol", RTK_correction_protocol_);
    nh_private_.get_parameter("RTK_server_IP", RTK_server_IP_);
    nh_private_.get_parameter("RTK_server_port", RTK_server_port_);
    nh_private_.get_parameter("GPS1_type", gps1_type_);
    nh_private_.get_parameter("GPS2_type", gps2_type_);
    nh_private_.get_parameter("RTK_rover", RTK_rover_);
    nh_private_.get_parameter("RTK_pos_period_multiple", RTK_pos_.period_multiple);
    nh_private_.get_parameter("RTK_rover_radio_enable", RTK_rover_radio_enable_);
    nh_private_.get_parameter("RTK_base_USB", RTK_base_USB_);
    nh_private_.get_parameter("RTK_base_serial", RTK_base_serial_);
    nh_private_.get_parameter("RTK_base_TCP", RTK_base_TCP_);
    nh_private_.get_parameter("GNSS_Compass", GNSS_Compass_);
    nh_private_.get_parameter("RTK_cmp_period_multiple", RTK_cmp_.period_multiple);
    nh_private_.get_parameter("inclination", magInclination_);
    nh_private_.get_parameter("declination", magDeclination_);
    nh_private_.get_parameter("dynamic_model", insDynModel_);

    // Params with arrays
    get_vector_flash_config("INS_rpy_radians", 3, insRotation_);
    get_vector_flash_config("INS_xyz", 3, insOffset_);
    get_vector_flash_config("GPS_ant1_xyz", 3, gps1AntOffset_);
    get_vector_flash_config("GPS_ant2_xyz", 3, gps2AntOffset_);
    get_vector_flash_config("GPS_ref_lla", 3, refLla_);
}

void InertialSenseROS::configure_data_streams()
{
    configure_data_streams(false);
}

void InertialSenseROS::configure_data_streams(bool startup) // if startup is true each step will be attempted without returning
{
    if (!gps1PosStreaming_) // we always need GPS for Fix status
    {
        RCLCPP_INFO(logger_, "Attempting to enable GPS1 Pos data stream.");
        SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback, 1);
    }
    if (!strobeInStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable strobe in data stream.");
        SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback, 1); // we always want the strobe
        strobeInStreaming_ = true;
        if (!startup)
            return;
    }
    if (!flashConfigStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable flash config data stream.");
        SET_CALLBACK(DID_FLASH_CONFIG, nvm_flash_cfg_t, flash_config_callback, 0);
        if (!startup)
            return;
    }

    if (DID_INS_1_.enabled && !ins1Streaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable INS1 data stream.");
        SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback, DID_INS_1_.period_multiple);
        if (!startup)
            return;
    }
    if (DID_INS_2_.enabled && !ins2Streaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable INS2 data stream.");
        SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback, DID_INS_2_.period_multiple);
        if (!startup)
            return;
    }
    if (DID_INS_4_.enabled && !ins4Streaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable INS4 data stream.");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, DID_INS_4_.period_multiple);
        if (!startup)
            return;
    }

    bool covarianceConfiged = (covariance_enabled_ && insCovarianceStreaming_) || !covariance_enabled_;

    if (odom_ins_ned_.enabled && !(ins4Streaming_ && imuStreaming_ && covarianceConfiged))
    {
        RCLCPP_INFO(logger_, "Attempting to enable odom INS NED data stream.");

        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, DID_INS_4_.period_multiple); // Need NED
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, preint_IMU_.period_multiple);                           // Need angular rate data from IMU
        IMU_.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    ned_odom_msg.pose.covariance[row * 6 + col] = 1;
                    ned_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    ned_odom_msg.pose.covariance[row * 6 + col] = 0;
                    ned_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!startup)
            return;
        ;
    }

    if (odom_ins_ecef_.enabled && !(ins4Streaming_ && imuStreaming_ && covarianceConfiged))
    {
        RCLCPP_INFO(logger_, "Attempting to enable odom INS ECEF data stream.");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, DID_INS_4_.period_multiple); // Need quaternion and ecef
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, preint_IMU_.period_multiple);                           // Need angular rate data from IMU
        IMU_.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    ecef_odom_msg.pose.covariance[row * 6 + col] = 1;
                    ecef_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    ecef_odom_msg.pose.covariance[row * 6 + col] = 0;
                    ecef_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!startup)
            return;
    }

    if (odom_ins_enu_.enabled && !(ins4Streaming_ && imuStreaming_ && covarianceConfiged))
    {
        RCLCPP_INFO(logger_, "Attempting to enable odom INS ENU data stream.");
        SET_CALLBACK(DID_INS_4, ins_4_t, INS4_callback, DID_INS_4_.period_multiple); // Need ENU
        if (covariance_enabled_)
            SET_CALLBACK(DID_ROS_COVARIANCE_POSE_TWIST, ros_covariance_pose_twist_t, INS_covariance_callback, 200); // Need Covariance data
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, preint_IMU_.period_multiple);                           // Need angular rate data from IMU
        IMU_.enabled = true;
        // Create Identity Matrix
        //
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                if (row == col)
                {
                    enu_odom_msg.pose.covariance[row * 6 + col] = 1;
                    enu_odom_msg.twist.covariance[row * 6 + col] = 1;
                }
                else
                {
                    enu_odom_msg.pose.covariance[row * 6 + col] = 0;
                    enu_odom_msg.twist.covariance[row * 6 + col] = 0;
                }
            }
        }
        if (!startup)
            return;
    }

    if (NavSatFix_.enabled && !NavSatFixConfigured)
    {
        RCLCPP_INFO(logger_, "Attempting to enable NavSatFix.");
        NavSatFix_.pub = nh_.create_publisher<sensor_msgs::msg::NavSatFix>("NavSatFix", 1);

        // Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS
        uint16_t gnssSatSigConst = IS_.GetFlashConfig().gnssSatSigConst;

        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GPS)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GPS;
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GLO)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GLONASS;
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_BDS)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_COMPASS; // includes BeiDou.
        }
        if (gnssSatSigConst & GNSS_SAT_SIG_CONST_GAL)
        {
            NavSatFix_msg.status.service |= NavSatFixService::SERVICE_GALILEO;
        }
        NavSatFixConfigured = true;
        // DID_GPS1_POS and DID_GPS1_VEL are always streamed for fix status. See below
        if (!startup)
            return;
    }

    if (INL2_states_.enabled && !inl2StatesStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable INS2 States data stream.");
        SET_CALLBACK(DID_INL2_STATES, inl2_states_t, INL2_states_callback, INL2_states_.period_multiple);
        if (!startup)
            return;
    }

    if (GPS1_.enabled)
    {
        GPS1_.pub = nh_.create_publisher<inertial_sense_msgs::msg::GPS>(gps1_topic_, 1);

        // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
        if (!gps1PosStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS1 Pos data stream.");
            SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback, GPS1_.period_multiple); // we always need GPS for Fix status
            if (!startup)
                return;
        }
        if (!gps1VelStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS1 Vel data stream.");
            SET_CALLBACK(DID_GPS1_VEL, gps_vel_t, GPS_vel_callback, GPS1_.period_multiple); // we always need GPS for Fix status
            if (!startup)
                return;
        }

        // GPS raw streaming already handles streaming of both GPS1 and GPS2 (and GPS_BASE).
        if (GPS1_raw_.enabled && !gps1RawStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS1 RAW data stream.");
            GPS1_raw_.pub_obs = nh_.create_publisher<inertial_sense_msgs::msg::GNSSObsVec>(gps1_topic_ + "/obs", 50);
            GPS1_raw_.pub_eph = nh_.create_publisher<inertial_sense_msgs::msg::GNSSEphemeris>(gps1_topic_ + "/eph", 50);
            GPS1_raw_.pub_geph = nh_.create_publisher<inertial_sense_msgs::msg::GlonassEphemeris>(gps1_topic_ + "/geph", 50);
            SET_CALLBACK(DID_GPS1_RAW, gps_raw_t, GPS_raw_callback, gps_raw_period_multiple);
            GPS_base_raw_.pub_obs = nh_.create_publisher<inertial_sense_msgs::msg::GNSSObsVec>(gps1_topic_ + "/base_obs", 50);
            GPS_base_raw_.pub_eph = nh_.create_publisher<inertial_sense_msgs::msg::GNSSEphemeris>(gps1_topic_ + "/base_eph", 50);
            GPS_base_raw_.pub_geph = nh_.create_publisher<inertial_sense_msgs::msg::GlonassEphemeris>(gps1_topic_ + "/base_geph", 50);
            SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback, gps_raw_period_multiple);
            obs_bundle_timer_ = nh_.create_wall_timer(1ms, [this]
                                                      { GPS_obs_bundle_timer_callback(); });
            if (!startup)
                return;
        }

        // Set up the GPS info ROS stream
        if (GPS1_info_.enabled && !gps1InfoStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS1 Info data stream.");
            SET_CALLBACK(DID_GPS1_SAT, gps_sat_t, GPS_info_callback, gps_info_period_multiple);
            if (!startup)
                return;
        }
    }

    // we only publish the second GPS is dual_GNSS (compassing) is disabled
    if (GPS2_.enabled)
    {
        GPS2_.pub = nh_.create_publisher<inertial_sense_msgs::msg::GPS>(gps2_topic_, 1);

        // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
        if (!gps2PosStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS2 Pos data stream.");
            SET_CALLBACK(DID_GPS2_POS, gps_pos_t, GPS_pos_callback, GPS2_.period_multiple); // we always need GPS for Fix status
            if (!startup)
                return;
        }
        if (!gps2VelStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS2 Vel data stream.");
            SET_CALLBACK(DID_GPS2_VEL, gps_vel_t, GPS_vel_callback, GPS2_.period_multiple); // we always need GPS for Fix status
            if (!startup)
                return;
        }

        // GPS raw streaming already handles streaming of both GPS1 and GPS2 (and GPS_BASE).
        if (GPS2_raw_.enabled && !gps2RawStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS2 Obs data stream.");
            GPS2_raw_.pub_obs = nh_.create_publisher<inertial_sense_msgs::msg::GNSSObsVec>(gps2_topic_ + "/obs", 50);
            GPS2_raw_.pub_eph = nh_.create_publisher<inertial_sense_msgs::msg::GNSSEphemeris>(gps2_topic_ + "/eph", 50);
            GPS2_raw_.pub_geph = nh_.create_publisher<inertial_sense_msgs::msg::GlonassEphemeris>(gps2_topic_ + "/geph", 50);
            SET_CALLBACK(DID_GPS2_RAW, gps_raw_t, GPS_raw_callback, gps_raw_period_multiple);
            GPS_base_raw_.pub_obs = nh_.create_publisher<inertial_sense_msgs::msg::GNSSObsVec>("/base_obs", 50);
            GPS_base_raw_.pub_eph = nh_.create_publisher<inertial_sense_msgs::msg::GNSSEphemeris>(gps1_topic_ + "/base_eph", 50);
            GPS_base_raw_.pub_geph = nh_.create_publisher<inertial_sense_msgs::msg::GlonassEphemeris>(gps1_topic_ + "/base_geph", 50);
            SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback, gps_raw_period_multiple);
            obs_bundle_timer_ = nh_.create_wall_timer(1ms, [this]
                                                      { GPS_obs_bundle_timer_callback(); });
            if (!startup)
                return;
        }

        // Set up the GPS info ROS stream
        if (GPS2_info_.enabled && !gps2InfoStreaming_)
        {
            RCLCPP_INFO(logger_, "Attempting to enable GPS Info data stream.");
            SET_CALLBACK(DID_GPS2_SAT, gps_sat_t, GPS_info_callback, gps_info_period_multiple);
            if (!startup)
                return;
        }
    }

    // Set up the magnetometer ROS stream
    if (mag_.enabled && !magStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable Mag data stream.");
        SET_CALLBACK(DID_MAGNETOMETER, magnetometer_t, mag_callback, mag_.period_multiple);
        if (!startup)
            return;
    }

    // Set up the barometer ROS stream
    if (baro_.enabled && !baroStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable baro data stream.");
        SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback, baro_.period_multiple);
        if (!startup)
            return;
    }

    // Set up the preintegrated IMU (coning and sculling integral) ROS stream
    if (preint_IMU_.enabled && !preintImuStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable preint IMU data stream.");
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, preint_IMU_.period_multiple);
        if (!startup)
            return;
    }
    if (IMU_.enabled && !imuStreaming_)
    {
        RCLCPP_INFO(logger_, "Attempting to enable IMU data stream.");
        SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback, IMU_.period_multiple);
        if (!startup)
            return;
    }
    if (!startup)
    {
        data_streams_enabled_ = true;
        data_stream_timer_->cancel();
        RCLCPP_INFO(logger_, "Inertial Sense ROS data streams successfully enabled.");
        return;
    }
}

void InertialSenseROS::start_log()
{
    std::string filename = getenv("HOME");
    filename += "/Documents/Inertial_Sense/Logs/" + cISLogger::CreateCurrentTimestamp();
    RCLCPP_INFO_STREAM(logger_, "Creating log in " << filename << " folder");
    IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_GROUND_VEHICLE);
}

void InertialSenseROS::configure_ascii_output()
{
    //  ascii_msgs_t msgs = {};
    //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
    //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
    //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
    //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
    //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
    //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
    //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);
}

void InertialSenseROS::connect()
{
    /// Connect to the uINS
    RCLCPP_INFO(logger_, "Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    if (!IS_.Open(port_.c_str(), baudrate_))
    {
        RCLCPP_FATAL(logger_, "inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
        exit(0);
    }
    else
    {
        // Print if Successful
        RCLCPP_INFO(logger_, "Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
    }
}

bool InertialSenseROS::firmware_compatiblity_check()
{
    if (IS_.GetDeviceInfo().protocolVer[0] != PROTOCOL_VERSION_CHAR0 ||
        IS_.GetDeviceInfo().protocolVer[1] != PROTOCOL_VERSION_CHAR1 ||
        IS_.GetDeviceInfo().protocolVer[2] != PROTOCOL_VERSION_CHAR2 ||
        IS_.GetDeviceInfo().protocolVer[3] != PROTOCOL_VERSION_CHAR3 ||
        IS_.GetDeviceInfo().firmwareVer[0] != FIRMWARE_VERSION_CHAR0 ||
        IS_.GetDeviceInfo().firmwareVer[1] != FIRMWARE_VERSION_CHAR1 ||
        IS_.GetDeviceInfo().firmwareVer[2] != FIRMWARE_VERSION_CHAR2)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void InertialSenseROS::configure_flash_parameters()
{
    bool reboot = false;
    nvm_flash_cfg_t current_flash_cfg = IS_.GetFlashConfig();
    // RCLCPP_INFO(logger_,("Configuring flash: \nCurrent: %i, \nDesired: %i\n", current_flash_cfg.ioConfig, ioConfig_);

    if (current_flash_cfg.startupNavDtMs != navigation_dt_ms_)
    {
        reboot = true;
        RCLCPP_INFO(logger_, "navigation rate change from %dms to %dms, resetting uINS to make change", current_flash_cfg.startupNavDtMs, navigation_dt_ms_);
    }
    if (current_flash_cfg.ioConfig != ioConfig_)
    {
        RCLCPP_INFO(logger_, "ioConfig change from %x to %x, resetting uINS to make change", current_flash_cfg.ioConfig, ioConfig_);
        reboot = true;
    }

    if (current_flash_cfg.startupNavDtMs != navigation_dt_ms_ ||
        current_flash_cfg.insRotation != insRotation_ ||
        current_flash_cfg.insOffset != insOffset_ ||
        current_flash_cfg.gps1AntOffset != gps1AntOffset_ ||
        current_flash_cfg.gps2AntOffset != gps2AntOffset_ ||
        current_flash_cfg.refLla != refLla_ ||
        current_flash_cfg.insDynModel != insDynModel_ ||
        current_flash_cfg.ioConfig != ioConfig_)
    {
        current_flash_cfg.startupNavDtMs = navigation_dt_ms_;
        memcpy(current_flash_cfg.insRotation, insRotation_, sizeof(insRotation_));
        memcpy(current_flash_cfg.insOffset, insOffset_, sizeof(insOffset_));
        memcpy(current_flash_cfg.gps1AntOffset, gps1AntOffset_, sizeof(gps1AntOffset_));
        memcpy(current_flash_cfg.gps2AntOffset, gps2AntOffset_, sizeof(gps2AntOffset_));
        memcpy(current_flash_cfg.refLla, refLla_, sizeof(refLla_));
        current_flash_cfg.ioConfig = ioConfig_;
        // current_flash_cfg.magInclination = magInclination_;
        current_flash_cfg.magDeclination = magDeclination_;
        current_flash_cfg.insDynModel = insDynModel_;

        IS_.SendData(DID_FLASH_CONFIG, (uint8_t *)(&current_flash_cfg), sizeof(nvm_flash_cfg_t), 0);
    }

    if (reboot)
    {
        sleep(3);
        reset_device();
    }
}

void InertialSenseROS::connect_rtk_client(const std::string &RTK_correction_protocol, const std::string &RTK_server_IP, const int RTK_server_port)
{
    rtk_connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = "TCP:" + RTK_correction_protocol_ + ":" + RTK_server_IP + ":" + std::to_string(RTK_server_port);
    if (!RTK_server_mount_.empty() && !RTK_server_username_.empty())
    { // NTRIP options
        RTK_connection += ":" + RTK_server_mount_ + ":" + RTK_server_username_ + ":" + RTK_server_password_;
    }

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < RTK_connection_attempt_limit_)
    {
        ++RTK_connection_attempt_count;

        bool connected = IS_.OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            RCLCPP_INFO_STREAM(logger_, "Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= RTK_connection_attempt_limit_)
            {
                RCLCPP_ERROR_STREAM(logger_, "Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * RTK_connection_attempt_backoff_;
                RCLCPP_WARN_STREAM(logger_, "Retrying connection in " << sleep_duration << " seconds");
                nh_.get_clock()->sleep_for(sleep_duration * 1s);
            }
        }
    }

    rtk_connecting_ = false;
}

void InertialSenseROS::start_rtk_server(const std::string &RTK_server_IP, const int RTK_server_port)
{
    // [type]:[ip/url]:[port]
    std::string RTK_connection = "TCP:" + RTK_server_IP + ":" + std::to_string(RTK_server_port);

    if (IS_.CreateHost(RTK_connection))
    {
        RCLCPP_INFO_STREAM(logger_, "Successfully created " << RTK_connection << " as RTK server");
        initialized_ = true;
        return;
    }
    else
        RCLCPP_ERROR_STREAM(logger_, "Failed to create base server at " << RTK_connection);
}

void InertialSenseROS::start_rtk_connectivity_watchdog_timer()
{

    if (!rtk_connectivity_watchdog_enabled_)
    {
        return;
    }

    if (!rtk_connectivity_watchdog_timer_->is_steady())
    {
        rtk_connectivity_watchdog_timer_ = nh_.create_wall_timer(rtk_connectivity_watchdog_timer_frequency_ * 1s, [this]()
                                                                 { InertialSenseROS::rtk_connectivity_watchdog_timer_callback(); });
    }
}

void InertialSenseROS::stop_rtk_connectivity_watchdog_timer()
{
    rtk_traffic_total_byte_count_ = 0;
    rtk_data_transmission_interruption_count_ = 0;
    rtk_connectivity_watchdog_timer_->cancel();
}

void InertialSenseROS::rtk_connectivity_watchdog_timer_callback()
{
    if (rtk_connecting_)
    {
        return;
    }

    int latest_byte_count = IS_.GetClientServerByteCount();
    if (rtk_traffic_total_byte_count_ == latest_byte_count)
    {
        ++rtk_data_transmission_interruption_count_;

        if (rtk_data_transmission_interruption_count_ >= rtk_data_transmission_interruption_limit_)
        {
            RCLCPP_WARN(logger_, "RTK transmission interruption, reconnecting...");

            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);
        }
    }
    else
    {
        rtk_traffic_total_byte_count_ = latest_byte_count;
        rtk_data_transmission_interruption_count_ = 0;
    }
}

void InertialSenseROS::configure_rtk()
{
    uint32_t RTKCfgBits = 0;
    if (gps1_type_ == "F9P")
    {
        if (RTK_rover_)
        {
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
            RTK_pos_.enabled = true;
            RTK_pos_.period_multiple;
            RCLCPP_INFO(logger_, "InertialSense: RTK Rover Configured.");
            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);

            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_pos_.period_multiple);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_pos_.period_multiple);
            RTK_pos_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_pos/info", 10);
            RTK_pos_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_pos/rel", 10);

            start_rtk_connectivity_watchdog_timer();
        }
        if (GNSS_Compass_)
        {
            RTK_cmp_.enabled = true;
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_cmp_.period_multiple);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_cmp_.period_multiple);
            RTK_cmp_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_cmp/info", 10);
            RTK_cmp_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_cmp/rel", 10);
            RCLCPP_INFO(logger_, "InertialSense: Dual GNSS (compassing) configured");
        }
        if (RTK_rover_radio_enable_)
        {
            RTK_pos_.enabled = true;
            RTK_pos_.period_multiple;
            RTK_base_USB_ = RTK_base_USB_ = false;
            RTK_base_serial_ = RTK_base_serial_ = false;
            RCLCPP_INFO(logger_, "InertialSense: Configured as RTK Rover with radio enabled");
            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL;
            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_pos_.period_multiple);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_pos_.period_multiple);
            RTK_pos_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_pos/info", 10);
            RTK_pos_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_pos/rel", 10);
        }
        if (RTK_base_USB_)
        {
            RCLCPP_INFO(logger_, "InertialSense: Base Configured.");
            RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB;
        }
        if (RTK_base_serial_)
        {
            RCLCPP_INFO(logger_, "InertialSense: Base Configured.");
            RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2;
        }
        if (RTK_base_TCP_)
        {
            start_rtk_server(RTK_server_IP_, RTK_server_port_);
        }

        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }

    else
    {

        RCLCPP_ERROR_EXPRESSION(logger_, RTK_rover_ && (RTK_base_serial_ || RTK_base_USB_ || RTK_base_TCP_), "unable to configure onboard receiver to be both RTK rover and base - default to rover");
        RCLCPP_ERROR_EXPRESSION(logger_, RTK_rover_ && GNSS_Compass_, "unable to configure onboard receiver to be both RTK rover as dual GNSS - default to dual GNSS");

        uint32_t RTKCfgBits = 0;
        if (GNSS_Compass_)
        {
            RTK_rover_ = false;
            RCLCPP_INFO(logger_, "InertialSense: Configured as dual GNSS (compassing)");

            RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
            SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_cmp_.period_multiple);
            SET_CALLBACK(DID_GPS2_RTK_CMP_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_cmp_.period_multiple);
            RTK_pos_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_cmp/info", 10);
            RTK_pos_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_cmp/rel", 10);
        }

        if (RTK_rover_radio_enable_)
        {
            RTK_base_serial_ = false;
            RTK_base_USB_ = false;
            RTK_base_TCP_ = false;
            RCLCPP_INFO(logger_, "InertialSense: Configured as RTK Rover with radio enabled");

            RTKCfgBits |= (gps1_type_ == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);

            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_pos_.period_multiple);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_pos_.period_multiple);
            RTK_pos_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_pos/info", 10);
            RTK_pos_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_pos/rel", 10);
        }
        else if (RTK_rover_)
        {
            RTK_base_serial_ = false;
            RTK_base_USB_ = false;
            RTK_base_TCP_ = false;

            RCLCPP_INFO(logger_, "InertialSense: Configured as RTK Rover");

            RTKCfgBits |= (gps1_type_ == "F9P" ? RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL : RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING);

            connect_rtk_client(RTK_correction_protocol_, RTK_server_IP_, RTK_server_port_);

            SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback, RTK_pos_.period_multiple);
            SET_CALLBACK(DID_GPS1_RTK_POS_REL, gps_rtk_rel_t, RTK_Rel_callback, RTK_pos_.period_multiple);
            RTK_pos_.pub_info = nh_.create_publisher<inertial_sense_msgs::msg::RTKInfo>("RTK_pos/info", 10);
            RTK_pos_.pub_rel = nh_.create_publisher<inertial_sense_msgs::msg::RTKRel>("RTK_pos/rel", 10);

            start_rtk_connectivity_watchdog_timer();
        }
        else if (RTK_base_USB_ || RTK_base_serial_ || RTK_base_TCP_)
        {
            RCLCPP_INFO(logger_, "InertialSense: Configured as RTK Base");
            if (RTK_base_serial_)
                RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;
            if (RTK_base_USB_)
                RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB;
            if (RTK_base_TCP_)
                start_rtk_server(RTK_server_IP_, RTK_server_port_);
        }
        IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    }
    printf("\n\nRTKCfgBits: %x\n", RTKCfgBits);
}

template <typename T>
void InertialSenseROS::get_vector_flash_config(std::string param_name, uint32_t size, T &data)
{
    std::vector<double> tmp(size, 0);
    if (!nh_private_.has_parameter(param_name))
    { // Parameter not provided.
        return;
    }

    nh_private_.get_parameter(param_name, tmp);

    for (int i = 0; i < size; i++)
    {
        data[i] = tmp[i];
    }
}

void InertialSenseROS::flash_config_callback(eDataIDs DID, const nvm_flash_cfg_t *const msg)
{
    if (!flashConfigStreaming_)
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

    flashConfigStreaming_ = true;
    refLla_[0] = msg->refLla[0];
    refLla_[1] = msg->refLla[1];
    refLla_[2] = msg->refLla[2];
    refLLA_known = true;
    RCLCPP_INFO(logger_, "refLla was set");
}

void InertialSenseROS::INS1_callback(eDataIDs DID, const ins_1_t *const msg)
{
    if (!ins1Streaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (DID_INS_1_.enabled)
        {
            DID_INS_1_.pub = nh_.create_publisher<inertial_sense_msgs::msg::DIDINS1>("DID_INS_1", 1);
        }
    }

    ins1Streaming_ = true;
    // Standard DID_INS_1 message
    if (DID_INS_1_.enabled)
    {
        did_ins_1_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
        did_ins_1_msg.header.frame_id = frame_id_;
        did_ins_1_msg.week = msg->week;
        did_ins_1_msg.timeofweek = msg->timeOfWeek;
        did_ins_1_msg.insstatus = msg->insStatus;
        did_ins_1_msg.hdwstatus = msg->hdwStatus;
        did_ins_1_msg.theta[0] = msg->theta[0];
        did_ins_1_msg.theta[1] = msg->theta[1];
        did_ins_1_msg.theta[2] = msg->theta[2];
        did_ins_1_msg.uvw[0] = msg->uvw[0];
        did_ins_1_msg.uvw[1] = msg->uvw[1];
        did_ins_1_msg.uvw[2] = msg->uvw[2];
        did_ins_1_msg.lla[0] = msg->lla[0];
        did_ins_1_msg.lla[1] = msg->lla[1];
        did_ins_1_msg.lla[2] = msg->lla[2];
        did_ins_1_msg.ned[0] = msg->ned[0];
        did_ins_1_msg.ned[1] = msg->ned[1];
        did_ins_1_msg.ned[2] = msg->ned[2];
        DID_INS_1_.pub->publish(did_ins_1_msg);
    }
}

void InertialSenseROS::INS2_callback(eDataIDs DID, const ins_2_t *const msg)
{
    if (!ins2Streaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (DID_INS_2_.enabled)
            DID_INS_2_.pub = nh_.create_publisher<inertial_sense_msgs::msg::DIDINS2>("DID_INS_2", 1);
    }

    ins2Streaming_ = true;
    if (DID_INS_2_.enabled)
    {
        // Standard DID_INS_2 message
        did_ins_2_msg.header.frame_id = frame_id_;
        did_ins_2_msg.week = msg->week;
        did_ins_2_msg.timeofweek = msg->timeOfWeek;
        did_ins_2_msg.insstatus = msg->insStatus;
        did_ins_2_msg.hdwstatus = msg->hdwStatus;
        did_ins_2_msg.qn2b[0] = msg->qn2b[0];
        did_ins_2_msg.qn2b[1] = msg->qn2b[1];
        did_ins_2_msg.qn2b[2] = msg->qn2b[2];
        did_ins_2_msg.qn2b[3] = msg->qn2b[3];
        did_ins_2_msg.uvw[0] = msg->uvw[0];
        did_ins_2_msg.uvw[1] = msg->uvw[1];
        did_ins_2_msg.uvw[2] = msg->uvw[2];
        did_ins_2_msg.lla[0] = msg->lla[0];
        did_ins_2_msg.lla[1] = msg->lla[1];
        did_ins_2_msg.lla[2] = msg->lla[2];
        DID_INS_2_.pub->publish(did_ins_2_msg);
    }
}

void InertialSenseROS::INS4_callback(eDataIDs DID, const ins_4_t *const msg)
{
    if (!ins4Streaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (DID_INS_4_.enabled)
            DID_INS_4_.pub = nh_.create_publisher<inertial_sense_msgs::msg::DIDINS4>("DID_INS_4", 1);

        if (odom_ins_ned_.enabled)
            odom_ins_ned_.pub = nh_.create_publisher<nav_msgs::msg::Odometry>("odom_ins_ned", 1);

        if (odom_ins_enu_.enabled)
            odom_ins_enu_.pub = nh_.create_publisher<nav_msgs::msg::Odometry>("odom_ins_enu", 1);

        if (odom_ins_ecef_.enabled)
            odom_ins_ecef_.pub = nh_.create_publisher<nav_msgs::msg::Odometry>("odom_ins_ecef", 1);
    }

    ins4Streaming_ = true;
    if (!refLLA_known)
    {
        RCLCPP_INFO(logger_, "REFERENCE LLA MUST BE RECEIVED");
        return;
    }
    if (DID_INS_4_.enabled)
    {
        // Standard DID_INS_2 message
        did_ins_4_msg.header.frame_id = frame_id_;
        did_ins_4_msg.week = msg->week;
        did_ins_4_msg.timeofweek = msg->timeOfWeek;
        did_ins_4_msg.insstatus = msg->insStatus;
        did_ins_4_msg.hdwstatus = msg->hdwStatus;
        did_ins_4_msg.qe2b[0] = msg->qe2b[0];
        did_ins_4_msg.qe2b[1] = msg->qe2b[1];
        did_ins_4_msg.qe2b[2] = msg->qe2b[2];
        did_ins_4_msg.qe2b[3] = msg->qe2b[3];
        did_ins_4_msg.ve[0] = msg->ve[0];
        did_ins_4_msg.ve[1] = msg->ve[1];
        did_ins_4_msg.ve[2] = msg->ve[2];
        did_ins_4_msg.ecef[0] = msg->ecef[0];
        did_ins_4_msg.ecef[1] = msg->ecef[1];
        did_ins_4_msg.ecef[2] = msg->ecef[2];
        DID_INS_4_.pub->publish(did_ins_4_msg);
    }

    if (odom_ins_ned_.enabled || odom_ins_enu_.enabled || odom_ins_ecef_.enabled)
    {
        // Note: the covariance matrices need to be transformed into required frames of reference before publishing the ROS message!
        ixMatrix3 Rb2e, I;
        ixVector4 qe2b, qe2n;
        ixVector3d Pe, lla;
        float Pout[36];

        eye_MatN(I, 3);
        qe2b[0] = msg->qe2b[0];
        qe2b[1] = msg->qe2b[1];
        qe2b[2] = msg->qe2b[2];
        qe2b[3] = msg->qe2b[3];

        rotMatB2R(qe2b, Rb2e);
        Pe[0] = msg->ecef[0];
        Pe[1] = msg->ecef[1];
        Pe[2] = msg->ecef[2];
        ecef2lla(Pe, lla, 5);
        quat_ecef2ned(lla[0], lla[1], qe2n);

        if (odom_ins_ecef_.enabled)
        {
            // Pose
            // Transform attitude body to ECEF
            transform_6x6_covariance(Pout, poseCov, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                ecef_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform angular_rate from body to ECEF
            transform_6x6_covariance(Pout, twistCov, I, Rb2e);
            for (int i = 0; i < 36; i++)
            {
                ecef_odom_msg.twist.covariance[i] = Pout[i];
            }
            ecef_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            ecef_odom_msg.header.frame_id = frame_id_;

            // Position

            ecef_odom_msg.pose.pose.position.x = msg->ecef[0];
            ecef_odom_msg.pose.pose.position.y = msg->ecef[1];
            ecef_odom_msg.pose.pose.position.z = -msg->ecef[2];

            // Attitude

            ecef_odom_msg.pose.pose.orientation.w = msg->qe2b[0];
            ecef_odom_msg.pose.pose.orientation.x = msg->qe2b[1];
            ecef_odom_msg.pose.pose.orientation.y = msg->qe2b[2];
            ecef_odom_msg.pose.pose.orientation.z = msg->qe2b[3];

            // Linear Velocity

            ecef_odom_msg.twist.twist.linear.x = msg->ve[0];
            ecef_odom_msg.twist.twist.linear.y = msg->ve[1];
            ecef_odom_msg.twist.twist.linear.z = msg->ve[2];

            // Angular Velocity
            ixVector3 result;
            ixEuler theta;
            quat2euler(msg->qe2b, theta);
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            vectorBodyToReference(angVelImu, theta, result);

            ecef_odom_msg.twist.twist.angular.x = result[0];
            ecef_odom_msg.twist.twist.angular.y = result[1];
            ecef_odom_msg.twist.twist.angular.z = result[2];

            odom_ins_ecef_.pub->publish(ecef_odom_msg);

            if (publishTf_)
            {;
                tf2::convert(ecef_odom_msg.pose.pose, transform_ECEF_);
                transform_ECEF_.header.frame_id="ins_ecef";
                transform_ECEF_.child_frame_id="ins_base_link_ecef";
                transform_ECEF_.header.stamp=now();
                br.sendTransform(transform_ECEF_);
            }
        }

        if (odom_ins_ned_.enabled)
        {
            ixVector4 qn2b;
            ixMatrix3 Rb2n, Re2n, buf;

            // NED-to-body quaternion
            mul_Quat_ConjQuat(qn2b, qe2b, qe2n);
            // Body-to-NED rotation matrix
            rotMatB2R(qn2b, Rb2n);
            // ECEF-to-NED rotation matrix
            rotMatB2R(qe2n, buf);
            transpose_Mat3(Re2n, buf);

            // Pose
            // Transform position from ECEF to NED and attitude from body to NED
            transform_6x6_covariance(Pout, poseCov, Re2n, Rb2n);
            for (int i = 0; i < 36; i++)
            {
                ned_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform velocity from ECEF to NED and angular rate from body to NED
            transform_6x6_covariance(Pout, twistCov, Re2n, Rb2n);
            for (int i = 0; i < 36; i++)
            {
                ned_odom_msg.twist.covariance[i] = Pout[i];
            }

            ned_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            ned_odom_msg.header.frame_id = frame_id_;

            // Position
            ixVector3d llaPosRadians;
            // ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians, 5);
            ixVector3 ned;
            ixVector3d refLlaRadians;
            // convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
            // lla to ned
            lla2ned_d(refLlaRadians, llaPosRadians, ned);

            ned_odom_msg.pose.pose.position.x = ned[0];
            ned_odom_msg.pose.pose.position.y = ned[1];
            ned_odom_msg.pose.pose.position.z = ned[2];

            // Attitude

            ned_odom_msg.pose.pose.orientation.w = qn2b[0]; // w
            ned_odom_msg.pose.pose.orientation.x = qn2b[1]; // x
            ned_odom_msg.pose.pose.orientation.y = qn2b[2]; // y
            ned_odom_msg.pose.pose.orientation.z = qn2b[3]; // z

            // Linear Velocity
            ixVector3 result, theta;

            quatConjRot(result, qe2n, msg->ve);

            ned_odom_msg.twist.twist.linear.x = result[0];
            ned_odom_msg.twist.twist.linear.y = result[1];
            ned_odom_msg.twist.twist.linear.z = result[2];

            // Angular Velocity

            // Transform from body frame to NED
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            quatRot(result, qn2b, angVelImu);

            ned_odom_msg.twist.twist.angular.x = result[0];
            ned_odom_msg.twist.twist.angular.y = result[1];
            ned_odom_msg.twist.twist.angular.z = result[2];
            odom_ins_ned_.pub->publish(ned_odom_msg);

            if (publishTf_)
            {
                // // Calculate the TF from the pose...
                // transform_NED.setOrigin(tf::Vector3(ned_odom_msg.pose.pose.position.x, ned_odom_msg.pose.pose.position.y, ned_odom_msg.pose.pose.position.z));
                // tf::Quaternion q;
                // tf::quaternionMsgToTF(ned_odom_msg.pose.pose.orientation, q);
                // transform_NED.setRotation(q);

                // br.sendTransform(tf::StampedTransform(transform_NED, now(), "ins_ned", "ins_base_link_ned"));
                tf2::convert(ned_odom_msg.pose.pose, transform_NED_);
                transform_NED_.header.frame_id="ins_ned";
                transform_NED_.child_frame_id="ins_base_link_ned";
                transform_NED_.header.stamp=now();
                br.sendTransform(transform_NED_);
            }
        }

        if (odom_ins_enu_.enabled)
        {
            ixVector4 qn2b, qn2enu, qe2enu, qenu2b;
            ixMatrix3 Rb2enu, Re2enu, buf;
            ixEuler eul = {M_PI, 0, 0.5 * M_PI};
            // ENU-to-NED quaternion
            euler2quat(eul, qn2enu);
            // NED-to-body quaternion
            mul_Quat_ConjQuat(qn2b, qe2b, qe2n);
            // ENU-to-body quaternion
            mul_Quat_ConjQuat(qenu2b, qn2b, qn2enu);
            // ECEF-to-ENU quaternion
            mul_Quat_Quat(qe2enu, qn2enu, qe2n);
            // Body-to-ENU rotation matrix
            rotMatB2R(qenu2b, Rb2enu);
            // ECEF-to-ENU rotation matrix
            rotMatB2R(qe2enu, buf);
            transpose_Mat3(Re2enu, buf);

            // Pose
            // Transform position from ECEF to ENU and attitude from body to ENU
            transform_6x6_covariance(Pout, poseCov, Re2enu, Rb2enu);
            for (int i = 0; i < 36; i++)
            {
                enu_odom_msg.pose.covariance[i] = Pout[i];
            }
            // Twist
            // Transform velocity from ECEF to ENU and angular rate from body to ENU
            transform_6x6_covariance(Pout, twistCov, Re2enu, Rb2enu);
            for (int i = 0; i < 36; i++)
            {
                enu_odom_msg.twist.covariance[i] = Pout[i];
            }

            enu_odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
            enu_odom_msg.header.frame_id = frame_id_;

            // Position
            // Calculate in NED then convert
            ixVector3d llaPosRadians;
            // ecef to lla (rad,rad,m)
            ecef2lla(msg->ecef, llaPosRadians, 5);
            ixVector3 ned;
            ixVector3d refLlaRadians;
            // convert refLla_ to radians
            lla_Deg2Rad_d(refLlaRadians, refLla_);
            // lla to ned
            lla2ned_d(refLlaRadians, llaPosRadians, ned);

            // Rearrange from NED to ENU
            enu_odom_msg.pose.pose.position.x = ned[1];
            enu_odom_msg.pose.pose.position.y = ned[0];
            enu_odom_msg.pose.pose.position.z = -ned[2];

            // Attitude

            enu_odom_msg.pose.pose.orientation.w = qenu2b[0];
            enu_odom_msg.pose.pose.orientation.x = qenu2b[1];
            enu_odom_msg.pose.pose.orientation.y = qenu2b[2];
            enu_odom_msg.pose.pose.orientation.z = qenu2b[3];

            // Linear Velocity
            // same as NED but rearranged.
            ixVector3 result, theta;
            quatConjRot(result, qe2n, msg->ve);

            enu_odom_msg.twist.twist.linear.x = result[1];
            enu_odom_msg.twist.twist.linear.y = result[0];
            enu_odom_msg.twist.twist.linear.z = -result[2];

            // Angular Velocity

            // Transform from body frame to ENU
            ixVector3 angVelImu = {(f_t)imu_msg.angular_velocity.x, (f_t)imu_msg.angular_velocity.y, (f_t)imu_msg.angular_velocity.z};
            quatRot(result, qenu2b, angVelImu);

            enu_odom_msg.twist.twist.angular.x = result[0];
            enu_odom_msg.twist.twist.angular.y = result[1];
            enu_odom_msg.twist.twist.angular.z = result[2];

            odom_ins_enu_.pub->publish(enu_odom_msg);
            if (publishTf_)
            {
                // Calculate the TF from the pose...
                // transform_ENU.setOrigin(tf::Vector3(enu_odom_msg.pose.pose.position.x, enu_odom_msg.pose.pose.position.y, enu_odom_msg.pose.pose.position.z));
                // tf::Quaternion q;
                // tf::quaternionMsgToTF(enu_odom_msg.pose.pose.orientation, q);
                // transform_ENU.setRotation(q);
                // br.sendTransform(tf::StampedTransform(transform_ENU, now(), "ins_enu", "ins_base_link_enu"));
                tf2::convert(ned_odom_msg.pose.pose, transform_ENU_);
                transform_ENU_.header.frame_id="ins_enu";
                transform_ENU_.child_frame_id="ins_base_link_enu";
                transform_ENU_.header.stamp=now();
                br.sendTransform(transform_ENU_);
            }
        }
    }
}

void InertialSenseROS::INL2_states_callback(eDataIDs DID, const inl2_states_t *const msg)
{
    if (!inl2StatesStreaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (INL2_states_.enabled)
            INL2_states_.pub = nh_.create_publisher<inertial_sense_msgs::msg::INL2States>("inl2_states", 1);
    }
    inl2StatesStreaming_ = true;
    inl2_states_msg.header.stamp = ros_time_from_tow(msg->timeOfWeek);
    inl2_states_msg.header.frame_id = frame_id_;

    inl2_states_msg.quatecef.w = msg->qe2b[0];
    inl2_states_msg.quatecef.x = msg->qe2b[1];
    inl2_states_msg.quatecef.y = msg->qe2b[2];
    inl2_states_msg.quatecef.z = msg->qe2b[3];

    inl2_states_msg.velecef.x = msg->ve[0];
    inl2_states_msg.velecef.y = msg->ve[1];
    inl2_states_msg.velecef.z = msg->ve[2];

    inl2_states_msg.posecef.x = msg->ecef[0];
    inl2_states_msg.posecef.y = msg->ecef[1];
    inl2_states_msg.posecef.z = msg->ecef[2];

    inl2_states_msg.gyrobias.x = msg->biasPqr[0];
    inl2_states_msg.gyrobias.y = msg->biasPqr[1];
    inl2_states_msg.gyrobias.z = msg->biasPqr[2];

    inl2_states_msg.accelbias.x = msg->biasAcc[0];
    inl2_states_msg.accelbias.y = msg->biasAcc[1];
    inl2_states_msg.accelbias.z = msg->biasAcc[2];

    inl2_states_msg.barobias = msg->biasBaro;
    inl2_states_msg.magdec = msg->magDec;
    inl2_states_msg.maginc = msg->magInc;

    // Use custom INL2 states message
    if (INL2_states_.enabled)
    {
        INL2_states_.pub->publish(inl2_states_msg);
    }
}

void InertialSenseROS::INS_covariance_callback(eDataIDs DID, const ros_covariance_pose_twist_t *const msg)
{
    if (!insCovarianceStreaming_)
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

    insCovarianceStreaming_ = true;
    float poseCovIn[36];
    int ind1, ind2;

    // Pose and twist covariances unwrapped from LD
    LD2Cov(msg->covPoseLD, poseCovIn, 6);
    LD2Cov(msg->covTwistLD, twistCov, 6);

    // Need to change order of variables.
    // Incoming order for msg->covPoseLD is [attitude, position]. Outgoing should be [position, attitude] => need to swap
    // Incoming order for msg->covTwistLD is [lin_velocity, ang_rate]. Outgoing should be [lin_velocity, ang_rate] => no change
    // Order change (block swap) in covariance matrix:
    // |A  C| => |B  C'|
    // |C' B|    |C  A |
    // where A and B are symetric, C' is transposed C

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            // Swap blocks A and B
            ind1 = (i + 3) * 6 + j + 3;
            ind2 = i * 6 + j;
            poseCov[ind2] = poseCovIn[ind1];
            poseCov[ind1] = poseCovIn[ind2];
            if (i != j)
            {
                // Copy lower diagonals to upper diagonals
                poseCov[j * 6 + i] = poseCov[ind2];
                poseCov[(j + 3) * 6 + (i + 3)] = poseCov[ind1];
            }
        }
        // Swap blocks C and C'
        for (int j = 0; j < 3; j++)
        {
            ind1 = (i + 3) * 6 + j;
            ind2 = i * 6 + j + 3;
            poseCov[ind2] = poseCovIn[ind1];
            poseCov[ind1] = poseCovIn[ind2];
        }
    }
}

void InertialSenseROS::GPS_pos_callback(eDataIDs DID, const gps_pos_t *const msg)
{
    if (DID == DID_GPS1_POS)
    {
        if (!gps1PosStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        gps1PosStreaming_ = true;
    }
    else if (DID == DID_GPS2_POS)
    {
        if (!gps2PosStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        gps2PosStreaming_ = true;
    }

    GPS_week_ = msg->week;
    GPS_towOffset_ = msg->towOffset;
    if (GPS1_.enabled && msg->status & GPS_STATUS_FIX_MASK && (DID == DID_GPS1_POS))
    {
        gps1_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
        gps1_msg.week = msg->week;
        gps1_msg.status = msg->status;
        gps1_msg.header.frame_id = frame_id_;
        gps1_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
        gps1_msg.cno = msg->cnoMean;
        gps1_msg.latitude = msg->lla[0];
        gps1_msg.longitude = msg->lla[1];
        gps1_msg.altitude = msg->lla[2];
        gps1_msg.posecef.x = ecef_[0] = msg->ecef[0];
        gps1_msg.posecef.y = ecef_[1] = msg->ecef[1];
        gps1_msg.posecef.z = ecef_[2] = msg->ecef[2];
        gps1_msg.hmsl = msg->hMSL;
        gps1_msg.hacc = msg->hAcc;
        gps1_msg.vacc = msg->vAcc;
        gps1_msg.pdop = msg->pDop;
        publishGPS1();
    }
    if (GPS2_.enabled && msg->status & GPS_STATUS_FIX_MASK && (DID == DID_GPS2_POS))
    {
        gps2_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
        gps2_msg.week = msg->week;
        gps2_msg.status = msg->status;
        gps2_msg.header.frame_id = frame_id_;
        gps2_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
        gps2_msg.cno = msg->cnoMean;
        gps2_msg.latitude = msg->lla[0];
        gps2_msg.longitude = msg->lla[1];
        gps2_msg.altitude = msg->lla[2];
        gps2_msg.posecef.x = ecef_[0] = msg->ecef[0];
        gps2_msg.posecef.y = ecef_[1] = msg->ecef[1];
        gps2_msg.posecef.z = ecef_[2] = msg->ecef[2];
        gps2_msg.hmsl = msg->hMSL;
        gps2_msg.hacc = msg->hAcc;
        gps2_msg.vacc = msg->vAcc;
        gps2_msg.pdop = msg->pDop;
        publishGPS2();
    }
    if (NavSatFix_.enabled)
    {
        NavSatFix_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1.0e3);
        NavSatFix_msg.header.frame_id = frame_id_;
        NavSatFix_msg.status.status = -1;                           // Assume no Fix
        if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_2D) // Check for fix and set
        {
            NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_FIX;
        }

        if (msg->status & GPS_STATUS_FIX_SBAS) // Check for SBAS only fix
        {
            NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_SBAS_FIX;
        }

        if (msg->status & GPS_STATUS_FIX_MASK >= GPS_STATUS_FIX_RTK_SINGLE) // Check for any RTK fix
        {
            NavSatFix_msg.status.status = NavSatFixStatusFixType::STATUS_GBAS_FIX;
        }

        // NavSatFix_msg.status.service - Service set at Node Startup
        NavSatFix_msg.latitude = msg->lla[0];
        NavSatFix_msg.longitude = msg->lla[1];
        NavSatFix_msg.altitude = msg->lla[2];

        // Diagonal Known
        const double varH = pow(msg->hAcc / 1000.0, 2);
        const double varV = pow(msg->vAcc / 1000.0, 2);
        NavSatFix_msg.position_covariance[0] = varH;
        NavSatFix_msg.position_covariance[4] = varH;
        NavSatFix_msg.position_covariance[8] = varV;
        NavSatFix_msg.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN;
        NavSatFix_.pub->publish(NavSatFix_msg);
    }
}

void InertialSenseROS::GPS_vel_callback(eDataIDs DID, const gps_vel_t *const msg)
{
    if (DID == DID_GPS1_VEL)
    {
        if (!gps1VelStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        gps1VelStreaming_ = true;
    }
    if (DID == DID_GPS2_VEL)
    {
        if (!gps2VelStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        gps2VelStreaming_ = true;
    }

    if (GPS1_.enabled && (DID == DID_GPS1_VEL) && abs(GPS_towOffset_) > 0.001)
    {
        gps1_velecef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
        gps1_velecef.vector.x = msg->vel[0];
        gps1_velecef.vector.y = msg->vel[1];
        gps1_velecef.vector.z = msg->vel[2];
        publishGPS1();
    }
    if (GPS2_.enabled && (DID == DID_GPS2_VEL) && abs(GPS_towOffset_) > 0.001)
    {
        gps2_velecef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1.0e3);
        gps2_velecef.vector.x = msg->vel[0];
        gps2_velecef.vector.y = msg->vel[1];
        gps2_velecef.vector.z = msg->vel[2];
        publishGPS2();
    }
}

void InertialSenseROS::publishGPS1()
{
    auto tv = rclcpp::Time(gps1_velecef.header.stamp) ;
    auto tmsg = rclcpp::Time(gps1_msg.header.stamp);
    double dt = (tv - tmsg).seconds();
    if (abs(dt) < 2.0e-3)
    {
        gps1_msg.velecef = gps1_velecef.vector;
        GPS1_.pub->publish(gps1_msg);
    }
}

void InertialSenseROS::publishGPS2()
{
    auto tv = rclcpp::Time(gps2_velecef.header.stamp) ;
    auto tmsg = rclcpp::Time(gps2_msg.header.stamp);
    double dt = (tv - tmsg).seconds();
    if (abs(dt) < 2.0e-3)
    {
        gps2_msg.velecef = gps2_velecef.vector;
        GPS2_.pub->publish(gps2_msg);
    }
}

void InertialSenseROS::update()
{
    IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(eDataIDs DID, const strobe_in_time_t *const msg)
{
    if (!strobeInStreaming_)
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
    strobeInStreaming_ = true;
    // create the subscriber if it doesn't exist
    if (std::string(strobe_pub_->get_topic_name()).empty())
        strobe_pub_ = nh_.create_publisher<std_msgs::msg::Header>("strobe_time", 1);

    if (abs(GPS_towOffset_) > 0.001)
    {
        std_msgs::msg::Header strobe_msg;
        strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1.0e-3);
        strobe_pub_->publish(strobe_msg);
    }
}

void InertialSenseROS::GPS_info_callback(eDataIDs DID, const gps_sat_t *const msg)
{
    if (DID = DID_GPS1_SAT)
    {
        if (!gps1InfoStreaming_)
        {
            RCLCPP_INFO(logger_, "%s (GPS1 info) response received", cISDataMappings::GetDataSetName(DID));
            if (GPS1_info_.enabled)
                GPS1_info_.pub = nh_.create_publisher<inertial_sense_msgs::msg::GPSInfo>(gps1_topic_ + "/info", 1);
            gps1InfoStreaming_ = true;
        }
    }
    if (DID = DID_GPS2_SAT)
    {
        if (!gps2InfoStreaming_)
        {
            RCLCPP_INFO(logger_, "%s (GPS2 info) response received", cISDataMappings::GetDataSetName(DID));
            if (GPS2_info_.enabled)
                GPS2_info_.pub = nh_.create_publisher<inertial_sense_msgs::msg::GPSInfo>(gps1_topic_ + "/info", 1);
            gps2InfoStreaming_ = true;
        }
    }

    if (abs(GPS_towOffset_) < 0.001)
    { // Wait for valid msg->timeofweekMs
        return;
    }

    gps_info_msg.header.stamp = ros_time_from_tow(msg->timeOfWeekMs / 1.0e3);
    gps_info_msg.header.frame_id = frame_id_;
    gps_info_msg.num_sats = msg->numSats;
    for (int i = 0; i < 50; i++)
    {
        gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
        gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
    }
    if (DID == DID_GPS1_SAT)
        GPS1_info_.pub->publish(gps_info_msg);
    else if (DID == DID_GPS2_SAT)
        GPS2_info_.pub->publish(gps_info_msg);
}

void InertialSenseROS::mag_callback(eDataIDs DID, const magnetometer_t *const msg)
{
    if (!magStreaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (mag_.enabled)
            mag_.pub = nh_.create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
    }
    magStreaming_ = true;
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = ros_time_from_start_time(msg->time);
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field.x = msg->mag[0];
    mag_msg.magnetic_field.y = msg->mag[1];
    mag_msg.magnetic_field.z = msg->mag[2];

    mag_.pub->publish(mag_msg);
}

void InertialSenseROS::baro_callback(eDataIDs DID, const barometer_t *const msg)
{
    if (!baroStreaming_)
    {
        RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        if (baro_.enabled)
            baro_.pub = nh_.create_publisher<sensor_msgs::msg::FluidPressure>("baro", 1);
    }

    baroStreaming_ = true;
    sensor_msgs::msg::FluidPressure baro_msg;
    baro_msg.header.stamp = ros_time_from_start_time(msg->time);
    baro_msg.header.frame_id = frame_id_;
    baro_msg.fluid_pressure = msg->bar;
    baro_msg.variance = msg->barTemp;

    baro_.pub->publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(eDataIDs DID, const pimu_t *const msg)
{

    if (preint_IMU_.enabled)
    {
        if (!preintImuStreaming_)
        {
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
            preint_IMU_.pub = nh_.create_publisher<inertial_sense_msgs::msg::PreIntIMU>("preint_imu", 1);
        }
        preintImuStreaming_ = true;
        preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
        preintIMU_msg.header.frame_id = frame_id_;
        preintIMU_msg.dtheta.x = msg->theta[0];
        preintIMU_msg.dtheta.y = msg->theta[1];
        preintIMU_msg.dtheta.z = msg->theta[2];

        preintIMU_msg.dvel.x = msg->vel[0];
        preintIMU_msg.dvel.y = msg->vel[1];
        preintIMU_msg.dvel.z = msg->vel[2];

        preintIMU_msg.dt = msg->dt;

        preint_IMU_.pub->publish(preintIMU_msg);
    }

    if (IMU_.enabled)
    {
        if (!imuStreaming_)
        {
            RCLCPP_INFO(logger_, "IMU response received");
            IMU_.pub = nh_.create_publisher<sensor_msgs::msg::Imu>("imu", 1);
        }
        imuStreaming_ = true;
        imu_msg.header.stamp = ros_time_from_start_time(msg->time);
        imu_msg.header.frame_id = frame_id_;

        imu_msg.angular_velocity.x = msg->theta[0] / msg->dt;
        imu_msg.angular_velocity.y = msg->theta[1] / msg->dt;
        imu_msg.angular_velocity.z = msg->theta[2] / msg->dt;
        imu_msg.linear_acceleration.x = msg->vel[0] / msg->dt;
        imu_msg.linear_acceleration.y = msg->vel[1] / msg->dt;
        imu_msg.linear_acceleration.z = msg->vel[2] / msg->dt;

        IMU_.pub->publish(imu_msg);
    }
}

void InertialSenseROS::RTK_Misc_callback(eDataIDs DID, const gps_rtk_misc_t *const msg)
{
    inertial_sense_msgs::msg::RTKInfo rtk_info;
    if (abs(GPS_towOffset_) > 0.001)
    {

        rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_info.baseantcount = msg->baseAntennaCount;
        rtk_info.baseeph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount + msg->baseGpsEphemerisCount;
        rtk_info.baseobs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount + msg->baseGpsObservationCount;
        rtk_info.basella[0] = msg->baseLla[0];
        rtk_info.basella[1] = msg->baseLla[1];
        rtk_info.basella[2] = msg->baseLla[2];

        rtk_info.rovereph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount + msg->roverGpsEphemerisCount;
        rtk_info.roverobs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount + msg->roverGpsObservationCount;
        rtk_info.cycle_slip_count = msg->cycleSlipCount;
    }
    if (DID == DID_GPS1_RTK_POS_MISC)
    {
        if (!rtkPosMiscStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        rtkPosMiscStreaming_ = true;
        RTK_pos_.pub_info->publish(rtk_info);
    }
    if (DID == DID_GPS2_RTK_CMP_MISC)
    {
        if (!rtkCmpMiscStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));

        rtkCmpMiscStreaming_ = true;
        RTK_cmp_.pub_info->publish(rtk_info);
    }
}

void InertialSenseROS::RTK_Rel_callback(eDataIDs DID, const gps_rtk_rel_t *const msg)
{
    inertial_sense_msgs::msg::RTKRel rtk_rel;
    if (abs(GPS_towOffset_) > 0.001)
    {
        rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
        rtk_rel.differential_age = msg->differentialAge;
        rtk_rel.ar_ratio = msg->arRatio;
        uint32_t fixStatus = msg->status & GPS_STATUS_FIX_MASK;
        if (fixStatus == GPS_STATUS_FIX_3D)
        {
            rtk_rel.egpsnavfixstatus = inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_3D;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_rel.egpsnavfixstatus = inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_SINGLE;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_rel.egpsnavfixstatus = inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_FLOAT;
        }
        else if (fixStatus == GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_rel.egpsnavfixstatus = inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_FIX;
        }
        else if (msg->status & GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_rel.egpsnavfixstatus = inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD;
        }

        rtk_rel.vector_base_to_rover.x = msg->baseToRoverVector[0];
        rtk_rel.vector_base_to_rover.y = msg->baseToRoverVector[1];
        rtk_rel.vector_base_to_rover.z = msg->baseToRoverVector[2];
        rtk_rel.distance_base_to_rover = msg->baseToRoverDistance;
        rtk_rel.heading_base_to_rover = msg->baseToRoverHeading;
    }

    if (DID == DID_GPS1_RTK_POS_REL)
    {
        if (!rtkPosRelStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        rtkPosRelStreaming_ = true;
        RTK_pos_.pub_rel->publish(rtk_rel);
    }
    if (DID == DID_GPS2_RTK_CMP_REL)
    {
        if (!rtkCmpRelStreaming_)
            RCLCPP_INFO(logger_, "%s response received", cISDataMappings::GetDataSetName(DID));
        rtkCmpRelStreaming_ = true;
        RTK_cmp_.pub_rel->publish(rtk_rel);
    }

    // save for diagnostics TODO - Add more diagnostic info
    diagnostic_ar_ratio_ = rtk_rel.ar_ratio;
    diagnostic_differential_age_ = rtk_rel.differential_age;
    diagnostic_heading_base_to_rover_ = rtk_rel.heading_base_to_rover;
    diagnostic_fix_type_ = rtk_rel.egpsnavfixstatus;
}

void InertialSenseROS::GPS_raw_callback(eDataIDs DID, const gps_raw_t *const msg)
{
    if (DID == DID_GPS1_RAW)
    {
        if (!gps1RawStreaming_)
            RCLCPP_INFO(logger_, "%s (GPS raw) response received", cISDataMappings::GetDataSetName(DID));
        gps1RawStreaming_ = true;
    }
    if (DID == DID_GPS2_RAW)
    {
        if (!gps2RawStreaming_)
            RCLCPP_INFO(logger_, "%s (GPS raw) response received", cISDataMappings::GetDataSetName(DID));
        gps2RawStreaming_ = true;
    }

    switch (msg->dataType)
    {
    case raw_data_type_observation:
        GPS_obs_callback(DID, (obsd_t *)&msg->data.obs, msg->obsCount);
        break;

    case raw_data_type_ephemeris:
        GPS_eph_callback(DID, (eph_t *)&msg->data.eph);
        break;

    case raw_data_type_glonass_ephemeris:
        GPS_geph_callback(DID, (geph_t *)&msg->data.gloEph);
        break;

    default:
        break;
    }
}

void InertialSenseROS::GPS_obs_callback(eDataIDs DID, const obsd_t *const msg, int nObs)
{
    if (DID == DID_GPS1_RAW)
    {
        if (gps1_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps1_obs_Vec_.obs[0].time.time ||
             msg[0].time.sec != gps1_obs_Vec_.obs[0].time.sec))
            GPS_obs_bundle_timer_callback();
    }
    else if (DID == DID_GPS2_RAW)
    {
        if (gps2_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != gps2_obs_Vec_.obs[0].time.time ||
             msg[0].time.sec != gps2_obs_Vec_.obs[0].time.sec))
            GPS_obs_bundle_timer_callback();
    }
    else if (DID == DID_GPS_BASE_RAW)
    {
        if (base_obs_Vec_.obs.size() > 0 &&
            (msg[0].time.time != base_obs_Vec_.obs[0].time.time ||
             msg[0].time.sec != base_obs_Vec_.obs[0].time.sec))
            GPS_obs_bundle_timer_callback();
    }

    for (int i = 0; i < nObs; i++)
    {
        inertial_sense_msgs::msg::GNSSObservation obs;
        obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
        obs.time.time = msg[i].time.time;
        obs.time.sec = msg[i].time.sec;
        obs.sat = msg[i].sat;
        obs.rcv = msg[i].rcv;
        obs.snr = msg[i].SNR[0];
        obs.lli = msg[i].LLI[0];
        obs.code = msg[i].code[0];
        obs.quall = msg[i].qualL[0];
        obs.qualp = msg[i].qualP[0];
        obs.l = msg[i].L[0];
        obs.p = msg[i].P[0];
        obs.d = msg[i].D[0];
        if (DID == DID_GPS1_RAW)
        {
            gps1_obs_Vec_.obs.push_back(obs);
            last_obs_time_1_ = now();
        }
        else if (DID == DID_GPS2_RAW)
        {
            gps2_obs_Vec_.obs.push_back(obs);
            last_obs_time_2_ = now();
        }
        else if (DID == DID_GPS_BASE_RAW)
        {
            base_obs_Vec_.obs.push_back(obs);
            last_obs_time_base_ = now();
        }
    }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback()
{
    if (gps1_obs_Vec_.obs.size() != 0)
    {
        if (abs((nh_.get_clock()->now() - last_obs_time_1_).seconds()) > 1e-2)
        {
            gps1_obs_Vec_.header.stamp = ros_time_from_gtime(gps1_obs_Vec_.obs[0].time.time, gps1_obs_Vec_.obs[0].time.sec);
            gps1_obs_Vec_.time = gps1_obs_Vec_.obs[0].time;
            GPS1_raw_.pub_obs->publish(gps1_obs_Vec_);
            gps1_obs_Vec_.obs.clear();
        }
    }
    if (gps2_obs_Vec_.obs.size() != 0)
    {

        if (abs((now() - last_obs_time_2_).seconds()) > 1e-2)
        {
            gps2_obs_Vec_.header.stamp = ros_time_from_gtime(gps2_obs_Vec_.obs[0].time.time, gps2_obs_Vec_.obs[0].time.sec);
            gps2_obs_Vec_.time = gps2_obs_Vec_.obs[0].time;
            GPS2_raw_.pub_obs->publish(gps2_obs_Vec_);
            gps2_obs_Vec_.obs.clear();
        }
    }
    if (base_obs_Vec_.obs.size() != 0)
    {
        if (abs((now() - last_obs_time_base_).seconds()) > 1e-2)
        {
            base_obs_Vec_.header.stamp = ros_time_from_gtime(base_obs_Vec_.obs[0].time.time, base_obs_Vec_.obs[0].time.sec);
            base_obs_Vec_.time = base_obs_Vec_.obs[0].time;
            GPS_base_raw_.pub_obs->publish(base_obs_Vec_);
            base_obs_Vec_.obs.clear();
        }
    }
}

void InertialSenseROS::GPS_eph_callback(eDataIDs DID, const eph_t *const msg)
{
    inertial_sense_msgs::msg::GNSSEphemeris eph;
    eph.sat = msg->sat;
    eph.iode = msg->iode;
    eph.iodc = msg->iodc;
    eph.sva = msg->sva;
    eph.svh = msg->svh;
    eph.week = msg->week;
    eph.code = msg->code;
    eph.flag = msg->flag;
    eph.toe.time = msg->toe.time;
    eph.toc.time = msg->toc.time;
    eph.ttr.time = msg->ttr.time;
    eph.toe.sec = msg->toe.sec;
    eph.toc.sec = msg->toc.sec;
    eph.ttr.sec = msg->ttr.sec;
    eph.a = msg->A;
    eph.e = msg->e;
    eph.i0 = msg->i0;
    eph.omg0 = msg->OMG0;
    eph.omg = msg->omg;
    eph.m0 = msg->M0;
    eph.deln = msg->deln;
    eph.omgd = msg->OMGd;
    eph.idot = msg->idot;
    eph.crc = msg->crc;
    eph.crs = msg->crs;
    eph.cuc = msg->cuc;
    eph.cus = msg->cus;
    eph.cic = msg->cic;
    eph.cis = msg->cis;
    eph.toes = msg->toes;
    eph.fit = msg->fit;
    eph.f0 = msg->f0;
    eph.f1 = msg->f1;
    eph.f2 = msg->f2;
    eph.tgd[0] = msg->tgd[0];
    eph.tgd[1] = msg->tgd[1];
    eph.tgd[2] = msg->tgd[2];
    eph.tgd[3] = msg->tgd[3];
    eph.adot = msg->Adot;
    eph.ndot = msg->ndot;
    if (DID == DID_GPS1_RAW)
        GPS1_raw_.pub_eph->publish(eph);
    else if (DID == DID_GPS2_RAW)
        GPS2_raw_.pub_eph->publish(eph);
    else if (DID == DID_GPS_BASE_RAW)
        GPS_base_raw_.pub_eph->publish(eph);
}

void InertialSenseROS::GPS_geph_callback(eDataIDs DID, const geph_t *const msg)
{
    inertial_sense_msgs::msg::GlonassEphemeris geph;
    geph.sat = msg->sat;
    geph.iode = msg->iode;
    geph.frq = msg->frq;
    geph.svh = msg->svh;
    geph.sva = msg->sva;
    geph.age = msg->age;
    geph.toe.time = msg->toe.time;
    geph.tof.time = msg->tof.time;
    geph.toe.sec = msg->toe.sec;
    geph.tof.sec = msg->tof.sec;
    geph.pos[0] = msg->pos[0];
    geph.pos[1] = msg->pos[1];
    geph.pos[2] = msg->pos[2];
    geph.vel[0] = msg->vel[0];
    geph.vel[1] = msg->vel[1];
    geph.vel[2] = msg->vel[2];
    geph.acc[0] = msg->acc[0];
    geph.acc[1] = msg->acc[1];
    geph.acc[2] = msg->acc[2];
    geph.taun = msg->taun;
    geph.gamn = msg->gamn;
    geph.dtaun = msg->dtaun;
    if (DID == DID_GPS1_RAW)
        GPS1_raw_.pub_geph->publish(geph);
    else if (DID == DID_GPS2_RAW)
        GPS2_raw_.pub_geph->publish(geph);
    else if (DID == DID_GPS_BASE_RAW)
        GPS_base_raw_.pub_geph->publish(geph);
}

void InertialSenseROS::diagnostics_callback()
{
    if (!diagnosticsStreaming_)
        RCLCPP_INFO(logger_, "Diagnostics response received");
    diagnosticsStreaming_ = true;
    // Create diagnostic objects
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = nh_.get_clock()->now();

    // CNO mean
    diagnostic_msgs::msg::DiagnosticStatus cno_mean;
    cno_mean.name = "CNO Mean";
    cno_mean.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    cno_mean.message = std::to_string(gps1_msg.cno);
    diag_array.status.push_back(cno_mean);

    if (RTK_pos_.enabled)
    {
        diagnostic_msgs::msg::DiagnosticStatus rtk_status;
        rtk_status.name = "RTK";
        rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        std::string rtk_message;

        // AR ratio
        diagnostic_msgs::msg::KeyValue ar_ratio;
        ar_ratio.key = "AR Ratio";
        ar_ratio.value = std::to_string(diagnostic_ar_ratio_);
        rtk_status.values.push_back(ar_ratio);
        if (diagnostic_fix_type_ == inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_3D)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message = "3D: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_SINGLE)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message = "Single: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_FLOAT)
        {
            rtk_message = "Float: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FIX_RTK_FIX)
        {
            rtk_message = "Fix: " + std::to_string(diagnostic_ar_ratio_);
        }
        else if (diagnostic_fix_type_ == inertial_sense_msgs::msg::RTKRel::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD)
        {
            rtk_message = "Fix and Hold: " + std::to_string(diagnostic_ar_ratio_);
        }
        else
        {
            rtk_message = "Unknown Fix: " + std::to_string(diagnostic_ar_ratio_);
        }

        // Differential age
        diagnostic_msgs::msg::KeyValue differential_age;
        differential_age.key = "Differential Age";
        differential_age.value = std::to_string(diagnostic_differential_age_);
        rtk_status.values.push_back(differential_age);
        if (diagnostic_differential_age_ > 1.5)
        {
            rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            rtk_message += " Differential Age Large";
        }

        // Heading base to rover
        diagnostic_msgs::msg::KeyValue heading_base_to_rover;
        heading_base_to_rover.key = "Heading Base to Rover (rad)";
        heading_base_to_rover.value = std::to_string(diagnostic_heading_base_to_rover_);
        rtk_status.values.push_back(heading_base_to_rover);

        rtk_status.message = rtk_message;
        diag_array.status.push_back(rtk_status);
    }

    diagnostics_.pub->publish(diag_array);
}

void InertialSenseROS::set_current_position_as_refLLA(TriggerReq req, TriggerRes res)
{
    (void)req;
    double current_lla_[3];
    current_lla_[0] = lla_[0];
    current_lla_[1] = lla_[1];
    current_lla_[2] = lla_[2];

    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
    while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

    if (current_lla_[0] == IS_.GetFlashConfig().refLla[0] && current_lla_[1] == IS_.GetFlashConfig().refLla[1] && current_lla_[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res->success = true;
        res->message = ("Update was succesful.  refLla: Lat: " + std::to_string(current_lla_[0]) + "  Lon: " + std::to_string(current_lla_[1]) + "  Alt: " + std::to_string(current_lla_[2]));
    }
    else
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res->success = false;
        res->message = "Unable to update refLLA. Please try again.";
    }
}

void InertialSenseROS::set_refLLA_to_value(RefllaReq req, RefllaRes res)
{
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t *>(&req->lla), sizeof(req->lla), offsetof(nvm_flash_cfg_t, refLla));

    comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

    int i = 0;
    nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
    while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerStep();
        i++;
        if (i > 100)
        {
            break;
        }
    }

    if (req->lla[0] == IS_.GetFlashConfig().refLla[0] && req->lla[1] == IS_.GetFlashConfig().refLla[1] && req->lla[2] == IS_.GetFlashConfig().refLla[2])
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res->success = true;
        res->message = ("Update was succesful.  refLla: Lat: " + std::to_string(req->lla[0]) + "  Lon: " + std::to_string(req->lla[1]) + "  Alt: " + std::to_string(req->lla[2]));
    }
    else
    {
        comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
        res->success = false;
        res->message = "Unable to update refLLA. Please try again.";
    }
}

void InertialSenseROS::perform_mag_cal_srv_callback(TriggerReq req, TriggerRes res)
{
    (void)req;
    uint32_t single_axis_command = 2;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.GetSerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.dataPtr + comm.dataHdr.offset);
            if (msg->insStatus & 0x00400000)
            {
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
            }
        }
    }
}

rclcpp::Time InertialSenseROS::now(){
    return nh_.get_clock()->now();
}

void InertialSenseROS::perform_multi_mag_cal_srv_callback(TriggerReq req, TriggerRes res)
{
    (void)req;
    uint32_t multi_axis_command = 1;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t *>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, state));

    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    serial_port_t *serialPort = IS_.GetSerialPort();
    uint8_t inByte;
    int n;

    while ((n = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        // Search comm buffer for valid packets
        if (is_comm_parse_byte(&comm, inByte) == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_INS_1)
        {
            ins_1_t *msg = (ins_1_t *)(comm.dataPtr + comm.dataHdr.offset);
            if (msg->insStatus & 0x00400000)
            {
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
            }
        }
    }
}

void InertialSenseROS::reset_device()
{
    // send reset command
    system_command_t reset_command;
    reset_command.command = 99;
    reset_command.invCommand = ~reset_command.command;
    IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t *>(&reset_command), sizeof(system_command_t), 0);
    RCLCPP_WARN(logger_, "Device reset required.\n\nShutting down Node.\n");
    rclcpp::shutdown();
}

void InertialSenseROS::update_firmware_srv_callback(FirmwareUpdateReq req, FirmwareUpdateRes res)
{
    //   IS_.Close();
    //   vector<InertialSense::bootload_result_t> results = IS_.BootloadFile("*", req->filename, 921600);
    //   if (!results[0].error.empty())
    //   {
    //      res->success = false;
    //      res->message = results[0].error;
    //     return false;
    //   }
    //   IS_.Open(port_.c_str(), baudrate_);

    return true;
}

rclcpp::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeofweek)
{
    rclcpp::Time rostime(0, 0);
    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeofweek) + week * 7 * 24 * 3600;
        uint64_t nsec = (timeofweek - floor(timeofweek)) * 1e9;
        rostime = rclcpp::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = now().seconds() - timeofweek;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = now().seconds() - timeofweek;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = rclcpp::Time(INS_local_offset_ + timeofweek);
    }
    return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
    rclcpp::Time rostime(0, 0);

    //  If we have a GPS fix, then use it to set timestamp
    if (abs(GPS_towOffset_) > 0.001)
    {
        double timeofweek = time + GPS_towOffset_;
        uint64_t sec = (uint64_t)(UNIX_TO_GPS_OFFSET + floor(timeofweek) + GPS_week_ * 7 * 24 * 3600);
        uint64_t nsec = (uint64_t)((timeofweek - floor(timeofweek)) * 1.0e9);
        rostime = rclcpp::Time(sec, nsec);
    }
    else
    {
        // Otherwise, estimate the uINS boot time and offset the messages
        if (!got_first_message_)
        {
            got_first_message_ = true;
            INS_local_offset_ = now().seconds() - time;
        }
        else // low-pass filter offset to account for drift
        {
            double y_offset = now().seconds() - time;
            INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
        }
        // Publish with ROS time
        rostime = rclcpp::Time(INS_local_offset_ + time);
    }
    return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
    return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const rclcpp::Time &rt)
{
    auto nsec = rt.nanoseconds();
    // auto res = std::lldiv(t, 1000000000);
    // uint32_t sec = res.quot;
    // uint64_t nsec = res.rem;
    // // auto nsec = std::modl


    return (UNIX_TO_GPS_OFFSET - GPS_week_ * 604800) + nsec * 1.0e-9;
}

rclcpp::Time InertialSenseROS::ros_time_from_gtime(const uint64_t sec, double subsec)
{
    rclcpp::Time out(sec - LEAP_SECONDS,subsec * 1e9);
    return out;
}

void InertialSenseROS::LD2Cov(const float *LD, float *Cov, int width)
{
    for (int j = 0; j < width; j++)
    {
        for (int i = 0; i < width; i++)
        {
            if (i < j)
            {
                Cov[i * width + j] = Cov[j * width + i];
            }
            else
            {
                Cov[i * width + j] = LD[(i * i + i) / 2 + j];
            }
        }
    }
}

void InertialSenseROS::rotMatB2R(const ixVector4 quat, ixMatrix3 R)
{
    R[0] = 1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]);
    R[1] = 2.0f * (quat[1] * quat[2] - quat[0] * quat[3]);
    R[2] = 2.0f * (quat[1] * quat[3] + quat[0] * quat[2]);
    R[3] = 2.0f * (quat[1] * quat[2] + quat[0] * quat[3]);
    R[4] = 1.0f - 2.0f * (quat[1] * quat[1] + quat[3] * quat[3]);
    R[5] = 2.0f * (quat[2] * quat[3] - quat[0] * quat[1]);
    R[6] = 2.0f * (quat[1] * quat[3] - quat[0] * quat[2]);
    R[7] = 2.0f * (quat[2] * quat[3] + quat[0] * quat[1]);
    R[8] = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);
}

void InertialSenseROS::transform_6x6_covariance(float Pout[36], float Pin[36], ixMatrix3 R1, ixMatrix3 R2)
{
    // Assumption: input covariance matrix is transformed due to change of coordinates,
    // so that fisrt 3 coordinates are rotated by R1 and the last 3 coordinates are rotated by R2
    // This is how the transformation looks:
    // |R1  0 | * |Pxx  Pxy'| * |R1' 0  | = |R1*Pxx*R1'  R1*Pxy'*R2'|
    // |0   R2|   |Pxy  Pyy |   |0   R2'|   |R2*Pxy*R1'  R2*Pyy*R2' |

    ixMatrix3 Pxx_in, Pxy_in, Pyy_in, Pxx_out, Pxy_out, Pyy_out, buf;

    // Extract 3x3 blocks from input covariance
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // Upper diagonal block in old frame
            Pxx_in[i * 3 + j] = Pin[i * 6 + j];
            // Lower left block of in old frame
            Pxy_in[i * 3 + j] = Pin[(i + 3) * 6 + j];
            // Lower diagonal block in old frame
            Pyy_in[i * 3 + j] = Pin[(i + 3) * 6 + j + 3];
        }
    }
    // Transform the 3x3 covariance blocks
    // New upper diagonal block
    mul_Mat3x3_Mat3x3(buf, R1, Pxx_in);
    mul_Mat3x3_Mat3x3_Trans(Pxx_out, buf, R1);
    // New lower left block
    mul_Mat3x3_Mat3x3(buf, R2, Pxy_in);
    mul_Mat3x3_Mat3x3_Trans(Pxy_out, buf, R1);
    // New lower diagonal  block
    mul_Mat3x3_Mat3x3(buf, R2, Pyy_in);
    mul_Mat3x3_Mat3x3_Trans(Pyy_out, buf, R2);

    // Copy the computed transformed blocks into output 6x6 covariance matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // Upper diagonal block in the new frame
            Pout[i * 6 + j] = Pxx_in[i * 3 + j];
            // Lower left block in the new frame
            Pout[(i + 3) * 6 + j] = Pxy_in[i * 3 + j];
            // Upper right block in the new frame
            Pout[i * 6 + j + 3] = Pxy_in[j * 3 + i];
            // Lower diagonal block in the new frame
            Pout[(i + 3) * 6 + j + 3] = Pyy_in[i * 3 + j];
        }
    }
}

template <typename Type>
bool InertialSenseROS::get_node_param_yaml(YAML::Node node, const std::string key, Type &val)
{
    if (node[key])
    {
        try
        {
            val = node[key].as<Type>();
            return true;
        }
        catch (const YAML::KeyNotFound &knf)
        {
            std::cout << "get_node_param_yaml(): Key \"" + key + "\" not found.  Using default value.\n";
            return false;
        }
    }
    else
    {
        std::cout << "get_node_param_yaml(): Key \"" + key + "\" not in yaml node.  Using default value.\n";
        return false;
    }
}

template <typename Derived1>
bool InertialSenseROS::get_node_vector_yaml(YAML::Node node, const std::string key, int size, Derived1 &val)
{
    if (node[key])
    {
        std::vector<double> vec;
        try
        {
            vec = node[key].as<std::vector<double>>();
            for (int i = 0; i < size; i++)
            {
                val[i] = vec[i];
            }
            return true;
        }
        catch (...)
        {
            std::cout << "get_node_param_yaml(): Key \"" + key + "\" not found.  Using default value.\n";
            return false;
        }
    }
    else
    {
        std::cout << "get_node_param_yaml(): Key \"" + key + "\" not in yaml node.  Using default value.\n";
        return false;
    }
}
