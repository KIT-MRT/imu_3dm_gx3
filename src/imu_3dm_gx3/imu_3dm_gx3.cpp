#include "imu_3dm_gx3.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <utils_ros/ros_console.hpp>

#include "device.hpp"
#include "helpers.hpp"

namespace imu_3dm_gx3_ros_tool {

Imu3dmGx3::Imu3dmGx3(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : reconfigure_server_{nh_private}, params_{nh_private} {

    /**
     * Initialization
     */
    utils_ros::setLoggerLevel(nh_private);
    params_.fromParamServer();
    setupDiagnostics();
    reconfigure_server_.setCallback(boost::bind(&Imu3dmGx3::reconfigureRequest, this, _1, _2));

    /**
     * Publisher
     */
    publisher_ = std::make_unique<PublisherImu>(
        nh_public.advertise<Imu>(params_.msg_name_publisher, params_.msg_queue_size), updater_,
        diagnostic_updater::FrequencyStatusParam(
            &params_.diagnostic_updater_rate, &params_.diagnostic_updater_rate,
            params_.diagnostic_updater_rate_tolerance, params_.msg_queue_size + 5),
        diagnostic_updater::TimeStampStatusParam());

    /**
     * Summary
     */
    utils_ros::showNodeInfo();

    /**
     * Try to init device twice
     */
    serial_port_ = std::make_unique<SerialPort>(io_service_);
    if (!initDevice()) {
        ROS_DEBUG_STREAM("Reinitializing.");
        ros::Duration(0.1).sleep();
        if (!initDevice()) {
            ROS_ERROR_STREAM("Connection not successful.");
            std::exit(EXIT_FAILURE);
        }
    }

    /**
     * Spin
     */
    spin(nh_private);

    /**
     * Stop continous mode and close device
     */
    boost::asio::write(*serial_port_, boost::asio::buffer(stop, 3));
    ROS_DEBUG_STREAM("Wait 0.1s");
    ros::Duration(0.1).sleep();
    serial_port_->close();
}

void Imu3dmGx3::spin(const ros::NodeHandle& nh) {

    ROS_DEBUG_STREAM("Start streaming data...");
    const Imu::Ptr msg{new Imu};
    msg->header.frame_id = params_.frame_id;

    unsigned char data[data_length];
    while (nh.ok()) {

        msg->header.stamp = ros::Time::now();

        /**
         * Read data
         */
        try {
            boost::asio::read(*serial_port_, boost::asio::buffer(data, data_length));
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("Could not read from device: " << e.what());
        }
        if (!validate_checksum(data, data_length)) {
            ROS_WARN_STREAM("Checksum failed. Ignoring message.");
            continue;
        }

        size_t k{1};

        /**
         * Linear acceleration
         */
        float acc[3];
        for (size_t i = 0; i < 3; i++, k += 4)
            acc[i] = extract_float(&(data[k]));
        msg->linear_acceleration.x = acc[0] * 9.81;
        msg->linear_acceleration.y = acc[1] * 9.81;
        msg->linear_acceleration.z = acc[2] * 9.81;

        /**
         * Angular velocity
         */
        float ang_vel[3];
        for (size_t i = 0; i < 3; i++, k += 4)
            ang_vel[i] = extract_float(&(data[k]));
        msg->angular_velocity.x = ang_vel[0];
        msg->angular_velocity.y = ang_vel[1];
        msg->angular_velocity.z = ang_vel[2];

        /**
         * Orientation covariance
         */
        float mag[3];
        for (size_t i = 0; i < 3; i++, k += 4)
            mag[i] = extract_float(&(data[k]));
        msg->orientation_covariance[0] = mag[0];
        msg->orientation_covariance[1] = mag[1];
        msg->orientation_covariance[2] = mag[2];

        /**
         * Orientation
         */
        float M[9];
        for (size_t i = 0; i < 9; i++, k += 4)
            M[i] = extract_float(&(data[k]));
        Eigen::Matrix3d R;
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                R(i, j) = M[j * 3 + i];
        tf::quaternionEigenToMsg(Eigen::Quaterniond(R), msg->orientation);

        /**
         * \attention wsascha There is also a time since sensor start provided.
         * However, we do not use this here. If you know what to do with this, make some changes!
         */
        //        const double T{static_cast<double>(extract_int(&(data[k]))) / 62500};

        publisher_->publish(msg);
    }
}

bool Imu3dmGx3::initDevice() {

    try {
        serial_port_->open(params_.port);
    } catch (boost::system::system_error& error) {
        ROS_ERROR_STREAM("Failed to open serial port " << params_.port << ": " << error.what());
    }
    if (!serial_port_->is_open()) {
        ROS_ERROR_STREAM("Failed to open serial port " << params_.port << ".");
    }

    serial_port_->set_option(sb::baud_rate(baud_rate));
    serial_port_->set_option(flow_control);
    serial_port_->set_option(parity);
    serial_port_->set_option(stop_bits);

    /**
     * Stop continous mode if continuous mode is running
     */
    boost::asio::write(*serial_port_, boost::asio::buffer(stop, 3));
    ROS_DEBUG_STREAM("Wait 0.1s");
    ros::Duration(0.1).sleep();
    boost::asio::write(*serial_port_, boost::asio::buffer(mode, 4));
    unsigned char reply[reply_length];
    boost::asio::read(*serial_port_, boost::asio::buffer(reply, reply_length));
    if (!validate_checksum(reply, reply_length)) {
        ROS_ERROR_STREAM("Failed to get mode.");
        if (serial_port_->is_open())
            serial_port_->close();
        return false;
    }

    /**
     * Change mode if not in active mode
     */
    if (reply[2] != '\x01') {
        mode[3] = '\x01';
        boost::asio::write(*serial_port_, boost::asio::buffer(mode, 4));
        boost::asio::read(*serial_port_, boost::asio::buffer(reply, reply_length));
        if (!validate_checksum(reply, reply_length)) {
            ROS_ERROR_STREAM("Failed to set mode to active.");
            if (serial_port_->is_open())
                serial_port_->close();
            return false;
        }
    }

    /**
     * Set to continuous preset mode
     */
    boost::asio::write(*serial_port_, boost::asio::buffer(preset, 4));
    boost::asio::read(*serial_port_, boost::asio::buffer(reply, reply_length));
    if (!validate_checksum(reply, reply_length)) {
        ROS_ERROR_STREAM("Failed to set continuous mode preset.");
        if (serial_port_->is_open())
            serial_port_->close();
        return false;
    }

    /**
     * Set to continous output mode
     */
    mode[3] = '\x02';
    boost::asio::write(*serial_port_, boost::asio::buffer(mode, 4));
    boost::asio::read(*serial_port_, boost::asio::buffer(reply, reply_length));
    if (!validate_checksum(reply, reply_length)) {
        ROS_ERROR_STREAM("Failed to set mode to continuous output.");
        if (serial_port_->is_open())
            serial_port_->close();
        return false;
    }

    /**
     * Set timer
     */
    char set_timer[8] = {'\xD7', '\xC1', '\x29', '\x01', '\x00', '\x00', '\x00', '\x00'};
    unsigned char reply_timer[7];
    boost::asio::write(*serial_port_, boost::asio::buffer(set_timer, 8));
    boost::asio::read(*serial_port_, boost::asio::buffer(reply_timer, 7));

    return true;
}

void Imu3dmGx3::reconfigureRequest(const Config& config, uint32_t level) {
    params_.fromConfig(config);
}

void Imu3dmGx3::setupDiagnostics() {
    diagnostic_status_.hardware_id = params_.diagnostic_updater_hardware_id;
    diagnostic_status_.message = "Starting...";
    diagnostic_status_.level = diagnostic_msgs::DiagnosticStatus::STALE;
    updater_.setHardwareID(params_.diagnostic_updater_hardware_id);
    updater_.add("Imu3dmGx3 Sensor Status", this, &Imu3dmGx3::checkSensorStatus);
    updater_.force_update();
}

void Imu3dmGx3::checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnostic_status_);
}

} // namespace imu_3dm_gx3_ros_tool
