#pragma once

#include <boost/asio.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio/serial_port.hpp>

#include "imu_3dm_gx3_ros_tool/Imu3dmGx3Parameters.h"

namespace imu_3dm_gx3_ros_tool {

class Imu3dmGx3 {

    using Parameters = Imu3dmGx3Parameters;
    using Config = Imu3dmGx3Config;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

    using Imu = sensor_msgs::Imu;
    using PublisherImu = diagnostic_updater::DiagnosedPublisher<Imu>;

    using IoService = boost::asio::io_service;
    using SerialPort = boost::asio::serial_port;

public:
    Imu3dmGx3(ros::NodeHandle, ros::NodeHandle);

private:
    void setupDiagnostics();
    void checkSensorStatus(diagnostic_updater::DiagnosticStatusWrapper&);
    void spin(const ros::NodeHandle&);
    void reconfigureRequest(const Config&, uint32_t);
    bool initDevice();

    Imu3dmGx3Parameters params_;
    ReconfigureServer reconfigure_server_;                ///< Dynamic reconfiguration service
    diagnostic_updater::Updater updater_;                 ///< Diagnostic updater
    std::unique_ptr<PublisherImu> publisher_;             ///< Diagnosed publisher
    diagnostic_msgs::DiagnosticStatus diagnostic_status_; ///< Current diagnostic status

    IoService io_service_;
    std::unique_ptr<SerialPort> serial_port_;
};
} // namespace imu_3dm_gx3_ros_tool
