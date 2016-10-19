#pragma once

#include <boost/asio/serial_port.hpp>

namespace imu_3dm_gx3_ros_tool {

constexpr size_t reply_length = 4;

constexpr char stop[3] = {'\xFA', '\x75', '\xB4'};
char mode[4] = {'\xD4', '\xA3', '\x47', '\x00'};
constexpr char preset[4] = {'\xD6', '\xC6', '\x6B', '\xCC'};

constexpr size_t data_length = 79;
constexpr size_t baud_rate = 115200;

using sb = boost::asio::serial_port_base;
sb::flow_control flow_control(sb::flow_control::none);
sb::parity parity(sb::parity::none);
sb::stop_bits stop_bits(sb::stop_bits::one);
}
