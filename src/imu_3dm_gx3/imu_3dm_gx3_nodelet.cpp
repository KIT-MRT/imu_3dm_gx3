#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "imu_3dm_gx3.hpp"

namespace imu_3dm_gx3_ros_tool {

class Imu3dmGx3Nodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<Imu3dmGx3>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<Imu3dmGx3> impl_;
};
} // namespace imu_3dm_gx3_ros_tool

PLUGINLIB_DECLARE_CLASS(imu_3dm_gx3_ros_tool, Imu3dmGx3Nodelet,
                        imu_3dm_gx3_ros_tool::Imu3dmGx3Nodelet, nodelet::Nodelet);
