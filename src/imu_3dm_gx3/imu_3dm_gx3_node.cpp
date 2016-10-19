#include "imu_3dm_gx3.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "imu_3dm_gx3_node");

    imu_3dm_gx3_ros_tool::Imu3dmGx3 imu_3dm_gx3(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
