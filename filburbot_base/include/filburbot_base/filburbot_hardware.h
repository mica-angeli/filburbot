#ifndef FILBURBOT_BASE_FILBURBOT_HARDWARE_H
#define FILBURBOT_BASE_FILBURBOT_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace filburbot_base {

  class FilburbotHardware : public hardware_interface::RobotHW
  {
  public:
    FilburbotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

    void sendCommandsToMotors();

  private:
    void registerControlInterfaces();

  private:
    ros::Subscriber sub_encoders_;
    ros::Publisher pub_cmd_diff_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

  };
}

#endif //FILBURBOT_BASE_FILBURBOT_HARDWARE_H
