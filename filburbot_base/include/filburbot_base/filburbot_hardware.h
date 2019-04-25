#ifndef FILBURBOT_BASE_FILBURBOT_HARDWARE_H
#define FILBURBOT_BASE_FILBURBOT_HARDWARE_H

#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <filburbot_msgs/Encoders.h>
#include <filburbot_msgs/CmdDiffVel.h>
#include <ros/ros.h>

namespace filburbot_base {

  class FilburbotHardware : public hardware_interface::RobotHW
  {
  public:
    FilburbotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

    void sendCommandsToMotors();

  private:
    struct Joint {
      std::string name;
      double command;
      double position;
      double velocity;
      double effort;

      Joint(const std::string &name): name(name), command(0.0), position(0.0), velocity(0.0), effort(0.0) {}
    };
    void encodersCallback(const filburbot_msgs::Encoders &msg);

    void registerControlInterfaces();

    inline double toRadians(double encoder_position) {
      return ((encoder_position / encoder_counts_per_rev_) / gear_reduction_) * 2.0 * M_PI;
    }

    inline double toEncoderPosition(double radians) {
      return (radians / (2.0 * M_PI)) * gear_reduction_ * encoder_counts_per_rev_;
    }

  private:
    ros::Subscriber sub_encoders_;
    ros::Publisher pub_cmd_diff_;

    Joint left_wheel;
    Joint right_wheel;

    double encoder_counts_per_rev_;
    double gear_reduction_;
    double speed_duration_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

  };
}

#endif //FILBURBOT_BASE_FILBURBOT_HARDWARE_H
