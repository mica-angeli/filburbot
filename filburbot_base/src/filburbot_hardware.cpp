#include "filburbot_base/filburbot_hardware.h"

namespace filburbot_base {

  FilburbotHardware::FilburbotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
  :
  left_wheel("left_wheel_joint"),
  right_wheel("right_wheel_joint"),
  encoder_counts_per_rev_(44.0),
  gear_reduction_(40.0),
  speed_duration_(0.1)
  {
    // Retrieve parameters
    private_nh.param<double>("encoder_counts_per_rev", encoder_counts_per_rev_, encoder_counts_per_rev_);
    private_nh.param<double>("gear_reduction", gear_reduction_, gear_reduction_);
    private_nh.param<double>("speed_duration", speed_duration_, speed_duration_);

    // Register publishers and subscribers
    pub_cmd_diff_ = nh.advertise<filburbot_msgs::CmdDiffVel>("/cmd_diff", 10);
    sub_encoders_ = nh.subscribe("/encoders", 10, &FilburbotHardware::encodersCallback, this);

    registerControlInterfaces();

  }

  void FilburbotHardware::sendCommandsToMotors() {
    filburbot_msgs::CmdDiffVel msg;
    msg.left_speed = static_cast<int>(toEncoderPosition(left_wheel.command) * speed_duration_);
    msg.right_speed = static_cast<int>(toEncoderPosition(right_wheel.command) * speed_duration_);
    pub_cmd_diff_.publish(msg);
  }

  void FilburbotHardware::registerControlInterfaces() {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle left_wheel_joint_state_handle(
        left_wheel.name,
        &left_wheel.position,
        &left_wheel.velocity,
        &left_wheel.effort);
    joint_state_interface_.registerHandle(left_wheel_joint_state_handle);

    hardware_interface::JointStateHandle right_wheel_joint_state_handle(
        right_wheel.name,
        &right_wheel.position,
        &right_wheel.velocity,
        &right_wheel.effort);
    joint_state_interface_.registerHandle(right_wheel_joint_state_handle);

    registerInterface(&joint_state_interface_);

    // Connect and register the joint velocity interface
    hardware_interface::JointHandle left_wheel_joint_handle(left_wheel_joint_state_handle, &left_wheel.command);
    velocity_joint_interface_.registerHandle(left_wheel_joint_handle);

    hardware_interface::JointHandle right_wheel_joint_handle(right_wheel_joint_state_handle, &right_wheel.command);
    velocity_joint_interface_.registerHandle(right_wheel_joint_handle);

    registerInterface(&velocity_joint_interface_);
  }

  void FilburbotHardware::encodersCallback(const filburbot_msgs::Encoders &msg) {
    left_wheel.position = toRadians(static_cast<double>(msg.left_position));
    left_wheel.velocity = toRadians(static_cast<double>(msg.left_speed)) / speed_duration_;

    right_wheel.position = toRadians(static_cast<double>(msg.right_position));
    right_wheel.velocity = toRadians(static_cast<double>(msg.right_speed)) / speed_duration_;
  }
}