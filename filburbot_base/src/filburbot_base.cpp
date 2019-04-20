#include <controller_manager/controller_manager.h>
#include "filburbot_base/filburbot_hardware.h"
#include <ros/callback_queue.h>

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(filburbot_base::FilburbotHardware &filburbot,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time) {
  // Calculate elapsed time
  time_source::time_point now_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = now_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = now_time;

  cm.update(ros::Time::now(), elapsed);
  filburbot.sendCommandsToMotors();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "filburbot_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 20.0);

  // Initialize filburbot hardware
  filburbot_base::FilburbotHardware filburbot(nh, private_nh);
  controller_manager::ControllerManager cm(&filburbot, nh);

  ros::CallbackQueue filburbot_queue;
  ros::AsyncSpinner filburbot_spinner(1, &filburbot_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(controlLoop, boost::ref(filburbot), boost::ref(cm), boost::ref(last_time)),
      &filburbot_queue
      );
  ros::Timer control_loop = nh.createTimer(control_timer);

  filburbot_spinner.start();

  ros::spin();

  return 0;
}
