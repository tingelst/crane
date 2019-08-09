// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the MIT license, see LICENSE

// Author: Lars Tingelstad (NTNU) <lars.tingelstad@ntnu.no>

#include <chrono>

#include <crane_hw_interface/crane_hw_interface_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crane_hw_interface");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;

  crane_hw_interface::CraneHardwareInterfaceSim hardware_interface;
  hardware_interface.init();

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  controller_manager::ControllerManager controller_manager(&hardware_interface, nh);

  hardware_interface.start();

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  ros::Rate rate(10);  // 100 hz
  while (ros::ok())
  {
    // Receive current state from robot
    hardware_interface.read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    hardware_interface.write(timestamp, period);

    rate.sleep();
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("crane_hw_interface", "Shutting down.");

  return 0;
}
