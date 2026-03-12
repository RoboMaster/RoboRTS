/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

#include "gimbal/gimbal.h"
#include "chassis/chassis.h"
#include "referee_system/referee_system.h"
#include "roborts_base_config.h"

namespace {
  std::atomic<bool> g_shutdown_requested{false};

  void SignalHandler(int sig) {
    ROS_INFO("Received signal %d, shutting down gracefully...", sig);
    g_shutdown_requested.store(true);
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "roborts_base_node");
  ros::NodeHandle nh("~");

  // Register signal handlers for graceful shutdown
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  // Load configuration
  roborts_base::Config config;
  config.GetParam(&nh);

  ROS_INFO("Initializing RoboRTS base node...");
  ROS_INFO("Serial port: %s", config.serial_port.c_str());
  ROS_INFO("Loop rate: %d Hz", config.loop_rate);

  // Initialize SDK handle
  auto handle = std::make_shared<roborts_sdk::Handle>(config.serial_port);
  if (!handle->Init()) {
    ROS_FATAL("Failed to initialize SDK handle on port: %s", config.serial_port.c_str());
    return 1;
  }
  ROS_INFO("SDK handle initialized successfully.");

  // Initialize hardware modules
  roborts_base::Chassis chassis(handle);
  roborts_base::Gimbal gimbal(handle);
  roborts_base::RefereeSystem referee_system(handle);

  ROS_INFO("All modules initialized. Entering main loop.");

  // Use configurable loop rate instead of hard-coded usleep
  ros::Rate rate(config.loop_rate);
  while (ros::ok() && !g_shutdown_requested.load()) {
    handle->Spin();
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("RoboRTS base node shutdown complete.");
  return 0;
}