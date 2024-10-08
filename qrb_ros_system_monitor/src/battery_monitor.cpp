// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_system_monitor/battery_monitor.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

using namespace std::chrono_literals;

#define POWER_SUPPLY_SYSFS_PATH "/sys/class/power_supply/"
#define SOC_ID_SYSFS_PATH "/sys/devices/soc0/soc_id"

namespace qrb_ros_system_monitor
{

BatteryMonitor::BatteryMonitor(const rclcpp::NodeOptions & options)
  : MonitorNode("battery_monitor", options)
{
  RCLCPP_INFO(this->get_logger(), "Battery Monitor start");
  pub_ = this->create_publisher<std_msgs::msg::Float32>("battery", 10);
  timer_ = create_wall_timer(
      std::chrono::seconds(interval_), std::bind(&BatteryMonitor::on_timer, this));
}

void BatteryMonitor::on_timer()
{
  auto msg = std::make_unique<std_msgs::msg::Float32>();
  get_battery_info(*msg);
  pub_->publish(std::move(msg));
}

void BatteryMonitor::get_battery_info(std_msgs::msg::Float32 & info)
{
  std::ifstream soc_id_file(SOC_ID_SYSFS_PATH);
  if (!soc_id_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "open %s error", SOC_ID_SYSFS_PATH);
    return;
  }

  std::string line;
  std::getline(soc_id_file, line);
  soc_id_file.close();

  auto soc_id = -1;
  std::istringstream iss(line);
  iss >> soc_id;

  auto battery_sysfs_path = std::string(POWER_SUPPLY_SYSFS_PATH);

  // kona: 455, kailua: 603, qcm6490: 498
  if (soc_id == 455 || soc_id == 603) {
    battery_sysfs_path += "battery/capacity";
  } else if (soc_id == 498) {
    battery_sysfs_path += "qcom-battmgr-bat/capacity";
  } else {
    RCLCPP_ERROR(this->get_logger(), "no battery sysfs found");
    return;
  }

  std::ifstream battery_file(battery_sysfs_path);
  if (!battery_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "open %s error", battery_sysfs_path.c_str());
    return;
  }
  line.clear();
  std::getline(battery_file, line);
  battery_file.close();

  std::istringstream battery_iss(line);
  battery_iss >> info.data;
}

}  // namespace qrb_ros_system_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros_system_monitor::BatteryMonitor)