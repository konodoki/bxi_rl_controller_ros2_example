//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "include/livox_ros_driver2.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

#include "driver_node.h"
#include "include/ros_headers.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

namespace livox_ros {

namespace {

std::map<std::string, std::string> ParseSerialNameMappings(
    const std::vector<std::string>& entries) {
  std::map<std::string, std::string> mappings;
  for (const std::string& entry : entries) {
    const auto separator = entry.find('=');
    if (separator == std::string::npos || separator == 0 ||
        separator + 1 == entry.size()) {
      throw std::invalid_argument(
          "each lidar_serial_mappings entry must be SERIAL=logical_name: " +
          entry);
    }
    const std::string serial = entry.substr(0, separator);
    const std::string logical_name = entry.substr(separator + 1);
    auto inserted = mappings.emplace(serial, logical_name);
    if (!inserted.second && inserted.first->second != logical_name) {
      throw std::invalid_argument("lidar serial is mapped more than once: " +
                                  serial);
    }
  }
  return mappings;
}

}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions& node_options)
    : Node("livox_driver_node", node_options) {
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s",
              LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 1;
  double publish_freq = 10.0; /* Hz */
  bool auto_connect = true;
  double device_timeout_sec = 3.0;
  std::string frame_id;
  std::string topic_namespace;
  std::string single_lidar_name;
  std::vector<std::string> serial_name_mappings;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", multi_topic);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("auto_connect", true);
  this->declare_parameter("device_timeout_sec", device_timeout_sec);
  this->declare_parameter("topic_namespace", "hardware");
  this->declare_parameter("single_lidar_name", "body_lidar");
  this->declare_parameter("lidar_serial_mappings",
                          std::vector<std::string>{});
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("auto_connect", auto_connect);
  this->get_parameter("device_timeout_sec", device_timeout_sec);
  this->get_parameter("topic_namespace", topic_namespace);
  this->get_parameter("single_lidar_name", single_lidar_name);
  this->get_parameter("lidar_serial_mappings", serial_name_mappings);
  this->get_parameter("frame_id", frame_id);

  if (xfer_format != kPointCloud2Msg && xfer_format != kLivoxCustomMsg) {
    throw std::invalid_argument(
        "xfer_format must be 0 (PointCloud2) or 1 (CustomMsg)");
  }
  if (multi_topic != 0 && multi_topic != 1) {
    throw std::invalid_argument("multi_topic must be 0 or 1");
  }

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribution control. */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, frame_id);
  lddc_ptr_->SetRosNode(this);

  std::string user_config_path;
  this->get_parameter("user_config_path", user_config_path);
  DRIVER_INFO(*this, "Config file: %s", user_config_path.c_str());

  LdsLidar* read_lidar = LdsLidar::GetInstance(publish_freq);
  read_lidar->SetAutoConnectMode(auto_connect);
  read_lidar->ConfigureTopicNaming(
      topic_namespace, single_lidar_name,
      ParseSerialNameMappings(serial_name_mappings), device_timeout_sec);
  DRIVER_INFO(*this, "Automatic LiDAR connection: %s",
              auto_connect ? "enabled" : "disabled");
  DRIVER_INFO(*this, "Per-LiDAR topic fallback: /%s/%s",
              topic_namespace.c_str(), single_lidar_name.c_str());
  lddc_ptr_->RegisterLds(static_cast<Lds*>(read_lidar));

  if (read_lidar->InitLdsLidar(user_config_path)) {
    DRIVER_INFO(*this, "Init lds lidar success!");
  } else {
    DRIVER_ERROR(*this, "Init lds lidar fail!");
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(
      &DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ =
      std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  device_refresh_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [read_lidar]() { read_lidar->RefreshLidarPresence(); });
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

void DriverNode::PointCloudDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread() {
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}
