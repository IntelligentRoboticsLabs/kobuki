// Copyright 2024 Juan Carlos Manzanares Serrano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KOBUKI__FAKE_BUMPER_HPP_
#define KOBUKI__FAKE_BUMPER_HPP_

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace kobuki
{

using std::placeholders::_1;

class FakeBumper : public rclcpp::Node
{
public:
  FakeBumper();

private:
  using LaserScan = sensor_msgs::msg::LaserScan;
  using BumperEvent = kobuki_ros_interfaces::msg::BumperEvent;

  void laserCallback(const LaserScan::UniquePtr msg);

  rclcpp::Publisher<BumperEvent>::SharedPtr bumper_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

  BumperEvent pressed_;

  static constexpr float OBSTACLE_DISTANCE = 0.2f;
};

}  // namespace kobuki

#endif  // KOBUKI__FAKE_BUMPER_HPP_
