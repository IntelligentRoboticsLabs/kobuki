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

#include "kobuki/FakeBumper.hpp"

namespace kobuki
{
FakeBumper::FakeBumper()
: Node("fake_bumper")
{
  pressed_.state = BumperEvent::RELEASED;

  bumper_pub_ =
    this->create_publisher<BumperEvent>("/events/bumper", 10);
  scan_sub_ = create_subscription<LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&FakeBumper::laserCallback, this, _1));
}

void
FakeBumper::laserCallback(const LaserScan::UniquePtr msg)
{
  if (pressed_.state) {

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      if (msg->ranges[i] < OBSTACLE_DISTANCE) {
        return;
      }
    }

    pressed_.state = BumperEvent::RELEASED;
    bumper_pub_->publish(pressed_);
  } else {
    size_t pos = 0, threshold_left = msg->ranges.size() / 4,
      threshold_right = msg->ranges.size() - threshold_left;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      if (msg->ranges[i] < OBSTACLE_DISTANCE && (i<threshold_left || i> threshold_right)) {
        pressed_.state = BumperEvent::PRESSED;
        pos = i;
        break;
      }
    }

    if (!pressed_.state) {return;}

    size_t threshold_center_left = threshold_left / 3,
      threshold_center_right = msg->ranges.size() - threshold_center_left;

    if (pos > threshold_center_left && pos < threshold_left) {
      pressed_.bumper = BumperEvent::LEFT;
    } else if (pos > threshold_right && pos < threshold_center_right) {
      pressed_.bumper = BumperEvent::RIGHT;
    } else {
      pressed_.bumper = BumperEvent::CENTER;
    }

    bumper_pub_->publish(pressed_);
  }
}
}  // namespace kobuki
