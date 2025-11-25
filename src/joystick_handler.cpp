// Copyright 2025 Nicolas Gres
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

#include "cartelo/joystick_handler.hpp"

namespace cartelo
{

JoystickHandler::JoystickHandler(rclcpp::Node * node, const std::string & topic_name)
{
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
    topic_name, 1,
    std::bind(&JoystickHandler::joystick_callback, this, std::placeholders::_1));
}

void JoystickHandler::register_on_press(int button_index, Callback cb)
{
  on_press_callbacks_[button_index].push_back(cb);
}

void JoystickHandler::register_on_release(int button_index, Callback cb)
{
  on_release_callbacks_[button_index].push_back(cb);
}

void JoystickHandler::register_axis(int axis_index, std::function<void(float)> cb)
{
  axis_callbacks_[axis_index].push_back(cb);
}

void JoystickHandler::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.empty() && msg->axes.empty()) {
    return;
  }

  if (last_buttons_.empty()) {
    last_buttons_.resize(msg->buttons.size(), 0);
  }

  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    // Check for press (0 -> 1)
    if (msg->buttons[i] == 1 && last_buttons_[i] == 0) {
      if (on_press_callbacks_.count(i)) {
        for (const auto & cb : on_press_callbacks_[i]) {
          cb();
        }
      }
    }
    // Check for release (1 -> 0)
    else if (msg->buttons[i] == 0 && last_buttons_[i] == 1) {
      if (on_release_callbacks_.count(i)) {
        for (const auto & cb : on_release_callbacks_[i]) {
          cb();
        }
      }
    }
  }

  for (size_t i = 0; i < msg->axes.size(); ++i) {
    if (axis_callbacks_.count(i)) {
      for (const auto & cb : axis_callbacks_[i]) {
        cb(msg->axes[i]);
      }
    }
  }

  last_buttons_ = msg->buttons;
}

}  // namespace cartelo
