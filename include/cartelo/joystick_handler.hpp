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

#ifndef CARTELO__JOYSTICK_HANDLER_HPP_
#define CARTELO__JOYSTICK_HANDLER_HPP_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace cartelo
{
/**
 * @brief Joystick handler class.
 *
 * Handles joystick input and provides callbacks for button presses and releases.
 */
class JoystickHandler
{
public:
  using Callback = std::function<void()>;

  /**
   * @brief Construct a new Joystick Handler object
   *
   * @param node ROS 2 node.
   * @param topic_name Name of the joystick topic to subscribe to.
   */
  JoystickHandler(rclcpp::Node * node, const std::string & topic_name);

  /**
   * @brief Register a callback to be called when a button is pressed.
   *
   * @param button_index Index of the button.
   * @param cb Callback function.
   */
  void register_on_press(int button_index, Callback cb);

  /**
   * @brief Register a callback to be called when a button is released.
   *
   * @param button_index Index of the button.
   * @param cb Callback function.
   */
  void register_on_release(int button_index, Callback cb);

private:
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::vector<int> last_buttons_;
  std::map<int, std::vector<Callback>> on_press_callbacks_;
  std::map<int, std::vector<Callback>> on_release_callbacks_;
};

}  // namespace cartelo

#endif  // CARTELO__JOYSTICK_HANDLER_HPP_
