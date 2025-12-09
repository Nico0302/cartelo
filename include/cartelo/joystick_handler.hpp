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
  /**
   * @brief Callback function type for button presses and releases.
   *
   */
  using ButtonCallback = std::function<void()>;

  /**
   * @brief Callback function type for axis changes.

   * @param float The new value of the axis in the range [-1.0, 1.0].
   */
  using AxisCallback = std::function<void(float)>;

  /**
   * @brief Construct a new Joystick Handler object
   *
   * @param node ROS 2 node pointer.
   * @param frame_id Frame name of the joystick to subscribe to.
   * @param topic_name Name of the joystick topic to subscribe to.
   */
  JoystickHandler(rclcpp::Node * node, const std::string & frame_id, const std::string & topic_name = "joy");

  /**
   * @brief Register a callback to be called when a button is pressed.
   *
   * @param button_index Index of the button.
   * @param cb Callback function.
   */
  void register_on_press(int button_index, ButtonCallback cb);

  /**
   * @brief Register a callback to be called when a button is released.
   *
   * @param button_index Index of the button.
   * @param cb Callback function.
   */
  void register_on_release(int button_index, ButtonCallback cb);

  /**
   * @brief Register a callback to be called when an axis changes.
   *
   * @param axis_index Index of the axis.
   * @param cb Callback function taking the new axis value.
   * @param threshold Optional threshold for the axis change.
   */
  void register_on_axis_change(int axis_index, AxisCallback cb, float threshold = 0.01);

private:
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  using AxisConfig = std::pair<AxisCallback, float>;

  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::vector<int> last_buttons_;
  std::vector<float> last_axes_;
  std::map<int, std::vector<ButtonCallback>> on_press_callbacks_;
  std::map<int, std::vector<ButtonCallback>> on_release_callbacks_;
  std::map<int, std::vector<AxisConfig>> axis_configs_;
};

}  // namespace cartelo

#endif  // CARTELO__JOYSTICK_HANDLER_HPP_
