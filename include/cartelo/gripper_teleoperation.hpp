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

#ifndef CARTELO__GRIPPER_TELEOPERATION_HPP_
#define CARTELO__GRIPPER_TELEOPERATION_HPP_

#include <memory>

#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "std_msgs/msg/float64.hpp"

#include "cartelo/joystick_handler.hpp"
#include "cartelo/gripper_teleoperation_parameters.hpp"

namespace cartelo
{

class GripperTeleoperation : public rclcpp::Node
{
public:
  explicit GripperTeleoperation(const rclcpp::NodeOptions & options);
  ~GripperTeleoperation();

private:
  /**
  * @brief Set the desired gripper state and send it to the gripper.
  * 
  * @param state 
  */
  void set_state(double state);

  /**
  * @brief Publish the desired, normalized state of the gripper.
  * 
  */
  void publish_state();

  /**
  * @brief Send the command to the gripper.
  * 
  */
  void send_command();

  std::shared_ptr<gripper_teleoperation::ParamListener> param_listener_;
  gripper_teleoperation::Params params_;

  std::shared_ptr<JoystickHandler> joystick_handler_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;

  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

  double current_state_{0.0}; // Current state of the gripper normalized between 0.0 and 1.0
  rclcpp::TimerBase::SharedPtr publish_state_timer_;
};

}  // namespace cartelo

#endif  // CARTELO__GRIPPER_TELEOPERATION_HPP_
