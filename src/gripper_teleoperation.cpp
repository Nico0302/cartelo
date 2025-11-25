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

#include "cartelo/gripper_teleoperation.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "control_msgs/action/gripper_command.hpp"

namespace cartelo
{

GripperTeleoperation::GripperTeleoperation(const rclcpp::NodeOptions & options)
: Node("gripper_teleoperation", options)
{
  param_listener_ =
    std::make_shared<gripper_teleoperation::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this,
      params_.command_topic);

  if (params_.state_topic != "") {
    state_pub_ = this->create_publisher<std_msgs::msg::Float64>(params_.state_topic, 10);
  }
  if (params_.publishing_rate > 0) {
    publish_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / params_.publishing_rate),
      std::bind(&GripperTeleoperation::publish_state, this));
  }

  joystick_handler_ = std::make_shared<JoystickHandler>(this, params_.joystick.topic);
  // Open button
  if (params_.joystick.open_button >= 0) {
    joystick_handler_->register_on_press(
      params_.joystick.open_button,
      [this]() {
        set_state(1.0);
      });
  }
  // Close button
  if (params_.joystick.close_button >= 0) {
    joystick_handler_->register_on_press(
      params_.joystick.close_button,
      [this]() {
        set_state(0.0);
      });
  }
  // Toggle button
  if (params_.joystick.toggle_button >= 0) {
    joystick_handler_->register_on_press(
      params_.joystick.toggle_button,
      [this]() {
        set_state(1.0 - current_state_);
      });
  }
  // Axis control
  if (params_.joystick.gripper_axis >= 0) {
    joystick_handler_->register_axis(
      params_.joystick.gripper_axis,
      [this](float val) {
        double state = (val + 1.0) / 2.0;
        state = std::max(0.0, std::min(1.0, state));
        set_state(state);
      });
  }
}

GripperTeleoperation::~GripperTeleoperation() {}

void GripperTeleoperation::set_state(double state)
{
  current_state_ = state;
  send_command();
}

void GripperTeleoperation::send_command()
{
  auto goal_msg = control_msgs::action::GripperCommand::Goal();
  goal_msg.command.position = current_state_ * params_.max_position;
  goal_msg.command.max_effort = 0.0;
  gripper_client_->async_send_goal(goal_msg);
}

void GripperTeleoperation::publish_state()
{
  std_msgs::msg::Float64 msg;
  msg.data = current_state_;
  state_pub_->publish(msg);
}

}  // namespace cartelo

RCLCPP_COMPONENTS_REGISTER_NODE(cartelo::GripperTeleoperation)
