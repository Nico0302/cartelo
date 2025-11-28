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

#include "cartelo/pose_teleoperation.hpp"

#include <chrono>
#include <exception>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cartelo
{

PoseTeleoperation::PoseTeleoperation(const rclcpp::NodeOptions & options)
: Node("pose_teleoperation", options)
{
  param_listener_ =
    std::make_shared<pose_teleoperation::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  joystick_handler_ = std::make_shared<JoystickHandler>(this);
  joystick_handler_->register_on_press(params_.joystick.calibrate_button, [this]() {
      try {
        RCLCPP_DEBUG(this->get_logger(), "Calibrate frame");
        calibrate_frame();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to calibrate frame: %s", e.what());
      }
  });
  joystick_handler_->register_on_press(params_.joystick.teleoperate_button, [this]() {
      try {
        RCLCPP_DEBUG(this->get_logger(), "Start teleoperation");
        start_teleoperation();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start teleoperation: %s", e.what());
      }
  });
  joystick_handler_->register_on_release(params_.joystick.teleoperate_button, [this]() {
      try {
        RCLCPP_DEBUG(this->get_logger(), "Stop teleoperation");
        stop_teleoperation();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop teleoperation: %s", e.what());
      }
  });

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "target_pose", 3);

  double period = 1.0 / params_.update_rate;
  pose_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&PoseTeleoperation::publish_target_pose, this));

  frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), // 10 Hz is sufficient for static transforms
    std::bind(&PoseTeleoperation::broadcast_frame_transform, this));
}

PoseTeleoperation::~PoseTeleoperation() {}


void PoseTeleoperation::calibrate_frame()
{
  frame_transform_ = get_frame_transform();
  broadcast_frame_transform();
}

void PoseTeleoperation::start_teleoperation()
{
  params_ = param_listener_->get_params();
  geometry_msgs::msg::TransformStamped b_T_e_msg, b_T_c_msg;

  try {
    b_T_e_msg = tf_buffer_->lookupTransform(
      params_.base_frame_id,
      params_.end_effector_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(
      "Could not transform " + params_.base_frame_id + " to " + params_.end_effector_frame_id +
        ": " + ex.what());
  }

  try {
    b_T_c_msg = tf_buffer_->lookupTransform(
      params_.base_frame_id,
      params_.controller_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(
      "Could not transform " + params_.base_frame_id + " to " + params_.controller_frame_id + ": " +
        ex.what());
  }

  tf2::Transform b_T_e, b_T_c;
  tf2::fromMsg(b_T_e_msg.transform, b_T_e);
  tf2::fromMsg(b_T_c_msg.transform, b_T_c);

  tf2::Transform delta;
  delta.setOrigin(b_T_c.getOrigin() - b_T_e.getOrigin());
  delta.setRotation(b_T_e.getRotation().inverse() * b_T_c.getRotation());
  delta_ = delta;

  last_controller_transform_.reset();
}

void PoseTeleoperation::stop_teleoperation()
{
  delta_.reset();
  last_controller_transform_.reset();
}

std::optional<geometry_msgs::msg::TransformStamped> PoseTeleoperation::get_frame_transform()
{
  params_ = param_listener_->get_params();
  geometry_msgs::msg::TransformStamped v_T_c_msg;

  try {
    v_T_c_msg = tf_buffer_->lookupTransform(
      params_.virtual_world_frame_id,
      params_.controller_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(
      "Could not transform " + params_.virtual_world_frame_id + " to " +
        params_.controller_frame_id + ": " + ex.what());
  }

  geometry_msgs::msg::TransformStamped v_T_b;
  v_T_b.header.stamp = v_T_c_msg.header.stamp;
  v_T_b.header.frame_id = v_T_c_msg.header.frame_id;
  v_T_b.child_frame_id = params_.base_frame_id;

  tf2::Transform v_T_c;
  tf2::fromMsg(v_T_c_msg.transform, v_T_c);

  tf2::Matrix3x3 rot_mat(
    0, -1, 0,
    0, 0, 1,
    -1, 0, 0);
  tf2::Vector3 trans_vec(0, 0, 0);
  tf2::Transform conversion_transform(rot_mat, trans_vec);

  tf2::Transform v_T_b_transform = v_T_c * conversion_transform;
  v_T_b.transform = tf2::toMsg(v_T_b_transform);

  return v_T_b;
}

void PoseTeleoperation::broadcast_frame_transform()
{
  if (!frame_transform_) {
    // Silently ignore, since calibration has not been performed
    return;
  }

  frame_transform_->header.stamp = this->now();
  tf_broadcaster_->sendTransform(*frame_transform_);
}

void PoseTeleoperation::publish_target_pose()
{
  if (!frame_transform_ || !delta_) {
    // Silently ignore, since teleoperation is not active
    return;
  }

  tf2::Transform b_T_c, b_T_e, target;

  try {
    auto b_T_c_msg = tf_buffer_->lookupTransform(
      params_.base_frame_id,
      params_.controller_frame_id,
      tf2::TimePointZero);
    tf2::fromMsg(b_T_c_msg.transform, b_T_c);
    
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(
      "Could not transform " + params_.base_frame_id + " to " + params_.controller_frame_id + ": " +
        ex.what());
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.base_frame_id;

  if (params_.use_relative_pose) {
    if (!last_controller_transform_) {
      last_controller_transform_ = b_T_c;
      return;
    }

    try {
      auto b_T_e_msg = tf_buffer_->lookupTransform(
        params_.base_frame_id,
        params_.end_effector_frame_id,
        tf2::TimePointZero).transform;
      tf2::fromMsg(b_T_e_msg, b_T_e);
    } catch (const tf2::TransformException & ex) {
      throw std::runtime_error(
        "Could not transform " + params_.base_frame_id + " to " + params_.end_effector_frame_id + ": " +
          ex.what());
    }

    tf2::Transform delta = last_controller_transform_->inverse() * b_T_c;
    
    target = b_T_e * delta.inverse();

    last_controller_transform_ = b_T_c;
  } else {
    target.setOrigin(b_T_c.getOrigin() - delta_->getOrigin());
    target.setRotation(b_T_c.getRotation() * delta_->getRotation().inverse());
  }

    tf2::Vector3 pos = target.getOrigin();
    tf2::Quaternion rot = target.getRotation();

  if (params_.bounds.enabled) {
      pos.setX(
        std::clamp(
          pos.x(),
          params_.bounds.x_min,
          params_.bounds.x_max));
      pos.setY(
        std::clamp(
          pos.y(),
          params_.bounds.y_min,
          params_.bounds.y_max));
      pos.setZ(
        std::clamp(
          pos.z(),
          params_.bounds.z_min,
          params_.bounds.z_max));
    }

    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
    msg.pose.orientation = tf2::toMsg(rot);

  pose_pub_->publish(msg);
}

}  // namespace cartelo

RCLCPP_COMPONENTS_REGISTER_NODE(cartelo::PoseTeleoperation)
