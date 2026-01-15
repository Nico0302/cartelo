// Copyright 2026 Nicolas Gres
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

#ifndef CARTELO__TWIST_TELEOPERATION_HPP_
#define CARTELO__TWIST_TELEOPERATION_HPP_

#include <memory>
#include <mutex>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cartelo/twist_teleoperation_parameters.hpp"
#include "cartelo/joystick_handler.hpp"
#include "cartelo/homing_handler.hpp"

namespace cartelo
{

class TwistTeleoperation : public rclcpp::Node
{
public:
  explicit TwistTeleoperation(const rclcpp::NodeOptions& options);
  virtual ~TwistTeleoperation();

private:
  /**
   * @brief Trigger the homing sequence.
   *
   */
  void trigger_homing();

  /**
   * @brief Startup initialization to get initial pose from robot.
   *
   */
  void startup();

  /**
   * @brief Integrate twist message into current pose.
   *
   * @param msg The twist message.
   */
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Publish the target end-effector pose.
   *
   */
  void publish_target_pose();

  std::shared_ptr<twist_teleoperation::ParamListener> param_listener_;
  twist_teleoperation::Params params_;

  std::shared_ptr<JoystickHandler> joystick_handler_;
  std::shared_ptr<HomingHandler> homing_handler_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Timer for startup check
  rclcpp::TimerBase::SharedPtr startup_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Integration state
  tf2::Vector3 pos_;
  tf2::Quaternion rot_;

  // Time tracking for integration
  rclcpp::Time last_twist_time_;
  bool first_twist_received_{ false };

  bool startup_done_{ false };
  bool is_homed_{ false };
};

}  // namespace cartelo

#endif  // CARTELO__TWIST_TELEOPERATION_HPP_
