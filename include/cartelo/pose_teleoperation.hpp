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

#ifndef CARTELO__POSE_TELEOPERATION_HPP_
#define CARTELO__POSE_TELEOPERATION_HPP_

#include <memory>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cartelo/pose_teleoperation_parameters.hpp"
#include "cartelo/joystick_handler.hpp"

namespace cartelo
{

class PoseTeleoperation : public rclcpp::Node
{
public:
  explicit PoseTeleoperation(const rclcpp::NodeOptions & options);
  virtual ~PoseTeleoperation();

private:
  /**
   * @brief Calibrate the reference frame for teleoperation by holding the controller close to the robot base.
   *
   */
  void calibrate_frame();

  /**
   * @brief Start teleoperation by calculating the initial transform delta between the controller and the end-effector.
   *
   */
  void start_teleoperation();

  /**
   * @brief Stop teleoperation by clearing the transform delta.
   *
   */
  void stop_teleoperation();

  /**
   * @brief Publish the target end-effector pose based on the controller's current pose and the transform delta.
   *
   */
  void publish_target_pose();

  /**
   * @brief Broadcast the frame transform between the virtual world and the controller.
   *
   */
  void broadcast_frame_transform();

  std::optional<geometry_msgs::msg::TransformStamped> get_frame_transform();

  std::shared_ptr<pose_teleoperation::ParamListener> param_listener_;
  pose_teleoperation::Params params_;

  std::shared_ptr<JoystickHandler> joystick_handler_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr frame_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<tf2::Transform> e_T_c_;
  std::optional<geometry_msgs::msg::TransformStamped> frame_transform_;
};

}  // namespace cartelo

#endif  // CARTELO__POSE_TELEOPERATION_HPP_
