// Copyright 2025 Justo Dario Valverde
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

#ifndef P7NODES__FOLLOWPERSON_NODE_HPP_
#define P7NODES__FOLLOWPERSON_NODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "p7nodes/PIDController.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace p7nodes
{

class FollowPersonNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(FollowPersonNode)
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowPersonNode();
  void timer_callback();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);
  double getYaw(const tf2::Quaternion & q);

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr repulsive_vector_sub_;
  void repulsive_callback(const geometry_msgs::msg::Vector3::ConstSharedPtr & detection);
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist vel_;
  rclcpp::Time last_detection_;
  std::string error;
  tf2::Transform bf2camera_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  static const int SEARCH = 0;
  static const int CHASE = 1;
  static const int CAPTURED = 2;
  int state_;

  geometry_msgs::msg::Vector3 repulsive_;
  PIDController vlin_pid_;
  PIDController vrot_pid_;
  double vel_lin_;
  double vel_rot_;
  double angle_;
  double dist_;

};
}

#endif
