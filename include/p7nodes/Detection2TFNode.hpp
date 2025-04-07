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

#ifndef P7NODES__DETECTION2TFNODE_HPP_
#define P7NODES__DETECTION2TFNODE_HPP_


#include <memory>

#include "p7nodes/PIDController.hpp"


#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "rclcpp/node.hpp"

namespace p7nodes {
class Detection2TFNode : public rclcpp::Node
{
public:
  using Detection3D = vision_msgs::msg::Detection3DArray;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  Detection2TFNode();
private:
  void detection3d_callback(const Detection3D::ConstSharedPtr& detection);
  rclcpp::Subscription<Detection3D>::SharedPtr detection_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  TransformStamped transform_;


};
}

#endif