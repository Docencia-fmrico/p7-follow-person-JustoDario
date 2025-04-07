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

#include "p7nodes/Detection2TFNode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace p7nodes{

using std::placeholders::_1;

Detection2TFNode::Detection2TFNode()
: Node("detection_2_tf")
{
  detection_sub_ = create_subscription<Detection3D>(
    "/detections_3d",
    rclcpp::SensorDataQoS().reliable(),
    std::bind(&Detection2TFNode::detection3d_callback, this, _1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void
Detection2TFNode::detection3d_callback(const Detection3D::ConstSharedPtr& detection_msg)
{
  for(const auto& detection : detection_msg->detections) {
    if(detection.results[0].hypothesis.class_id == "person") {
      transform_.header.frame_id = detection_msg->header.frame_id;
      transform_.header.stamp = detection_msg->header.stamp;
      transform_.child_frame_id = "person";
      transform_.transform.translation.x = detection.bbox.center.position.x;
      transform_.transform.translation.y = detection.bbox.center.position.y; 
      transform_.transform.translation.z = detection.bbox.center.position.z;
      tf_broadcaster_->sendTransform(transform_);
      return; //Nos aseguramos de enviar solo la primera persona vista
    }
  }
}

}