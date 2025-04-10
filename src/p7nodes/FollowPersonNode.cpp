// Copyright 2024 Justo Dario Valverde
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

#include "p7nodes/FollowPersonNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace p7nodes
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

FollowPersonNode::FollowPersonNode()
: LifecycleNode("follow_person"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.3),
  vrot_pid_(0.0, 0.6, -0.1, 0.5)
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  repulsive_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/obstacle", 10,
    std::bind(&FollowPersonNode::repulsive_callback, this, _1));
  repulsive_.x = 0.0;
  repulsive_.y = 0.0;
  repulsive_.z = 0.0;

}

void
FollowPersonNode::repulsive_callback(const geometry_msgs::msg::Vector3::ConstSharedPtr & msg){
  if(msg->x > 0) {
    repulsive_ = *msg;
    repulsive_.y = -repulsive_.y;
    last_obstacle_ = this->now();
  }

}
CallbackReturn
FollowPersonNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");
  bool bf2camera_succeeded = false;
  do{
  if (tf_buffer_.canTransform("base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero, &error)) {
    auto bf2camera_msg = tf_buffer_.lookupTransform("base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero);
    tf2::fromMsg(bf2camera_msg.transform, bf2camera_);
    bf2camera_succeeded = true;
  }
  }while(!bf2camera_succeeded);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");
  timer_ = create_wall_timer(
    50ms, std::bind(&FollowPersonNode::timer_callback, this));
  state_ = SEARCH;
  vel_publisher_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_ = nullptr;
  vel_.angular.z = 0.0;
  vel_.linear.x = 0.0;
  vel_publisher_->publish(vel_);
  vel_publisher_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

double
FollowPersonNode::getYaw(const tf2::Quaternion & q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}
bool
FollowPersonNode::obstacle_is_target(geometry_msgs::msg::Vector3 obs, geometry_msgs::msg::Vector3 target){
  if (std::abs(obs.x - target.x) < 0.3  && std::abs(obs.y - target.y) < 0.2) {
    return true;
  }
  return false;
}

void
FollowPersonNode::timer_callback()
{
  if(this->now() - last_obstacle_ > 1s){
    repulsive_.x = 0.0;
    repulsive_.y = 0.0;
  }


  //Obtener tf a la persona
  tf2::Transform  camera2person;
  tf2::Transform  bf2person; 
  geometry_msgs::msg::TransformStamped bf2person_msg;
  geometry_msgs::msg::Vector3 atractive_;
  if (tf_buffer_.canTransform("camera_rgb_optical_frame", "person", tf2::TimePointZero, &error)) {
    auto camera2person_msg = tf_buffer_.lookupTransform("camera_rgb_optical_frame", "person", tf2::TimePointZero);
    tf2::fromMsg(camera2person_msg.transform, camera2person);
    bf2person = bf2camera_ * camera2person;
    bf2person_msg.transform = tf2::toMsg(bf2person);
    if(this->now()- camera2person_msg.header.stamp > 1.2s) {  
      state_ = SEARCH;
      RCLCPP_INFO(this->get_logger(),"Cambiando a SEARCH");
    }
    else{
      if(state_ != CAPTURED) {
        state_ = CHASE;
      }
    }
    RCLCPP_INFO(this->get_logger(),"Persona a : %fx %fy ",bf2person_msg.transform.translation.x,bf2person_msg.transform.translation.y);
    atractive_.x = bf2person_msg.transform.translation.x;
    atractive_.y = bf2person_msg.transform.translation.y;
    atractive_.z = bf2person_msg.transform.translation.z; 
  }
  //Obtener vector resultante y ver si obstaculo y persona son lo mismo
  geometry_msgs::msg::Vector3 result_;
  float anglediff;
  bool are_the_same = obstacle_is_target(repulsive_,atractive_ );
  if(are_the_same){
    repulsive_.x = 0;
    repulsive_.y = 0;
    RCLCPP_INFO(this->get_logger(), "Son lo mismo");
    anglediff = atan2(atractive_.y, atractive_.x);
  }
  else {
    float angle2target = atan2(atractive_.y, atractive_.x);
    float angle2obstacle = atan2(repulsive_.y, repulsive_.x);
    anglediff = angle2target - angle2obstacle;
  }

  result_.x = atractive_.x - repulsive_.x ;
  result_.y = atractive_.y - repulsive_.y ;
  result_.z = atractive_.z - repulsive_.z ;
  if(result_.x > 1.2 && state_ == CAPTURED) {
    state_ = CHASE;
  }

  //Actuar
  switch(state_) {

  case SEARCH:
    vel_.linear.x = 0.0;
    if (atractive_.y < 0) {
      vel_.angular.z = 0.3;
    }
    else {
      vel_.angular.z = -0.3;
    }
    vel_publisher_->publish(vel_);
    break;


  case CHASE:
    if (atractive_.x < 1.1) {
      vel_.linear.x = 0.0;
      vel_.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(),"Cambiando a CAPTURED");
      state_ = CAPTURED;
      break;
    }
    if(std::abs(anglediff) > 0.5 && !are_the_same){
        angle_ = atan2(result_.y, result_.x);
        vel_rot_ = std::clamp(vrot_pid_.get_output(angle_), -0.5, 0.5);
        dist_ = sqrt(result_.x * result_.x + result_.y * result_.y);
        vel_lin_ = std::clamp(vlin_pid_.get_output(dist_ - 1), -0.3, 0.3);
      }
    else {
      if(anglediff < 0) {
        RCLCPP_INFO(this->get_logger(),"menor");
        vel_rot_ = std::clamp(vrot_pid_.get_output(-0.7), -0.5, 0.5);
      }
      else if(anglediff > 0) {
        RCLCPP_INFO(this->get_logger(),"mayor");
        vel_rot_ = std::clamp(vrot_pid_.get_output(0.7), -0.5, 0.5);
      }
      vel_lin_ = 0.25;
    }
      
    vel_.linear.x = vel_lin_;
    vel_.angular.z = vel_rot_;
    vel_publisher_->publish(vel_);
    break;


  case CAPTURED:
    vel_.linear.x = 0.0;
    vel_.angular.z = 0.0;
    vel_publisher_->publish(vel_);
    RCLCPP_INFO(this->get_logger(),"State cambiado a Captured");
    if (atractive_.x < 0.8) {
        vel_.linear.x = -0.1;
      }
      vel_publisher_->publish(vel_);
    break;
}
}
}  //  namespace p7nodes
