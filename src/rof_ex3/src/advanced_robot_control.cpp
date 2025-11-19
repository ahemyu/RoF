// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "jetbot_gazebo/advanced_robot_control.h"

using namespace std::chrono_literals;

void AdvancedRobotControl::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Read out the current position (x, y) and quaternion (x, y, z, w),
  // convert the quaternion to Euler-angle (roll, pitch, yaw)
  // read out the current_theta from yaw
 








}

void AdvancedRobotControl::laser_front_callback(
    const sensor_msgs::msg::Range::SharedPtr msg)
{
  // If current measured value is within the measuring range of the sensor
  // save in variable






}

void AdvancedRobotControl::laser_side_callback(
    const sensor_msgs::msg::Range::SharedPtr msg)
{
  // If current measured value is within the measuring range of the sensor
  // save in variable







  // If the measured value is above 10m or outside the measuring range,
  // no object was detected in front of the sensor
  // Use it as an indicator that the end of a box has been reached







  // If the measured value falls below a value of 1.5m, an object was
  // in front of the sensor is detected Used as an indicator that the beginning
  // of a box has been reached













}

void AdvancedRobotControl::setpoint_callback()
{
  // Resetting the speeds causes the robot to decelerate if the
  // value is not overwritten again below
  geometry_msgs::msg::Twist vel_msg;

  // As long as the minimum distance in the direction of travel
  // has not been undercut Travel forward at 0.5 m/s











  // If the first position has been reached and the robot has not yet rotated
  // a rotation speed is specified













  // After the rotation a linear velocity is given to drive to the target point
  // additionally small rotations are compensated to drive parallel to the boxes







  
  // At the target position the velocities are set to zero









  publisher_->publish(vel_msg);
}

AdvancedRobotControl::AdvancedRobotControl()
    : Node("navigation_node_solution"), count_(0)
{

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("jetbot/cmd_vel", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "jetbot/odom", 1,
      std::bind(&AdvancedRobotControl::odom_callback, this,
                std::placeholders::_1));

  distance_side_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "jetbot/laser_side", 1,
      std::bind(&AdvancedRobotControl::laser_side_callback, this,
                std::placeholders::_1));

  distance_front_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
      "jetbot/laser_front", 1,
      std::bind(&AdvancedRobotControl::laser_front_callback, this,
                std::placeholders::_1));

  timer_ = this->create_wall_timer(
      100ms, std::bind(&AdvancedRobotControl::setpoint_callback, this));
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedRobotControl>());
  rclcpp::shutdown();
  return 0;
}