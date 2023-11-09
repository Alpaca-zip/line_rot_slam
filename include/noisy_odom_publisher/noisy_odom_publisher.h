/**
 * Copyright (C) 2023  Alpaca-zip
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>

class NoisyOdomPublisher
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  nav_msgs::Odometry prev_odom_;
  std::default_random_engine generator_;
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf;
  bool odom_initialized_;
  double translation_noise_factor_;
  double rotation_noise_factor_;
  double translation_threshold_;
  double rotation_threshold_;
  double pose_[3];

public:
  NoisyOdomPublisher();
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void initializeOdom(const nav_msgs::OdometryConstPtr& odom_msg);
  void computeTransformAndRotation(const nav_msgs::OdometryConstPtr& odom_msg, double& transform, double& rotation);
  void addNoiseToTransformAndRotation(double& transform, double& rotation);
  void updatePose(const double& transform, const double& rotation);
  void publishOdom(const nav_msgs::OdometryConstPtr& odom_msg);
  double getYaw(const tf2::Quaternion& quat);
};
