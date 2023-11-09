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

#include "noisy_odom_publisher/noisy_odom_publisher.h"

NoisyOdomPublisher::NoisyOdomPublisher() : pnh_("~")
{
  pnh_.param<bool>("publish_tf", publish_tf, false);
  pnh_.param<double>("translation_noise_factor", translation_noise_factor_, 1.0);
  pnh_.param<double>("rotation_noise_factor", rotation_noise_factor_, 1.0);
  pnh_.param<double>("translation_threshold", translation_threshold_, 0.001);
  pnh_.param<double>("rotation_threshold", rotation_threshold_, 0.001);

  odom_sub_ = nh_.subscribe("odom", 1, &NoisyOdomPublisher::odomCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_noise", 1);
  odom_initialized_ = false;

  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void NoisyOdomPublisher::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  if (!odom_initialized_)
  {
    initializeOdom(odom_msg);
    return;
  }

  double transform, rotation;

  computeTransformAndRotation(odom_msg, transform, rotation);
  addNoiseToTransformAndRotation(transform, rotation);
  updatePose(transform, rotation);
  publishOdom(odom_msg);

  prev_odom_ = *odom_msg;
}

void NoisyOdomPublisher::initializeOdom(const nav_msgs::OdometryConstPtr& odom_msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(odom_msg->pose.pose.orientation, quat);

  prev_odom_ = *odom_msg;
  pose_[0] = odom_msg->pose.pose.position.x;
  pose_[1] = odom_msg->pose.pose.position.y;
  pose_[2] = getYaw(quat);

  odom_initialized_ = true;
}

void NoisyOdomPublisher::computeTransformAndRotation(const nav_msgs::OdometryConstPtr& odom_msg, double& transform,
                                                     double& rotation)
{
  double dx = odom_msg->pose.pose.position.x - prev_odom_.pose.pose.position.x;
  double dy = odom_msg->pose.pose.position.y - prev_odom_.pose.pose.position.y;

  tf2::Quaternion prev_quat, current_quat;
  tf2::fromMsg(prev_odom_.pose.pose.orientation, prev_quat);
  tf2::fromMsg(odom_msg->pose.pose.orientation, current_quat);

  double prev_yaw = getYaw(prev_quat);
  double current_yaw = getYaw(current_quat);

  transform = hypot(dx, dy);
  rotation = current_yaw - prev_yaw;
}

void NoisyOdomPublisher::addNoiseToTransformAndRotation(double& transform, double& rotation)
{
  double transform_noise_std_dev = translation_noise_factor_ * transform;
  double rotation_noise_std_dev = rotation_noise_factor_ * fabs(rotation);

  std::normal_distribution<> dist_transform(0, transform_noise_std_dev);
  std::normal_distribution<> dist_rotation(0, rotation_noise_std_dev);

  if (transform > translation_threshold_)
  {
    transform += dist_transform(generator_);
  }

  if (fabs(rotation) > rotation_threshold_)
  {
    rotation += dist_rotation(generator_);
  }
}

void NoisyOdomPublisher::updatePose(const double& transform, const double& rotation)
{
  tf2::Quaternion prev_quat;
  tf2::fromMsg(prev_odom_.pose.pose.orientation, prev_quat);

  double prev_yaw = getYaw(prev_quat);

  pose_[0] += transform * cos(prev_yaw + rotation);
  pose_[1] += transform * sin(prev_yaw + rotation);
  pose_[2] += rotation;
}

void NoisyOdomPublisher::publishOdom(const nav_msgs::OdometryConstPtr& odom_msg)
{
  nav_msgs::Odometry odom_noise_msg;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose_[2]);

  odom_noise_msg.header = odom_msg->header;
  odom_noise_msg.child_frame_id = odom_msg->child_frame_id;
  odom_noise_msg.pose.pose.position.x = pose_[0];
  odom_noise_msg.pose.pose.position.y = pose_[1];
  odom_noise_msg.pose.pose.position.z = odom_msg->pose.pose.position.z;
  odom_noise_msg.pose.pose.orientation = tf2::toMsg(quat);
  odom_noise_msg.twist.twist = odom_msg->twist.twist;

  odom_pub_.publish(odom_noise_msg);

  if (publish_tf)
  {
    geometry_msgs::TransformStamped odom_noise_tf;
    odom_noise_tf.header.stamp = odom_noise_msg.header.stamp;
    odom_noise_tf.header.frame_id = odom_noise_msg.header.frame_id;
    odom_noise_tf.child_frame_id = odom_noise_msg.child_frame_id;
    odom_noise_tf.transform.translation.x = pose_[0];
    odom_noise_tf.transform.translation.y = pose_[1];
    odom_noise_tf.transform.translation.z = odom_msg->pose.pose.position.z;
    odom_noise_tf.transform.rotation = tf2::toMsg(quat);

    tf_broadcaster_->sendTransform(odom_noise_tf);
  }
}

double NoisyOdomPublisher::getYaw(const tf2::Quaternion& quat)
{
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  return yaw;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noisy_odom_publisher_node");
  NoisyOdomPublisher node;
  ros::spin();

  return 0;
}
