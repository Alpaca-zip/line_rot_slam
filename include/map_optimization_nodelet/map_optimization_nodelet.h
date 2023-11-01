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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <line_rot_slam/OptimizationAction.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

typedef actionlib::SimpleActionServer<line_rot_slam::OptimizationAction> Server;

namespace OPTIMIZATION
{
class MapOptimizationNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher map_pub_;
  ros::Timer timer_;
  std::string points_in_;
  std::string odom_frame_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vector_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  boost::shared_ptr<Server> server_;
  bool optimization_flag_;

public:
  virtual void onInit();
  void broadcastMapFrame(const ros::TimerEvent&);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void execute(const line_rot_slam::OptimizationGoalConstPtr& goal);
  void transformCloud();
};

}  // namespace OPTIMIZATION
