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
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <actionlib/server/simple_action_server.h>
#include <line_rot_slam/OptimizationAction.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
typedef actionlib::SimpleActionServer<line_rot_slam::OptimizationAction> Server;

namespace OPTIMIZATION
{
class MapOptimizationNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher marker_pub_;
  ros::Publisher map_pub_;
  ros::Timer timer_;
  std::string points_in_;
  std::string feature_points_in_;
  std::string odom_frame_;
  std::string map_frame_;
  sensor_msgs::PointCloud2ConstPtr cloud_msg_;
  sensor_msgs::PointCloud2ConstPtr feature_cloud_msg_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_cloud_vector_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> feature_cloud_vector_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> transformed_cloud_vector_;
  std::vector<geometry_msgs::Point> old_marker_points_;
  std::vector<Eigen::Affine3f> transform_vector_;
  Eigen::Affine3f prev_transform_;
  Eigen::Affine3f prev_optimization_transform_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_optimized_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_feature_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> feature_cloud_sub_;
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  boost::shared_ptr<Server> server_;
  bool optimization_flag_;
  bool push_back_first_cloud_flag_;
  int marker_id_;
  float last_angle_z_;
  double translation_threshold_;
  double rotation_threshold_;
  double leaf_size_;

public:
  void onInit() override;
  void broadcastMapFrame(const ros::TimerEvent& event);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          const sensor_msgs::PointCloud2ConstPtr& feature_cloud_msg);
  void pushBackTransformAndCloud();
  void pushBackTransformAndCloud(const Eigen::Affine3f& incremental_transform);
  void execute(const line_rot_slam::OptimizationGoalConstPtr& goal);
  void moveOptimization();
  void turnOptimization();
  void calculateTransMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud, Eigen::Affine3f& transform_matrix);
  void calculateRotMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud, Eigen::Matrix3f& rotation_matrix);
  void publishMarkerArray();
  void transformCloud();
  void publishMapCloud();
  bool getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                    Eigen::Affine3f& transform);
  bool shouldPushBackCloud(const Eigen::Affine3f& current_transform, Eigen::Affine3f& incremental_transform);
  bool isRobotStopped();
  double calculateMedianError(std::vector<double>& direction_error_vector);
};

}  // namespace OPTIMIZATION
