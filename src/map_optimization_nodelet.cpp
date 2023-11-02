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

#include "map_optimization_nodelet/map_optimization_nodelet.h"

namespace OPTIMIZATION
{
void MapOptimizationNodelet::onInit()
{
  pnh_ = getPrivateNodeHandle();
  pnh_.param<std::string>("input_points_topic", points_in_, "points_in");
  pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<double>("translation_threshold", translation_threshold_, 0.1);
  pnh_.param<double>("rotation_threshold", rotation_threshold_, 0.1);
  pnh_.param<double>("voxel_grid_leaf_size", leaf_size_, 0.1);

  pointcloud_sub_ = nh_.subscribe(points_in_, 1, &MapOptimizationNodelet::pointcloudCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &MapOptimizationNodelet::broadcastMapFrame, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("optimization_marker", 1);
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
  prev_transform_ = Eigen::Affine3d::Identity();
  optimization_flag_ = false;
  marker_id_ = 0;

  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster);
  server_.reset(new Server(nh_, "optimization", boost::bind(&MapOptimizationNodelet::execute, this, _1), false));
  server_->start();
}

void MapOptimizationNodelet::broadcastMapFrame(const ros::TimerEvent&)
{
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = map_frame_;
  transformStamped.child_frame_id = odom_frame_;

  // TODO: get transform from optimization result

  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  tf_broadcaster_->sendTransform(transformStamped);
}

void MapOptimizationNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (!optimization_flag_)
  {
    Eigen::Affine3d current_transform;

    if (getTransform(odom_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, current_transform))
    {
      if (shouldPushBackCloud(current_transform))
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
        cloud_vector_.push_back(pcl_cloud);
        transform_vector_.push_back(current_transform);

        prev_transform_ = current_transform;
      }
    }
    else
    {
      ROS_DEBUG("No transform available from %s to %s", cloud_msg->header.frame_id.c_str(), odom_frame_.c_str());
    }
  }
}

void MapOptimizationNodelet::execute(const line_rot_slam::OptimizationGoalConstPtr& goal)
{
  line_rot_slam::OptimizationResult result;

  optimization_flag_ = true;

  transformCloud();

  /* TODO
  if (goal->state == "MOVE")
  {
    moveOptimization();
  }
  else (goal->state == "TURN")
  {
    turnOptimization();
  }
  */

  if (goal->state == "MOVE")
  {
    publishMarkerArray();
  }

  publishMapCloud();

  result.success = true;
  server_->setSucceeded(result);
  cloud_vector_.clear();
  transformed_cloud_vector_.clear();
  transform_vector_.clear();

  optimization_flag_ = false;
}

void MapOptimizationNodelet::transformCloud()
{
  geometry_msgs::TransformStamped transform_stamped;
  Eigen::Affine3d current_transform;
  Eigen::Affine3d incremental_transform;

  for (int i = 0; i < cloud_vector_.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = cloud_vector_[i];
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_vector_[i]);
    transformed_cloud_vector_.push_back(transformed_cloud);
  }
}

void MapOptimizationNodelet::publishMarkerArray()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker cube;

  cube.header.frame_id = map_frame_;
  cube.header.stamp = ros::Time::now();
  cube.ns = "cube";
  cube.id = marker_id_++;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.scale.x = 0.05;
  cube.scale.y = 0.05;
  cube.scale.z = 0.05;
  cube.color.r = 0.0;
  cube.color.g = 1.0;
  cube.color.b = 0.0;
  cube.color.a = 1.0;
  cube.pose.orientation.w = 1.0;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = map_frame_;
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "line_strip";
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.scale.x = 0.01;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;
  line_strip.pose.orientation.w = 1.0;

  if (!transform_vector_.empty())
  {
    Eigen::Vector3d translation = transform_vector_.back().translation();
    geometry_msgs::Point point;
    point.x = translation.x();
    point.y = translation.y();
    point.z = translation.z();

    cube.pose.position = point;
    marker_array.markers.push_back(cube);

    for (const geometry_msgs::Point& old_point : old_marker_points_)
    {
      line_strip.points.push_back(old_point);
    }

    line_strip.points.push_back(point);
    old_marker_points_.push_back(point);
  }

  if (line_strip.points.size() >= 2)
  {
    marker_array.markers.push_back(line_strip);
  }

  marker_pub_.publish(marker_array);
}

void MapOptimizationNodelet::publishMapCloud()
{
  sensor_msgs::PointCloud2Ptr map_cloud_msg(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud : transformed_cloud_vector_)
  {
    *map_cloud_ += *cloud;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(map_cloud_);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid.filter(*downsampled_cloud);

  pcl::toROSMsg(*map_cloud_, *map_cloud_msg);
  map_cloud_msg->header.frame_id = map_frame_;
  map_cloud_msg->header.stamp = ros::Time::now();

  map_pub_.publish(map_cloud_msg);
}

bool MapOptimizationNodelet::getTransform(const std::string& target_frame, const std::string& source_frame,
                                          const ros::Time& time, Eigen::Affine3d& transform)
{
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time);
    transform = tf2::transformToEigen(transform_stamped);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_DEBUG("Transform error: %s", ex.what());
    return false;
  }
}

bool MapOptimizationNodelet::shouldPushBackCloud(const Eigen::Affine3d& current_transform)
{
  Eigen::Affine3d incremental_transform;
  Eigen::Vector3d euler_angle;
  double translation_distance;
  double rotation_angle;

  incremental_transform = prev_transform_.inverse() * current_transform;

  euler_angle = incremental_transform.rotation().eulerAngles(0, 1, 2);
  rotation_angle = std::abs(euler_angle[2]);

  translation_distance = incremental_transform.translation().norm();

  return translation_distance >= translation_threshold_ || rotation_angle >= rotation_threshold_;
}

}  // namespace OPTIMIZATION

PLUGINLIB_EXPORT_CLASS(OPTIMIZATION::MapOptimizationNodelet, nodelet::Nodelet)
