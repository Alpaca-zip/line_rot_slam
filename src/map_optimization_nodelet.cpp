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

  pointcloud_sub_ = nh_.subscribe(points_in_, 1, &MapOptimizationNodelet::pointcloudCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &MapOptimizationNodelet::broadcastMapFrame, this);
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
  prev_transform_ = Eigen::Affine3d::Identity();
  optimization_flag_ = false;

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
  transformStamped.header.frame_id = "map";
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
    cloud_vector_.push_back(pcl_cloud);
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

  publishMapCloud();

  result.success = true;
  server_->setSucceeded(result);
  cloud_vector_.clear();

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

    if (tf_buffer_->canTransform(odom_frame_, cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header).stamp))
    {
      try
      {
        transform_stamped = tf_buffer_->lookupTransform(odom_frame_, cloud->header.frame_id,
                                                        pcl_conversions::fromPCL(cloud->header).stamp);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_DEBUG("Transform error at index %d: %s", i, ex.what());
        continue;
      }

      current_transform = tf2::transformToEigen(transform_stamped);
      incremental_transform = prev_transform_.inverse() * current_transform;
      pcl::transformPointCloud(*cloud, *transformed_cloud, prev_transform_ * incremental_transform);

      transformed_cloud_vector_.push_back(transformed_cloud);
      transform_vector_.push_back(prev_transform_ * incremental_transform);

      prev_transform_ = current_transform;
    }
    else
    {
      ROS_DEBUG("No transform available from %s to %s", cloud->header.frame_id.c_str(), odom_frame_.c_str());
    }
  }
}

void MapOptimizationNodelet::publishMapCloud()
{
  sensor_msgs::PointCloud2Ptr transformed_cloud_msg(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud : transformed_cloud_vector_)
  {
    *map_cloud += *cloud;
  }

  pcl::toROSMsg(*map_cloud, *transformed_cloud_msg);
  transformed_cloud_msg->header.frame_id = "map";
  transformed_cloud_msg->header.stamp = ros::Time::now();

  map_pub_.publish(transformed_cloud_msg);
}

}  // namespace OPTIMIZATION

PLUGINLIB_EXPORT_CLASS(OPTIMIZATION::MapOptimizationNodelet, nodelet::Nodelet)
