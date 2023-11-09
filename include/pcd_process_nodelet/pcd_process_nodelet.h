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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

namespace PCD_PROCESS
{
class PcdProcessNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher image_x_pub_;
  ros::Publisher image_y_pub_;
  ros::Publisher image_z_pub_;
  ros::Publisher image_intensity_pub_;
  sensor_msgs::PointCloud2Ptr output_cloud_msg_;
  sensor_msgs::ImagePtr image_x_msg_;
  sensor_msgs::ImagePtr image_y_msg_;
  sensor_msgs::ImagePtr image_z_msg_;
  sensor_msgs::ImagePtr image_intensity_msg_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_;
  std::string points_in_;
  std::string points_out_;
  cv::Mat cloud_mat_;
  bool publish_image_;
  float resolution_;
  float vfov_;
  float hfov_;

public:
  void onInit() override;
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  sensor_msgs::ImagePtr createImageMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const cv::Mat& cloud_mat,
                                       int channel);
  sensor_msgs::PointCloud2Ptr createCloudMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             const cv::Mat& cloud_mat);
  cv::Mat convertToColorMap(const cv::Mat& cloud_mat);
  cv::Mat cloudToMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const float vfov, const float hfov,
                     const int mat_row, const int mat_col);
  float radToDeg(const float rad);
};

}  // namespace PCD_PROCESS
