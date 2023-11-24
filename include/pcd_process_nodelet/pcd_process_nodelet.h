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

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(const smoothness_t& left, const smoothness_t& right)
  {
    return left.value < right.value;
  }
};

namespace PCD_PROCESS
{
class PcdProcessNodelet : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher surface_pointcloud_pub_;
  ros::Publisher corner_pointcloud_pub_;
  ros::Publisher image_x_pub_;
  ros::Publisher image_y_pub_;
  ros::Publisher image_z_pub_;
  ros::Publisher image_intensity_pub_;
  sensor_msgs::PointCloud2Ptr output_cloud_msg_;
  sensor_msgs::PointCloud2Ptr output_surface_cloud_msg_;
  sensor_msgs::PointCloud2Ptr output_corner_cloud_msg_;
  sensor_msgs::ImagePtr image_x_msg_;
  sensor_msgs::ImagePtr image_y_msg_;
  sensor_msgs::ImagePtr image_z_msg_;
  sensor_msgs::ImagePtr image_intensity_msg_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_;
  std::vector<int> start_ring_index_;
  std::vector<int> end_ring_index_;
  std::vector<int> point_col_index_;
  std::vector<float> point_range_;
  std::vector<smoothness_t> cloud_smoothness_;
  std::string points_in_;
  std::string points_out_;
  std::string surface_points_out_;
  std::string corner_points_out_;
  cv::Mat cloud_mat_;
  bool publish_image_;
  int* cloud_neighbor_picked_;
  int* cloud_label_;
  float resolution_;
  float vfov_;
  float hfov_;
  float surface_threshold_;
  float corner_threshold_;
  float* cloud_curvature_;

public:
  void onInit() override;
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void extractFeatureCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in, const cv::Mat& cloud_mat);
  void extractClouds(const cv::Mat& cloud_mat);
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr& surface_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud);
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
