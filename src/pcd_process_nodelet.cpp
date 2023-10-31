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

#include "pcd_process_nodelet/pcd_process_nodelet.h"

namespace PCD_PROCESS
{
void PcdProcessNodelet::onInit()
{
  pnh_ = getPrivateNodeHandle();
  pnh_.param<std::string>("points_topic", points_topic_, "points_in");
  pnh_.param<bool>("publish_image", publish_image_, false);
  pnh_.param<float>("resolution", resolution_, 1.0f);
  pnh_.param<float>("vfov", vfov_, 30.0f);
  pnh_.param<float>("hfov", hfov_, 360.0f);

  pointcloud_sub_ = nh_.subscribe(points_topic_, 1, &PcdProcessNodelet::pointcloudCallback, this);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_out", 1);
  image_x_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_x", 1);
  image_y_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_y", 1);
  image_z_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_z", 1);
  image_intensity_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_intensity", 1);

  output_cloud_msg_.reset(new sensor_msgs::PointCloud2);
  pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void PcdProcessNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud_);

  cloud_mat_ = cloudToMat(pcl_cloud_, vfov_, hfov_, vfov_ * resolution_, hfov_ * resolution_);

  if (publish_image_)
  {
    image_x_msg_ = createImageMsg(cloud_msg, cloud_mat_, 0);
    image_y_msg_ = createImageMsg(cloud_msg, cloud_mat_, 1);
    image_z_msg_ = createImageMsg(cloud_msg, cloud_mat_, 2);
    image_intensity_msg_ = createImageMsg(cloud_msg, cloud_mat_, 3);

    image_x_pub_.publish(image_x_msg_);
    image_y_pub_.publish(image_y_msg_);
    image_z_pub_.publish(image_z_msg_);
    image_intensity_pub_.publish(image_intensity_msg_);
  }

  output_cloud_msg_ = createCloudMsg(cloud_msg, cloud_mat_);
  pointcloud_pub_.publish(output_cloud_msg_);
}

sensor_msgs::ImagePtr PcdProcessNodelet::createImageMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                                        const cv::Mat& cloud_mat, int channel)
{
  cv_bridge::CvImage cv_image;

  cv_image.header = cloud_msg->header;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  std::vector<cv::Mat> channels;
  cv::split(cloud_mat, channels);
  cv_image.image = convertToColorMap(channels[channel]);

  return cv_image.toImageMsg();
}

sensor_msgs::PointCloud2Ptr PcdProcessNodelet::createCloudMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_in,
                                                              const cv::Mat& cloud_mat)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->header.frame_id = cloud_in->header.frame_id;
  pcl_conversions::toPCL(cloud_in->header.stamp, cloud->header.stamp);
  cloud->height = cloud_mat.rows;
  cloud->width = cloud_mat.cols;
  cloud->is_dense = false;
  cloud->points.resize(cloud->height * cloud->width);

  for (int row_idx = 0; row_idx < cloud_mat.rows; row_idx++)
  {
    for (int col_idx = 0; col_idx < cloud_mat.cols; col_idx++)
    {
      pcl::PointXYZI& point = cloud->points[row_idx * cloud_mat.cols + col_idx];
      cv::Vec4f mat_cell = cloud_mat.at<cv::Vec4f>(row_idx, col_idx);

      point.x = mat_cell[0];
      point.y = mat_cell[1];
      point.z = mat_cell[2];
      point.intensity = mat_cell[3];
    }
  }

  sensor_msgs::PointCloud2Ptr cloud_out(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *cloud_out);

  return cloud_out;
}

cv::Mat PcdProcessNodelet::convertToColorMap(const cv::Mat& cloud_mat)
{
  cv::Mat valid_mask, normalized_mat, colored_mat;
  valid_mask = ~cv::Mat(cloud_mat != cloud_mat);

  cloud_mat.copyTo(normalized_mat, valid_mask);
  cv::normalize(cloud_mat, normalized_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::applyColorMap(normalized_mat, colored_mat, cv::COLORMAP_JET);

  return colored_mat;
}

cv::Mat PcdProcessNodelet::cloudToMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const float vfov,
                                      const float hfov, const int mat_row, const int mat_col)
{
  cv::Mat cloud_mat = cv::Mat(mat_row, mat_col, CV_32FC4, cv::Scalar::all(NAN));

  for (pcl::PointXYZI point : cloud->points)
  {
    if (std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) || std::isinf(point.intensity))
    {
      continue;
    }

    int first_row_idx = 0;
    int first_col_idx = 0;
    int last_row_idx = mat_row - 1;
    int last_col_idx = mat_col - 1;
    float azimuth = std::atan2(point.y, point.x);
    float elevation = std::atan2(point.z, std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)));
    float radius = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));

    int calculated_row_idx = int(
        std::min(std::max(std::round(static_cast<float>(mat_row) * (1 - (radToDeg(elevation) + (vfov / 2.0f)) / vfov)),
                          static_cast<float>(first_row_idx)),
                 static_cast<float>(last_row_idx)));
    int calculated_col_idx =
        int(std::min(std::max(std::round(static_cast<float>(mat_col) * ((radToDeg(azimuth) + (hfov / 2.0f)) / hfov)),
                              static_cast<float>(first_col_idx)),
                     static_cast<float>(last_col_idx)));

    cv::Vec4f& mat_cell = cloud_mat.at<cv::Vec4f>(calculated_row_idx, calculated_col_idx);

    if (std::isnan(mat_cell[0]) ||
        radius < std::sqrt(std::pow(mat_cell[0], 2) + std::pow(mat_cell[1], 2) + std::pow(mat_cell[2], 2)))
    {
      mat_cell[0] = point.x;
      mat_cell[1] = point.y;
      mat_cell[2] = point.z;
      mat_cell[3] = point.intensity;
    }
  }

  return cloud_mat;
}

float PcdProcessNodelet::radToDeg(const float rad)
{
  return rad * 180.0f / M_PI;
}

}  // namespace PCD_PROCESS

PLUGINLIB_EXPORT_CLASS(PCD_PROCESS::PcdProcessNodelet, nodelet::Nodelet)
