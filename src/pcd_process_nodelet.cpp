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
  pnh_.param<std::string>("input_points_topic", points_in_, "points_in");
  pnh_.param<std::string>("output_points_topic", points_out_, "points_out");
  pnh_.param<std::string>("output_surface_points_topic", surface_points_out_, "surface_points_out");
  pnh_.param<std::string>("output_corner_points_topic", corner_points_out_, "corner_points_out");
  pnh_.param<bool>("publish_image", publish_image_, false);
  pnh_.param<float>("resolution", resolution_, 1.0f);
  pnh_.param<float>("vfov", vfov_, 30.0f);
  pnh_.param<float>("hfov", hfov_, 360.0f);
  pnh_.param<float>("surface_threshold", surface_threshold_, 0.1f);
  pnh_.param<float>("corner_threshold", corner_threshold_, 1.0f);

  pointcloud_sub_ = nh_.subscribe(points_in_, 1, &PcdProcessNodelet::pointcloudCallback, this);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_out_, 1);
  surface_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(surface_points_out_, 1);
  corner_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(corner_points_out_, 1);
  image_x_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_x", 1);
  image_y_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_y", 1);
  image_z_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_z", 1);
  image_intensity_pub_ = nh_.advertise<sensor_msgs::Image>("image/point_intensity", 1);
  cloud_neighbor_picked_ = new int[static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_)];
  cloud_label_ = new int[static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_)];
  cloud_curvature_ = new float[static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_)];

  output_cloud_msg_.reset(new sensor_msgs::PointCloud2);
  output_surface_cloud_msg_.reset(new sensor_msgs::PointCloud2);
  output_corner_cloud_msg_.reset(new sensor_msgs::PointCloud2);
  pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  extracted_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  surface_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cloud_smoothness_.resize(vfov_ * resolution_ * hfov_ * resolution_);
  start_ring_index_.assign(vfov_ * resolution_, 0);
  end_ring_index_.assign(vfov_ * resolution_, 0);
  point_col_index_.assign(vfov_ * resolution_ * hfov_ * resolution_, 0);
  point_range_.assign(vfov_ * resolution_ * hfov_ * resolution_, 0);
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

  extractFeatureCloud(cloud_msg, cloud_mat_);
}

void PcdProcessNodelet::extractFeatureCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in, const cv::Mat& cloud_mat)
{
  extractClouds(cloud_mat);
  calculateSmoothness();
  markOccludedPoints();
  extractFeatures(surface_cloud_, corner_cloud_);

  cv::Mat surface_mat = cloudToMat(surface_cloud_, vfov_, hfov_, vfov_ * resolution_, hfov_ * resolution_);
  cv::Mat corner_mat = cloudToMat(corner_cloud_, vfov_, hfov_, vfov_ * resolution_, hfov_ * resolution_);
  output_surface_cloud_msg_ = createCloudMsg(cloud_in, surface_mat);
  output_corner_cloud_msg_ = createCloudMsg(cloud_in, corner_mat);
  surface_pointcloud_pub_.publish(output_surface_cloud_msg_);
  corner_pointcloud_pub_.publish(output_corner_cloud_msg_);

  extracted_cloud_->clear();
  surface_cloud_->clear();
  corner_cloud_->clear();
  start_ring_index_.clear();
  end_ring_index_.clear();
  point_col_index_.clear();
  point_range_.clear();
}

void PcdProcessNodelet::extractClouds(const cv::Mat& cloud_mat)
{
  int count = 0;
  for (int i = 0; i < cloud_mat.rows; ++i)
  {
    start_ring_index_[i] = count - 1 + 5;
    for (int j = 0; j < cloud_mat.cols; ++j)
    {
      cv::Vec4f point_data = cloud_mat.at<cv::Vec4f>(i, j);
      if (std::isnan(point_data[0]) || std::isnan(point_data[1]) || std::isnan(point_data[2]))
        continue;

      float range = std::hypot(point_data[0], point_data[1], point_data[2]);
      point_col_index_[count] = j;
      point_range_[count] = range;

      pcl::PointXYZI point;
      point.x = point_data[0];
      point.y = point_data[1];
      point.z = point_data[2];
      point.intensity = point_data[3];
      extracted_cloud_->push_back(point);
      ++count;
    }
    end_ring_index_[i] = count - 1 - 5;
  }
}

void PcdProcessNodelet::calculateSmoothness()
{
  int cloud_size = extracted_cloud_->points.size();
  int size_neighbor_picked = static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_);
  int size_label = static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_);
  int size_curvature = static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_);
  int size_smoothness = static_cast<int>(vfov_ * resolution_ * hfov_ * resolution_);
  for (int i = 5; i < cloud_size - 5; i++)
  {
    float diff_range = point_range_[i - 5] + point_range_[i - 4] + point_range_[i - 3] + point_range_[i - 2] +
                       point_range_[i - 1] - point_range_[i] * 10 + point_range_[i + 1] + point_range_[i + 2] +
                       point_range_[i + 3] + point_range_[i + 4] + point_range_[i + 5];

    cloud_curvature_[i] = std::pow(diff_range, 2);

    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = 0;
    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].ind = i;
  }
}

void PcdProcessNodelet::markOccludedPoints()
{
  int cloud_size = extracted_cloud_->points.size();
  for (int i = 5; i < cloud_size - 6; ++i)
  {
    float depth1 = point_range_[i];
    float depth2 = point_range_[i + 1];
    int column_diff = std::abs(int(point_col_index_[i + 1] - point_col_index_[i]));
    if (column_diff < 10)
    {
      if (depth1 - depth2 > 0.3)
      {
        cloud_neighbor_picked_[i - 5] = 1;
        cloud_neighbor_picked_[i - 4] = 1;
        cloud_neighbor_picked_[i - 3] = 1;
        cloud_neighbor_picked_[i - 2] = 1;
        cloud_neighbor_picked_[i - 1] = 1;
        cloud_neighbor_picked_[i] = 1;
      }
      else if (depth2 - depth1 > 0.3)
      {
        cloud_neighbor_picked_[i + 1] = 1;
        cloud_neighbor_picked_[i + 2] = 1;
        cloud_neighbor_picked_[i + 3] = 1;
        cloud_neighbor_picked_[i + 4] = 1;
        cloud_neighbor_picked_[i + 5] = 1;
        cloud_neighbor_picked_[i + 6] = 1;
      }
    }
    float diff1 = std::abs(float(point_range_[i - 1] - point_range_[i]));
    float diff2 = std::abs(float(point_range_[i + 1] - point_range_[i]));

    if (diff1 > 0.02 * point_range_[i] && diff2 > 0.02 * point_range_[i])
      cloud_neighbor_picked_[i] = 1;
  }
}

void PcdProcessNodelet::extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr& surface_cloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& corner_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_scan(new pcl::PointCloud<pcl::PointXYZI>());

  for (int i = 0; i < vfov_ * resolution_; i++)
  {
    surface_cloud_scan->clear();

    for (int j = 0; j < 6; j++)
    {
      int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
      int ep = (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 - 1;

      if (sp >= ep)
        continue;

      std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, by_value());

      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--)
      {
        int ind = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > corner_threshold_)
        {
          largest_picked_num++;
          if (largest_picked_num <= 20)
          {
            cloud_label_[ind] = 1;
            corner_cloud->push_back(extracted_cloud_->points[ind]);
          }
          else
            break;

          cloud_neighbor_picked_[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            int column_diff = std::abs(int(point_col_index_[ind + l] - point_col_index_[ind + l - 1]));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            int column_diff = std::abs(int(point_col_index_[ind + l] - point_col_index_[ind + l + 1]));
            if (column_diff > 10)
              break;
            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        int ind = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < surface_threshold_)
        {
          cloud_label_[ind] = -1;
          cloud_neighbor_picked_[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            int column_diff = std::abs(int(point_col_index_[ind + l] - point_col_index_[ind + l - 1]));
            if (column_diff > 10)
              break;

            cloud_neighbor_picked_[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            int column_diff = std::abs(int(point_col_index_[ind + l] - point_col_index_[ind + l + 1]));
            if (column_diff > 10)
              break;

            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        if (cloud_label_[k] <= 0)
          surface_cloud_scan->push_back(extracted_cloud_->points[k]);
      }
    }
    *surface_cloud += *surface_cloud_scan;
  }
}

sensor_msgs::ImagePtr PcdProcessNodelet::createImageMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                                        const cv::Mat& cloud_mat, int channel)
{
  cv_bridge::CvImage cv_image;

  cv_image.header = cloud_msg->header;
  cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  std::vector<cv::Mat> channels;
  cv::split(cloud_mat, channels);

  cv::Mat flipped_channel;
  cv::flip(channels[channel], flipped_channel, 1);
  cv_image.image = convertToColorMap(flipped_channel);

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
    float elevation = std::atan2(point.z, std::hypot(point.x, point.y));
    float radius = std::hypot(point.x, point.y, point.z);

    int calculated_row_idx = int(
        std::min(std::max(std::round(static_cast<float>(mat_row) * (1 - (radToDeg(elevation) + (vfov / 2.0f)) / vfov)),
                          static_cast<float>(first_row_idx)),
                 static_cast<float>(last_row_idx)));
    int calculated_col_idx =
        int(std::min(std::max(std::round(static_cast<float>(mat_col) * ((radToDeg(azimuth) + (hfov / 2.0f)) / hfov)),
                              static_cast<float>(first_col_idx)),
                     static_cast<float>(last_col_idx)));

    cv::Vec4f& mat_cell = cloud_mat.at<cv::Vec4f>(calculated_row_idx, calculated_col_idx);

    if (std::isnan(mat_cell[0]) || radius < std::hypot(mat_cell[0], mat_cell[1], mat_cell[2]))
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
