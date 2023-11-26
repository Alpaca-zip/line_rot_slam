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
  pnh_.param<std::string>("input_feature_points_topic", feature_points_in_, "feature_points_in");
  pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<double>("translation_threshold", translation_threshold_, 0.1);
  pnh_.param<double>("rotation_threshold", rotation_threshold_, 0.1);
  pnh_.param<double>("voxel_grid_leaf_size", leaf_size_, 0.1);

  cloud_sub_.subscribe(nh_, points_in_, 1);
  feature_cloud_sub_.subscribe(nh_, feature_points_in_, 1);
  sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy>>(1);
  sync_->connectInput(cloud_sub_, feature_cloud_sub_);
  sync_->registerCallback(boost::bind(&MapOptimizationNodelet::pointcloudCallback, this, _1, _2));
  timer_ = nh_.createTimer(ros::Duration(0.1), &MapOptimizationNodelet::broadcastMapFrame, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("optimization_marker", 1);
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
  prev_transform_ = Eigen::Affine3d::Identity();
  prev_optimization_transform_ = Eigen::Affine3d::Identity();
  optimization_flag_ = false;
  marker_id_ = 0;

  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  optimized_cloud_container_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_feature_cloud_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster);
  server_.reset(new Server(nh_, "optimization", boost::bind(&MapOptimizationNodelet::execute, this, _1), false));
  server_->start();
}

void MapOptimizationNodelet::broadcastMapFrame(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  // TODO: get transform from optimization result

  transform_stamped.transform.translation.x = 0;
  transform_stamped.transform.translation.y = 0;
  transform_stamped.transform.translation.z = 0;
  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;

  tf_broadcaster_->sendTransform(transform_stamped);
}

void MapOptimizationNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                                const sensor_msgs::PointCloud2ConstPtr& feature_cloud_msg)
{
  if (!optimization_flag_)
  {
    Eigen::Affine3d current_transform;
    Eigen::Affine3d incremental_transform;

    if (getTransform(odom_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, current_transform))
    {
      if (shouldPushBackCloud(current_transform, incremental_transform))
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *map_cloud);
        pcl::fromROSMsg(*feature_cloud_msg, *feature_cloud);
        map_cloud_vector_.push_back(map_cloud);
        feature_cloud_vector_.push_back(feature_cloud);

        prev_optimization_transform_ = prev_optimization_transform_ * incremental_transform;
        transform_vector_.push_back(prev_optimization_transform_);

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

  if (goal->state == "MOVE")
  {
    moveOptimization();
    publishMarkerArray();
  }
  else if (goal->state == "TURN")
  {
    turnOptimization();
  }

  publishMapCloud();

  result.success = true;
  server_->setSucceeded(result);

  prev_optimization_transform_ = transform_vector_.back();

  map_cloud_vector_.clear();
  feature_cloud_vector_.clear();
  transform_vector_.clear();

  optimization_flag_ = false;
}

void MapOptimizationNodelet::moveOptimization()
{
  Eigen::Vector3d direction =
      (transform_vector_.back().translation() - transform_vector_.front().translation()).normalized();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  moveAndAppendCloud(feature_cloud_vector_[0], transform_vector_[0].translation(), cloud);

  if (optimized_cloud_container_->empty())
  {
    optimized_cloud_container_ = cloud;
  }

  for (int i = 1; i < feature_cloud_vector_.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<double> direction_errors;

    moveAndAppendCloud(feature_cloud_vector_[i], transform_vector_[i - 1].translation(), transformed_cloud);

    moveOptimizationCore(optimized_cloud_container_, transformed_cloud, direction, direction_errors);

    double median_error = calculateMedianError(direction_errors);

    moveOptimizeAndAppendCloud(transformed_cloud, direction, median_error);

    transform_vector_[i].translation() = transform_vector_[i - 1].translation() - median_error * direction;
  }
}

void MapOptimizationNodelet::turnOptimization()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  turnAndAppendCloud(feature_cloud_vector_[0], transform_vector_[0].linear(), cloud);

  if (optimized_cloud_container_->empty())
  {
    optimized_cloud_container_ = cloud;
  }

  for (int i = 1; i < feature_cloud_vector_.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<double> direction_errors;

    turnAndAppendCloud(feature_cloud_vector_[i], transform_vector_[i - 1].linear(), transformed_cloud);

    turnOptimizationCore(optimized_cloud_container_, transformed_cloud, direction_errors);

    double median_error = calculateMedianError(direction_errors);

    turnOptimizeAndAppendCloud(transformed_cloud, median_error);

    transform_vector_[i].linear() =
        transform_vector_[i - 1].linear() * Eigen::AngleAxisd(median_error, Eigen::Vector3d::UnitZ());
  }
}

void MapOptimizationNodelet::moveAndAppendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                                const Eigen::Vector3d& transform_offset,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud)
{
  for (const auto& point : input_cloud->points)
  {
    if (!pcl::isFinite(point))
      continue;

    pcl::PointXYZI transformed_point = point;
    transformed_point.x += transform_offset(0);
    transformed_point.y += transform_offset(1);
    transformed_point.z += transform_offset(2);
    output_cloud->points.push_back(transformed_point);
  }
}

void MapOptimizationNodelet::turnAndAppendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                                const Eigen::Matrix3d& rotation_matrix,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud)
{
  for (const auto& point : input_cloud->points)
  {
    if (!pcl::isFinite(point))
      continue;

    Eigen::Vector3d transformed_point = rotation_matrix * Eigen::Vector3d(point.x, point.y, point.z);
    pcl::PointXYZI new_point;
    new_point.x = transformed_point(0);
    new_point.y = transformed_point(1);
    new_point.z = transformed_point(2);
    output_cloud->points.push_back(new_point);
  }
}

void MapOptimizationNodelet::moveOptimizeAndAppendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                                        const Eigen::Vector3d& direction, const double& error)
{
  for (const auto& point : input_cloud->points)
  {
    pcl::PointXYZI optimized_point = point;
    optimized_point.x -= error * direction(0);
    optimized_point.y -= error * direction(1);
    optimized_point.z -= error * direction(2);
    optimized_cloud_container_->points.push_back(optimized_point);
  }

  while (optimized_cloud_container_->points.size() > 10000)
  {
    optimized_cloud_container_->points.erase(optimized_cloud_container_->points.begin());
  }
}

void MapOptimizationNodelet::turnOptimizeAndAppendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                                        const double& error)
{
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << cos(error), -sin(error), 0, sin(error), cos(error), 0, 0, 0, 1;

  for (const auto& point : input_cloud->points)
  {
    Eigen::Vector3d point_vector(point.x, point.y, point.z);
    Eigen::Vector3d rotate_vector = rotation_matrix * point_vector;
    pcl::PointXYZI optimized_point;
    optimized_point.x = rotate_vector(0);
    optimized_point.y = rotate_vector(1);
    optimized_point.z = rotate_vector(2);
    optimized_cloud_container_->points.push_back(optimized_point);
  }

  while (optimized_cloud_container_->points.size() > 10000)
  {
    optimized_cloud_container_->points.erase(optimized_cloud_container_->points.begin());
  }
}

void MapOptimizationNodelet::moveOptimizationCore(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
                                                  const Eigen::Vector3d& optimize_direction,
                                                  std::vector<double>& direction_error_vector)
{
  kdtree_feature_cloud_->setInputCloud(map_cloud);

  for (int j = 0; j < current_cloud->size(); j++)
  {
    std::vector<int> point_search_index;
    std::vector<float> point_search_distance;

    pcl::PointXYZI current_point = current_cloud->points[j];
    kdtree_feature_cloud_->nearestKSearch(current_point, 5, point_search_index, point_search_distance);

    cv::Mat covariance_mat(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat eigen_value(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat eigen_vector(3, 3, CV_32F, cv::Scalar::all(0));

    if (point_search_distance[4] < 1.0)
    {
      float center_x = 0, center_y = 0, center_z = 0;
      for (int k = 0; k < 5; k++)
      {
        center_x += map_cloud->points[point_search_index[k]].x;
        center_y += map_cloud->points[point_search_index[k]].y;
        center_z += map_cloud->points[point_search_index[k]].z;
      }
      center_x /= 5;
      center_y /= 5;
      center_z /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int k = 0; k < 5; k++)
      {
        float ax = map_cloud->points[point_search_index[k]].x - center_x;
        float ay = map_cloud->points[point_search_index[k]].y - center_y;
        float az = map_cloud->points[point_search_index[k]].z - center_z;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      covariance_mat.at<float>(0, 0) = a11;
      covariance_mat.at<float>(0, 1) = a12;
      covariance_mat.at<float>(0, 2) = a13;
      covariance_mat.at<float>(1, 0) = a12;
      covariance_mat.at<float>(1, 1) = a22;
      covariance_mat.at<float>(1, 2) = a23;
      covariance_mat.at<float>(2, 0) = a13;
      covariance_mat.at<float>(2, 1) = a23;
      covariance_mat.at<float>(2, 2) = a33;

      cv::eigen(covariance_mat, eigen_value, eigen_vector);

      if (eigen_value.at<float>(0, 0) > 3 * eigen_value.at<float>(0, 1))
      {
        Eigen::Vector3d current_position(current_point.x, current_point.y, current_point.z);
        Eigen::Vector3d center_position(center_x, center_y, center_z);
        Eigen::Vector3d position_difference = current_position - center_position;
        double direction_error = position_difference.dot(optimize_direction.normalized());
        direction_error_vector.push_back(direction_error);
      }
    }
  }
}

void MapOptimizationNodelet::turnOptimizationCore(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
                                                  std::vector<double>& direction_error_vector)
{
  kdtree_feature_cloud_->setInputCloud(map_cloud);

  for (int j = 0; j < current_cloud->size(); j++)
  {
    std::vector<int> point_search_index;
    std::vector<float> point_search_distance;

    pcl::PointXYZI current_point = current_cloud->points[j];
    kdtree_feature_cloud_->nearestKSearch(current_point, 5, point_search_index, point_search_distance);

    cv::Mat covariance_mat(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat eigen_value(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat eigen_vector(3, 3, CV_32F, cv::Scalar::all(0));

    if (point_search_distance[4] < 1.0)
    {
      float center_x = 0, center_y = 0, center_z = 0;
      for (int k = 0; k < 5; k++)
      {
        center_x += map_cloud->points[point_search_index[k]].x;
        center_y += map_cloud->points[point_search_index[k]].y;
        center_z += map_cloud->points[point_search_index[k]].z;
      }
      center_x /= 5;
      center_y /= 5;
      center_z /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int k = 0; k < 5; k++)
      {
        float ax = map_cloud->points[point_search_index[k]].x - center_x;
        float ay = map_cloud->points[point_search_index[k]].y - center_y;
        float az = map_cloud->points[point_search_index[k]].z - center_z;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      covariance_mat.at<float>(0, 0) = a11;
      covariance_mat.at<float>(0, 1) = a12;
      covariance_mat.at<float>(0, 2) = a13;
      covariance_mat.at<float>(1, 0) = a12;
      covariance_mat.at<float>(1, 1) = a22;
      covariance_mat.at<float>(1, 2) = a23;
      covariance_mat.at<float>(2, 0) = a13;
      covariance_mat.at<float>(2, 1) = a23;
      covariance_mat.at<float>(2, 2) = a33;

      cv::eigen(covariance_mat, eigen_value, eigen_vector);

      if (eigen_value.at<float>(0, 0) > 3 * eigen_value.at<float>(0, 1))
      {
        Eigen::Vector3d current_position(current_point.x, current_point.y, current_point.z);
        Eigen::Vector3d center_position(center_x, center_y, center_z);
        Eigen::Vector3d cross_product = current_position.cross(center_position);
        double dot_product = current_position.dot(center_position);
        double current_norm = current_position.norm();
        double center_norm = center_position.norm();
        double cos_angle = dot_product / (current_norm * center_norm);
        double angle = std::acos(cos_angle);

        if (cross_product.z() < 0)
          angle = -angle;

        direction_error_vector.push_back(angle);
      }
    }
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
  for (size_t i = 0; i < map_cloud_vector_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*map_cloud_vector_[i], *transformed_cloud, transform_vector_[i]);

    *map_cloud_ += *transformed_cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(map_cloud_);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid.filter(*downsampled_cloud);

  sensor_msgs::PointCloud2Ptr map_cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_cloud_, *map_cloud_msg);
  map_cloud_msg->header.frame_id = map_frame_;
  map_cloud_msg->header.stamp = ros::Time::now();

  map_pub_.publish(map_cloud_msg);
}

bool MapOptimizationNodelet::getTransform(const std::string& target_frame, const std::string& source_frame,
                                          const ros::Time& time, Eigen::Affine3d& transform)
{
  try
  {
    geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time);
    transform = tf2::transformToEigen(transform_stamped);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_DEBUG("Transform error: %s", ex.what());
    return false;
  }
}

bool MapOptimizationNodelet::shouldPushBackCloud(const Eigen::Affine3d& current_transform,
                                                 Eigen::Affine3d& incremental_transform)
{
  incremental_transform = prev_transform_.inverse() * current_transform;
  Eigen::Vector3d euler_angle = incremental_transform.rotation().eulerAngles(0, 1, 2);
  double rotation_angle = std::abs(euler_angle[2]);
  double translation_distance = incremental_transform.translation().norm();

  return translation_distance >= translation_threshold_ || rotation_angle >= rotation_threshold_;
}

double MapOptimizationNodelet::calculateMedianError(std::vector<double>& direction_error_vector)
{
  if (!direction_error_vector.empty())
  {
    std::sort(direction_error_vector.begin(), direction_error_vector.end());
    return direction_error_vector[direction_error_vector.size() / 2];
  }
  else
    return 0;
}

}  // namespace OPTIMIZATION

PLUGINLIB_EXPORT_CLASS(OPTIMIZATION::MapOptimizationNodelet, nodelet::Nodelet)
