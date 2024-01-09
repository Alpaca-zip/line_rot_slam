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
  prev_transform_ = Eigen::Affine3f::Identity();
  prev_optimization_transform_ = Eigen::Affine3f::Identity();
  prev_optimization_transform_.translate(Eigen::Vector3f(-7.0, 0.0, 0.0));  // TODO: get initial position from parameter
  optimization_flag_ = false;
  push_back_first_cloud_flag_ = true;
  marker_id_ = 0;

  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  last_optimized_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
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
  cloud_msg_ = cloud_msg;
  feature_cloud_msg_ = feature_cloud_msg;

  if (!optimization_flag_)
  {
    if (push_back_first_cloud_flag_)
    {
      pushBackTransformAndCloud();
      push_back_first_cloud_flag_ = false;
    }

    Eigen::Affine3f current_transform = Eigen::Affine3f::Identity();
    Eigen::Affine3f incremental_transform = Eigen::Affine3f::Identity();
    getTransform(odom_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, current_transform);

    if (shouldPushBackCloud(current_transform, incremental_transform))
    {
      pushBackTransformAndCloud(incremental_transform);
    }
  }
}

void MapOptimizationNodelet::pushBackTransformAndCloud()
{
  std::vector<int> indices;
  Eigen::Affine3f current_transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f incremental_transform = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_feature_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  getTransform(odom_frame_, cloud_msg_->header.frame_id, cloud_msg_->header.stamp, current_transform);
  incremental_transform = prev_transform_.inverse() * current_transform;
  current_transform = prev_optimization_transform_ * incremental_transform;
  prev_transform_ = current_transform;

  pcl::fromROSMsg(*cloud_msg_, *map_cloud);
  pcl::fromROSMsg(*feature_cloud_msg_, *feature_cloud);
  pcl::removeNaNFromPointCloud(*map_cloud, *filtered_map_cloud, indices);
  pcl::removeNaNFromPointCloud(*feature_cloud, *filtered_feature_cloud, indices);
  map_cloud_vector_.push_back(filtered_map_cloud);
  feature_cloud_vector_.push_back(filtered_feature_cloud);
  transform_vector_.push_back(current_transform);
}

void MapOptimizationNodelet::pushBackTransformAndCloud(const Eigen::Affine3f& incremental_transform)
{
  std::vector<int> indices;
  Eigen::Affine3f current_transform = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_feature_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  current_transform = prev_optimization_transform_ * incremental_transform;
  prev_transform_ = current_transform;

  pcl::fromROSMsg(*cloud_msg_, *map_cloud);
  pcl::fromROSMsg(*feature_cloud_msg_, *feature_cloud);
  pcl::removeNaNFromPointCloud(*map_cloud, *filtered_map_cloud, indices);
  pcl::removeNaNFromPointCloud(*feature_cloud, *filtered_feature_cloud, indices);
  map_cloud_vector_.push_back(filtered_map_cloud);
  feature_cloud_vector_.push_back(filtered_feature_cloud);
  transform_vector_.push_back(current_transform);
}

void MapOptimizationNodelet::execute(const line_rot_slam::OptimizationGoalConstPtr& goal)
{
  line_rot_slam::OptimizationResult result;

  optimization_flag_ = true;

  while (!isRobotStopped())
  {
    ros::Duration(0.1).sleep();
  }

  if (goal->state == "MOVE")
  {
    moveOptimization();
    publishMarkerArray();
  }
  else if (goal->state == "TURN")
  {
    turnOptimization();
  }

  transformCloud();

  publishMapCloud();

  result.success = true;
  server_->setSucceeded(result);

  prev_optimization_transform_ = transform_vector_.back();

  transformed_cloud_vector_.clear();
  map_cloud_vector_.clear();
  feature_cloud_vector_.clear();
  transform_vector_.clear();

  optimization_flag_ = false;
  push_back_first_cloud_flag_ = true;
}

void MapOptimizationNodelet::moveOptimization()
{
  if (last_optimized_cloud_->empty())
  {
    *last_optimized_cloud_ = *feature_cloud_vector_[0];
  }

  Eigen::Affine3f cumulative_transform = prev_optimization_transform_;

  for (size_t i = 0; i < feature_cloud_vector_.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*last_optimized_cloud_, *transformed_cloud, cumulative_transform.inverse());

    Eigen::Affine3f transform;
    calculateTransMat(transformed_cloud, feature_cloud_vector_[i], transform);

    cumulative_transform = cumulative_transform * transform;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (i == map_cloud_vector_.size() - 1)
    {
      pcl::transformPointCloud(*map_cloud_vector_[i], *optimized_cloud, cumulative_transform);
    }
    else
    {
      pcl::transformPointCloud(*feature_cloud_vector_[i], *optimized_cloud, cumulative_transform);
    }

    *last_optimized_cloud_ = *optimized_cloud;
    transform_vector_[i] = cumulative_transform.matrix();
  }
}

void MapOptimizationNodelet::turnOptimization()
{
  if (last_optimized_cloud_->empty())
  {
    *last_optimized_cloud_ = *map_cloud_vector_[0];
  }

  last_angle_z_ = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3f inverse_transform = Eigen::Affine3f::Identity();
  inverse_transform.translate(-prev_optimization_transform_.translation());
  pcl::transformPointCloud(*last_optimized_cloud_, *transformed_cloud, inverse_transform);

  for (size_t i = 0; i < map_cloud_vector_.size(); i++)
  {
    Eigen::Matrix3f rotation_matrix;
    calculateRotMat(transformed_cloud, map_cloud_vector_[i], rotation_matrix);

    Eigen::Affine3f combined_transform = Eigen::Affine3f::Identity();
    combined_transform.translate(prev_optimization_transform_.translation());
    combined_transform.rotate(rotation_matrix);

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (i == map_cloud_vector_.size() - 1)
    {
      pcl::transformPointCloud(*feature_cloud_vector_[i], *optimized_cloud, combined_transform);
    }
    else
    {
      pcl::transformPointCloud(*map_cloud_vector_[i], *optimized_cloud, combined_transform);
    }

    *last_optimized_cloud_ = *optimized_cloud;
    transform_vector_[i] = combined_transform.matrix();
  }
}

void MapOptimizationNodelet::calculateTransMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                                               const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
                                               Eigen::Affine3f& transform_matrix)
{
  std::vector<double> direction_error_vector;
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
        Eigen::Vector3f current_position(current_point.x, current_point.y, current_point.z);
        Eigen::Vector3f center_position(center_x, center_y, center_z);
        Eigen::Vector3f position_difference = current_position - center_position;
        double direction_error = position_difference.dot(Eigen::Vector3f(1, 0, 0));
        direction_error_vector.push_back(direction_error);
      }
    }
  }

  double median_error = calculateMedianError(direction_error_vector);

  transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translate(Eigen::Vector3f(-median_error, 0, 0));
}

void MapOptimizationNodelet::calculateRotMat(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,
                                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
                                             Eigen::Matrix3f& rotation_matrix)
{
  Eigen::MatrixXf map_matrix(3, map_cloud->size());
  Eigen::MatrixXf current_matrix(3, current_cloud->size());

  for (size_t i = 0; i < map_cloud->size(); ++i)
  {
    map_matrix.col(i) = Eigen::Vector3f(map_cloud->points[i].x, map_cloud->points[i].y, map_cloud->points[i].z);
  }
  for (size_t i = 0; i < current_cloud->size(); ++i)
  {
    current_matrix.col(i) =
        Eigen::Vector3f(current_cloud->points[i].x, current_cloud->points[i].y, current_cloud->points[i].z);
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_map(map_matrix, Eigen::ComputeFullU);
  Eigen::MatrixXf u_map = svd_map.matrixU();

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_current(current_matrix, Eigen::ComputeFullU);
  Eigen::MatrixXf u_current = svd_current.matrixU();

  for (int i = 0; i < u_current.cols(); ++i)
  {
    if (u_current.col(i).dot(u_map.col(i)) < 0)
    {
      u_current.col(i) *= -1;
    }
  }

  rotation_matrix = u_map * u_current.transpose();
  float angle_z = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

  if (angle_z * last_angle_z_ < 0 && std::abs(angle_z) > 1.0)
  {
    if (angle_z > 0)
    {
      angle_z = angle_z - M_PI;
    }
    else
    {
      angle_z = angle_z + M_PI;
    }
  }

  last_angle_z_ = angle_z;
  rotation_matrix = Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ()).toRotationMatrix();
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
    Eigen::Vector3f translation = transform_vector_.back().translation();
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

void MapOptimizationNodelet::transformCloud()
{
  for (int i = 0; i < map_cloud_vector_.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = map_cloud_vector_[i];
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_vector_[i]);
    transformed_cloud_vector_.push_back(transformed_cloud);
  }
}

void MapOptimizationNodelet::publishMapCloud()
{
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud : transformed_cloud_vector_)
  {
    *map_cloud_ += *cloud;
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
                                          const ros::Time& time, Eigen::Affine3f& transform)
{
  try
  {
    geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time);
    Eigen::Affine3d transform_d = tf2::transformToEigen(transform_stamped);
    transform = transform_d.cast<float>();
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_DEBUG("Transform error: %s", ex.what());
    return false;
  }
}

bool MapOptimizationNodelet::shouldPushBackCloud(const Eigen::Affine3f& current_transform,
                                                 Eigen::Affine3f& incremental_transform)
{
  incremental_transform = prev_transform_.inverse() * current_transform;
  Eigen::Vector3f euler_angle = incremental_transform.rotation().eulerAngles(0, 1, 2);
  double rotation_angle = std::abs(euler_angle[2]);
  double translation_distance = incremental_transform.translation().norm();

  return translation_distance >= translation_threshold_ || rotation_angle >= rotation_threshold_;
}

bool MapOptimizationNodelet::isRobotStopped()
{
  Eigen::Affine3f current_transform = Eigen::Affine3f::Identity();
  getTransform(odom_frame_, cloud_msg_->header.frame_id, cloud_msg_->header.stamp, current_transform);
  Eigen::Affine3f incremental_transform = prev_transform_.inverse() * current_transform;
  Eigen::Vector3f euler_angle = incremental_transform.rotation().eulerAngles(0, 1, 2);
  double rotation_angle = std::abs(euler_angle[2]);
  double translation_distance = incremental_transform.translation().norm();

  if (translation_distance >= 0.01 || rotation_angle >= 0.01)
  {
    prev_transform_ = current_transform;
    return false;
  }
  else
  {
    pushBackTransformAndCloud(incremental_transform);
    return true;
  }
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
