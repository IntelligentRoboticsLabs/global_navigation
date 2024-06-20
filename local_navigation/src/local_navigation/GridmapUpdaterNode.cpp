// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <list>

#include "local_navigation/GridmapUpdaterNode.hpp"


#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/buffer.h"
#include "pcl/point_types.h"

#include "pcl_ros/transforms.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"

namespace local_navigation
{

using std::placeholders::_1;
using namespace std::chrono_literals;

GridmapUpdaterNode::GridmapUpdaterNode(const rclcpp::NodeOptions & options)
: Node("gridmap_updater_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  init_colors();
  init_gridmap();

  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_pc", 10, std::bind(&GridmapUpdaterNode::pc_callback, this, _1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "input_path", 10, std::bind(&GridmapUpdaterNode::path_callback, this, _1));
  gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
}

void
GridmapUpdaterNode::init_gridmap()
{
  // Init gridmap size and frame
  gridmap_ = std::make_shared<grid_map::GridMap>();
  gridmap_->setFrameId(map_frame_id_);
  gridmap_->setGeometry(
    grid_map::Length(size_x_, size_y_),
    resolution_gridmap_, grid_map::Position(0.0, 0.0));

  // Adding layers
  gridmap_->add("elevation");
  gridmap_->add("transversality");

  reset_gridmap();
}

void
GridmapUpdaterNode::init_colors()
{
  Eigen::Vector3i color_unknown_v(200, 200, 200);
  Eigen::Vector3i color_free_v(0, 255, 0);
  Eigen::Vector3i color_obstacle_v(255, 0, 0);

  grid_map::colorVectorToValue(color_unknown_v, color_unknown_);
  grid_map::colorVectorToValue(color_free_v, color_free_);
  grid_map::colorVectorToValue(color_obstacle_v, color_obstacle_);
}

void
GridmapUpdaterNode::reset_gridmap()
{
  // Use matrix to set the values.
  em_ = Eigen::MatrixXf(gridmap_->getSize()(0), gridmap_->getSize()(1));
  tm_ = Eigen::MatrixXf(gridmap_->getSize()(0), gridmap_->getSize()(1));

  // Set init values in matrixes
  for (auto i = 0; i < gridmap_->getSize()(0); i++) {
    for (auto j = 0; j < gridmap_->getSize()(1); j++) {
      em_(i, j) = NAN;
      tm_(i, j) = color_unknown_;
    }
  }

  // Dump matrixes to gridmap layers
  for (auto i = 0; i < gridmap_->getSize()(0); i++) {
    for (auto j = 0; j < gridmap_->getSize()(1); j++) {
      grid_map::Index map_index(i, j);
      gridmap_->at("elevation", map_index) = em_(i, j);
      gridmap_->at("transversality", map_index) = tm_(i, j);
    }
  }
}

void
GridmapUpdaterNode::update_gridmap(
  const pcl::PointCloud<pcl::PointXYZ> & pc_map,
  const pcl::PointCloud<pcl::PointXYZ> & pc_robot)
{
  for (size_t i = 0; i < pc_map.size(); i++) {
    const auto & point = pc_map[i];
    const auto & point_robot = pc_robot[i];

    if (std::isnan(point.x)) {continue;}
    if (std::isnan(point.y)) {continue;}
    if (std::isnan(point.z)) {continue;}
    if (std::isinf(point.x)) {continue;}
    if (std::isinf(point.y)) {continue;}
    if (std::isinf(point.z)) {continue;}
    if (std::isnan(point_robot.x)) {continue;}
    if (std::isnan(point_robot.y)) {continue;}
    if (std::isnan(point_robot.z)) {continue;}
    if (std::isinf(point_robot.x)) {continue;}
    if (std::isinf(point_robot.y)) {continue;}
    if (std::isinf(point_robot.z)) {continue;}

    if (point_robot.x < robot_radious_max_x_ && point_robot.x > robot_radious_min_x_ &&
      abs(point_robot.y) < robot_radious_y_)
    {
      continue;
    }
    if (abs(point_robot.x) > infl_radious_x_ || abs(point_robot.y) > infl_radious_y_) {
      continue;
    }

    if (abs(point.x) > size_x_ / 2 || abs(point.y) > size_y_ / 2) {continue;}
    if (point_robot.z > 2.0) {continue;}

    grid_map::Position position(point.x, point.y);
    grid_map::Index idx;
    gridmap_->getIndex(position, idx);

    float height = em_(idx(0), idx(1));

    if (std::isnan(height)) {
      em_(idx(0), idx(1)) = point.z;
    } else {
      em_(idx(0), idx(1)) = (em_(idx(0), idx(1)) + point.z) / 2.0;
    }

    gridmap_->at("elevation", idx) = em_(idx(0), idx(1));
  }
}

void
GridmapUpdaterNode::publish_gridmap(const builtin_interfaces::msg::Time & stamp)
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg;
  msg = grid_map::GridMapRosConverter::toMessage(*gridmap_);
  msg->header.frame_id = map_frame_id_;
  msg->header.stamp = stamp;

  gridmap_pub_->publish(std::move(msg));
}


std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>
transform_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & pc_in, std_msgs::msg::Header & header,
  const std::string & map_frame, tf2_ros::Buffer & tf_buffer)
{
  tf2::Stamped<tf2::Transform> sensor2wf;
  std::string error;

  if (tf_buffer.canTransform(
      map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp), 1s, &error))
  {
    auto sensor2wf_msg = tf_buffer.lookupTransform(
      map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp));

    tf2::fromMsg(sensor2wf_msg, sensor2wf);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(pc_in, *transformed_cloud, sensor2wf);

    return {transformed_cloud, ""};
  } else {
    return {nullptr, error};
  }
}

void
GridmapUpdaterNode::pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in)
{
  RCLCPP_INFO(get_logger(), "PointCloud received");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_in, *pcl_cloud);

  auto [cloud_map, error_map] = transform_cloud(
    *pcl_cloud, pc_in->header, map_frame_id_, tf_buffer_);
  auto [cloud_robot, error_robot] = transform_cloud(
    *pcl_cloud, pc_in->header, robot_frame_id_, tf_buffer_);


  if (cloud_map == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error_map.c_str());
    return;
  }
  if (cloud_robot == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error_robot.c_str());
    return;
  }

  // reset_gridmap();
  update_gridmap(*cloud_map, *cloud_robot);

  publish_gridmap(pc_in->header.stamp);
}

void
GridmapUpdaterNode::path_callback(nav_msgs::msg::Path::UniquePtr path_in)
{
  RCLCPP_INFO(get_logger(), "Path received");
  for (const auto & pose : path_in->poses) {
    grid_map::Position position(pose.pose.position.x, pose.pose.position.y);

    for (grid_map::CircleIterator it(*gridmap_, position, robot_radious_);
      !it.isPastEnd(); ++it)
    {
      grid_map::Position currentPositionInCircle;
      gridmap_->getPosition(*it, currentPositionInCircle);

      grid_map::Index idx;
      gridmap_->getIndex(currentPositionInCircle, idx);
      tm_(idx(0), idx(1)) = color_free_;

      gridmap_->at("transversality", idx) = tm_(idx(0), idx(1));
    }
  }
}

}  // namespace local_navigation
