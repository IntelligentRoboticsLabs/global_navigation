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
  get_params();

  init_colors();
  init_gridmap();

  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, 10, std::bind(&GridmapUpdaterNode::pc_callback, this, _1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, std::bind(&GridmapUpdaterNode::path_callback, this, _1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 10, std::bind(&GridmapUpdaterNode::pose_callback, this, _1));
  subscription_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 1,
      std::bind(&GridmapUpdaterNode::topic_callback_info, this, _1));
  subscription_img_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 1,
      std::bind(&GridmapUpdaterNode::image_callback, this, _1));
  gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  subgridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("subgrid_map", 10);
  img_pub_ = create_publisher<sensor_msgs::msg::Image>("img_proy_debug", 10);
}

void
GridmapUpdaterNode::get_params()
{
  this->declare_parameter("camera_info_topic", "/camera/color/camera_info");
  this->declare_parameter("camera_topic", "/camera/color/image_raw");
  this->declare_parameter("lidar_topic", "/lidar_points");
  this->declare_parameter("path_topic", "/path");
  this->declare_parameter("pose_topic", "/current_pose");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("camera_frame", "camera_color_optical_frame");

  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  camera_topic_ = this->get_parameter("camera_topic").as_string();
  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  path_topic_ = this->get_parameter("path_topic").as_string();
  pose_topic_ = this->get_parameter("pose_topic").as_string();
  map_frame_id_ = this->get_parameter("map_frame").as_string();
  robot_frame_id_ = this->get_parameter("robot_frame").as_string();
  camera_frame_id_ = this->get_parameter("camera_frame").as_string();
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
  gridmap_->add("RGB");
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
  cm_ = Eigen::MatrixXf(gridmap_->getSize()(0), gridmap_->getSize()(1));

  // Set init values in matrixes
  for (auto i = 0; i < gridmap_->getSize()(0); i++) {
      for (auto j = 0; j < gridmap_->getSize()(1); j++) {
        em_(i, j) = NAN;
        tm_(i, j) = color_unknown_;
        cm_(i, j) = 0;
      }
  }

  // Dump matrixes to gridmap layers
  for (auto i = 0; i < gridmap_->getSize()(0); i++) {
      for (auto j = 0; j < gridmap_->getSize()(1); j++) {
        grid_map::Index map_index(i, j);
        gridmap_->at("elevation", map_index) = em_(i, j);
        gridmap_->at("transversality", map_index) = tm_(i, j);
        gridmap_->at("RGB", map_index) = cm_(i, j);
      }
  }
}

std::tuple<float, int, int>
get_point_color(
  const pcl::PointXYZ & point, const image_geometry::PinholeCameraModel & camera_model,
  const cv::Mat & image_rgb_raw)
{
  cv::Mat world_point_fromCamera = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
  cv::Point2d point_2d = camera_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));

  double point_x = point_2d.x;
  double point_y = point_2d.y;

  if (point_x > 0 && point_x < image_rgb_raw.cols && point_y > 0 && point_y < image_rgb_raw.rows) {
    if (image_rgb_raw.type() == CV_8UC3) {
      cv::Vec3b color = image_rgb_raw.at<cv::Vec3b>(static_cast<int>(point_y),
        static_cast<int>(point_x));
      Eigen::Vector3i color_eigen(color[0], color[1], color[2]);
      float color_value;
      grid_map::colorVectorToValue(color_eigen, color_value);
      return {color_value, point_x, point_y};
    } else if (image_rgb_raw.type() == CV_8UC1) {
      float color_value = image_rgb_raw.at<uint8_t>(static_cast<int>(point_y),
        static_cast<int>(point_x));
      return {color_value, point_x, point_y};
    } else if (image_rgb_raw.type() == CV_8UC4) {
      cv::Vec4b color = image_rgb_raw.at<cv::Vec4b>(static_cast<int>(point_y),
        static_cast<int>(point_x));
      Eigen::Vector3i color_eigen(color[2], color[1], color[0]);
      float color_value;
      grid_map::colorVectorToValue(color_eigen, color_value);
      return {color_value, point_x, point_y};
    } else {
      return {-1.0, -1.0, -1.0};
    }
  } else {
    return {-1.0, -1.0, -1.0};
  }
}

void
GridmapUpdaterNode::update_gridmap(
  const pcl::PointCloud<pcl::PointXYZ> & pc_map,
  const pcl::PointCloud<pcl::PointXYZ> & pc_robot,
  const pcl::PointCloud<pcl::PointXYZ> & pc_camera)
{
  // RCLCPP_INFO(get_logger(), "Updating gridmap");
  for (size_t i = 0; i < pc_map.size(); i++) {
    const auto & point = pc_map[i];
    const auto & point_robot = pc_robot[i];
    const auto & point_camera = pc_camera[i];

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

    if (std::isnan(point_camera.x)) {continue;}
    if (std::isnan(point_camera.y)) {continue;}
    if (std::isnan(point_camera.z)) {continue;}
    if (std::isinf(point_camera.x)) {continue;}
    if (std::isinf(point_camera.y)) {continue;}
    if (std::isinf(point_camera.z)) {continue;}

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

    if (camera_model_ != nullptr && !image_rgb_raw_.empty() &&
      point_camera.z > 0)  // Prevent to proyect points behind the camera
    {
      auto [color, p_x, p_y] = get_point_color(point_camera, *camera_model_, image_rgb_raw_);
      if (color > 0) {
        cm_(idx(0), idx(1)) = color;
        gridmap_->at("RGB", idx) = color;
      }
    } else {
      gridmap_->at("RGB", idx) = cm_(idx(0), idx(1));
    }
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
GridmapUpdaterNode::pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr pose)
{
  grid_map::GridMap submap;

  grid_map::Position position(pose->pose.position.x + subgridmap_size_ / 2 * resolution_gridmap_,
    pose->pose.position.y + subgridmap_size_ * resolution_gridmap_ / 2);
  grid_map::Index submapStartIndex;
  gridmap_->getIndex(position, submapStartIndex);
  grid_map::Index submapBufferSize(subgridmap_size_, subgridmap_size_);

  submap.setFrameId(robot_frame_id_);
  submap.setGeometry(grid_map::Length(subgridmap_size_ * resolution_gridmap_,
    subgridmap_size_ * resolution_gridmap_), resolution_gridmap_);
  submap.add("elevation");
  submap.add("RGB");

  grid_map::Matrix & data_rgb = submap["RGB"];
  grid_map::Matrix & data_elevation = submap["elevation"];


  grid_map::GridMapIterator iterator(submap);
  for (grid_map::SubmapIterator submap_iterator(*gridmap_, submapStartIndex, submapBufferSize);
    !submap_iterator.isPastEnd() && !iterator.isPastEnd(); ++submap_iterator, ++iterator)
  {
    grid_map::Position currentPositionInSubmap;
    gridmap_->getPosition(*submap_iterator, currentPositionInSubmap);

    grid_map::Index idx;
    gridmap_->getIndex(currentPositionInSubmap, idx);
    const int i = iterator.getLinearIndex();
    data_rgb(i) = gridmap_->at("RGB", idx);
    data_elevation(i) = gridmap_->at("elevation", idx);
  }

  std::unique_ptr<grid_map_msgs::msg::GridMap> msg;
  msg = grid_map::GridMapRosConverter::toMessage(submap);
  msg->header.frame_id = robot_frame_id_;
  msg->header.stamp = pose->header.stamp;

  subgridmap_pub_->publish(std::move(msg));
}


void
GridmapUpdaterNode::pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_in, *pcl_cloud);

  auto [cloud_map, error_map] = transform_cloud(
    *pcl_cloud, pc_in->header, map_frame_id_, tf_buffer_);
  auto [cloud_robot, error_robot] = transform_cloud(
    *pcl_cloud, pc_in->header, robot_frame_id_, tf_buffer_);
  auto [cloud_camera, error_camera] = transform_cloud(
    *pcl_cloud, pc_in->header, camera_frame_id_, tf_buffer_);


  if (cloud_map == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error_map.c_str());
    return;
  }
  if (cloud_robot == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error_robot.c_str());
    return;
  }
  if (cloud_camera == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error_camera.c_str());
    return;
  }

  // reset_gridmap();
  update_gridmap(*cloud_map, *cloud_robot, *cloud_camera);

  publish_gridmap(pc_in->header.stamp);
}

void
GridmapUpdaterNode::path_callback(nav_msgs::msg::Path::UniquePtr path_in)
{
  // RCLCPP_INFO(get_logger(), "Path received");
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

void
GridmapUpdaterNode::topic_callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  camera_model_->fromCameraInfo(*msg);

  subscription_info_ = nullptr;
}

void
GridmapUpdaterNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  cv_bridge::CvImagePtr image_rgb_ptr;

  try {
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      image_rgb_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      image_rgb_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGRA8) {
      image_rgb_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGRA8);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported encoding %s", msg->encoding.c_str());
      return;
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  image_rgb_raw_ = image_rgb_ptr->image;
}

}  // namespace local_navigation
