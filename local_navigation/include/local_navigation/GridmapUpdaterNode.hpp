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

#ifndef LOCAL_NAVIGATION__GRIDMAPUPDATERNODE_HPP_
#define LOCAL_NAVIGATION__GRIDMAPUPDATERNODE_HPP_

#include <string>
#include <memory>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"

namespace local_navigation
{

class GridmapUpdaterNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GridmapUpdaterNode)

  explicit GridmapUpdaterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void init_gridmap();
  void reset_gridmap();
  void update_gridmap(
    const pcl::PointCloud<pcl::PointXYZ> & pc_map,
    const pcl::PointCloud<pcl::PointXYZ> & pc_robot);
  void publish_gridmap(const builtin_interfaces::msg::Time & stamp);

  void init_colors();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;

  void pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in);
  void path_callback(nav_msgs::msg::Path::UniquePtr path_in);

  std::string map_frame_id_ {"map"};
  std::string robot_frame_id_ {"robot_base_link"};
  double resolution_gridmap_ {0.2};
  double size_x_ {100.0};
  double size_y_ {100.0};
  double infl_radious_x_ {10.0};
  double infl_radious_y_ {10.0};
  double robot_radious_min_x_ {-4.0};
  double robot_radious_max_x_ {1.0};
  double robot_radious_y_ {1.0};
  double robot_radious_ {0.5};

  std::shared_ptr<grid_map::GridMap> gridmap_;
  Eigen::MatrixXf em_;
  Eigen::MatrixXf tm_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float color_unknown_;
  float color_free_;
  float color_obstacle_;
};

}  // namespace local_navigation

#endif  // LOCAL_NAVIGATION__GRIDMAPUPDATERNODE_HPP_