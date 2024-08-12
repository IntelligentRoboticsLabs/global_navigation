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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "tf2_msgs/msg/tf_message.hpp"

#include "rclcpp/rclcpp.hpp"


class ZedTFFixer : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ZedTFFixer)

  explicit ZedTFFixer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("zed_tf_fixer", options)
  {
    tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local());
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).reliable());

    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/rosbag/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [&] (tf2_msgs::msg::TFMessage::UniquePtr msg) {
        // std::cerr << "-" << std::endl;
        tf2_msgs::msg::TFMessage fixed_msg;

        for (const auto & tf : msg->transforms) {
          if (tf.header.frame_id != "base_link" && tf.child_frame_id != "base_link") {
            fixed_msg.transforms.push_back(tf);
          } else {
            RCLCPP_INFO_STREAM(
              get_logger(),
              "Filtering a tf: " << tf.header.frame_id << " -> " << tf.child_frame_id);
          }
        }

        geometry_msgs::msg::TransformStamped fix_tf;
        fix_tf.header.stamp = msg->transforms[0].header.stamp;
        fix_tf.header.frame_id = "robot_front_camera_base_link";
        fix_tf.header.stamp = now();
        fix_tf.child_frame_id = "zed2_camera_center";

        fixed_msg.transforms.push_back(fix_tf);
        tf_static_pub_->publish(fixed_msg);
      });

    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/rosbag/tf", rclcpp::QoS(100).reliable(),
      [&] (tf2_msgs::msg::TFMessage::UniquePtr msg) {
        // std::cerr << "*" << std::endl;
        tf2_msgs::msg::TFMessage fixed_msg;

        for (const auto & tf : msg->transforms) {
          if (tf.header.frame_id != "base_link" && tf.child_frame_id != "base_link") {
            fixed_msg.transforms.push_back(tf);
          } else {
            RCLCPP_INFO_STREAM(
              get_logger(),
              "Filtering a tf: " << tf.header.frame_id << " -> " << tf.child_frame_id);
          }
        }

        geometry_msgs::msg::TransformStamped fix_tf;
        fix_tf.header.stamp = msg->transforms[0].header.stamp;
        fix_tf.header.frame_id = "robot_front_camera_base_link";
        fix_tf.header.stamp = now();
        fix_tf.child_frame_id = "zed2_camera_center";

        fixed_msg.transforms.push_back(fix_tf);
        tf_pub_->publish(fixed_msg);
      });
  }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto zed_tf_fixer_node = ZedTFFixer::make_shared();

  rclcpp::spin(zed_tf_fixer_node);

  rclcpp::shutdown();
  return 0;
}
