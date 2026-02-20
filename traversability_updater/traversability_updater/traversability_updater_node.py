"""
This module contains the implementation of the GridMapSubscriber node.

It subscribes to grid map messages, analyzes the traversability of the terrain,
and republishes the modified grid map with traversability information.

Classes:
    GridMapSubscriber: A ROS2 node that subscribes to grid map messages,
    processes them to analyze traversability,
                       and republishes the modified grid map.

Functions:
    main(args=None): Initializes the ROS2 system, creates a GridMapSubscriber
    node, and spins it to process messages.
"""

# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes

from grid_map_msgs.msg import GridMap as GridMapMsg

from traversability_updater.ground_analyzer import GroundAnalyzer

import numpy as np

import rclpy
from rclpy.node import Node


def get_rgb_image(submap):
    # Get 3 dims RGB image from submap
    return np.stack(((submap & 255), ((submap >> 8) & 255),
                     (submap >> 16) & 255), axis=-1).astype(np.uint8)


def map_layer_to_numpy(msg, layer_name):
    layer_index = msg.layers.index(layer_name)
    return np.array(msg.data[layer_index].data).reshape(
        msg.data[layer_index].layout.dim[0].size,
        msg.data[layer_index].layout.dim[1].size)


def map_rgb_layer_to_numpy(msg, layer_name):
    layer_index = msg.layers.index(layer_name)
    data = np.array([ctypes.c_uint32.from_buffer(
        ctypes.c_float(val)).value for val in msg.data[layer_index].data])
    return np.array(data).reshape(msg.data[layer_index].layout.dim[0].size,
                                  msg.data[layer_index].layout.dim[1].size)


class GridMapSubscriber(Node):
    """
    Traversability Analyzer Node.

    A ROS2 node that subscribes to grid map messages, processes them to
    analyze traversability, and republishes the modified grid map.
    """

    def __init__(self):
        """
        Initialize the GridMapSubscriber node.

        Sets up the node, initializes the GroundAnalyzer, and creates
        subscriptions and publishers for grid map messages.
        """
        super().__init__('nav_analyzer_node')

        self.declare_parameter('grid_map_topic', '/grid_map')
        grid_map_topic = self.get_parameter('grid_map_topic') \
            .get_parameter_value().string_value

        self.declare_parameter('subgrid_map_topic', '/subgrid_map')
        subgrid_map_topic = self.get_parameter(
            'subgrid_map_topic'
        ).get_parameter_value().string_value

        self.declare_parameter('save_folder_name', '/home/migueldm/bagfiles/')
        save_folder_name = self.get_parameter(
            'save_folder_name'
        ).get_parameter_value().string_value

        # Set to True to enable learning during Teleoperation
        self.learning = True

        # Set mode:
        # 'HC' for Hand Crafted Features
        # 'VAE' for VAE features
        # self.mode = 'HC'
        self.mode = 'VAE'

        self.save_maps = False
        self.data_folder = save_folder_name
        self.num = 0

        self.analyzer_ = GroundAnalyzer(img_mode=self.mode)

        if not self.learning:
            self.analyzer_.load_features()
            self.get_logger().info('Params Loaded')

        self.grid_map_sub = self.create_subscription(
            GridMapMsg,
            grid_map_topic,
            self.grid_map_callback,
            1
        )

        self.subgrid_map_sub = self.create_subscription(
            GridMapMsg,
            subgrid_map_topic,
            self.subgrid_map_callback,
            1
        )

        self.pub = self.create_publisher(GridMapMsg, 'grid_map_topic_out', 10)

    def set_layer_data(self, msg, layer_name, data):
        """
        Set the data for a specific layer in the message.

        Args:
            msg (MessageType): The message object containing layers and data.
            layer_name (str): The name of the layer to update.
            data (Any): The data to set for the specified layer.
        Returns:
            MessageType: The updated message object with the new data for the
            specified layer.
        """
        layer_index = msg.layers.index(layer_name)
        msg.data[layer_index].data = data
        return msg

    def subgrid_map_callback(self, msg):
        """
        Process subgrid map messages.

        This function is triggered when a new subgrid map message is received.
        Depending on the current mode, it processes the message using different
        methods of the analyzer.

        Args:
            msg: The subgrid map message to be processed.

        Modes:
            'VAE': Uses the VAE method to insert the sample.
            'HC': Uses image and elevation methods to insert the sample.

        Note:
            This function only processes messages if learning is enabled.
        """

        self.get_logger().info('Insert sample')

        img_map = map_rgb_layer_to_numpy(msg, 'RGB')
        map_elev = map_layer_to_numpy(msg, 'elevation')
        map_elev = np.expand_dims(map_elev, axis=-1)  # Add a third dimension

        img_map = get_rgb_image(img_map)

        if self.save_maps and (np.sum(img_map == 0) == 0):
            np.save(f'{self.data_folder}img_map_{self.num:04d}.npy', img_map)
            np.save(f'{self.data_folder}elev_map_{self.num:04d}.npy', map_elev)
            self.num += 1
            return

        if self.learning:
            self.analyzer_.insert_sample(img_map, map_elev)

    def grid_map_callback(self, msg):
        """
        Process grid map messages.

        Depending on the mode, this function will recompute the
        transversality of the grid map using different methods and
        publish the updated map.

        Args:
            msg (GridMap): The incoming grid map message.

        Modes:
            'VAE': Uses a VAE-based method to recompute transversality.
            'HC': Uses image and elevation-based methods to recompute
              transversality and combines the results.

        The updated transversality data is set in the 'transversality'
        layer of the grid map and published.
        """

        img_map = map_rgb_layer_to_numpy(msg, 'RGB')
        map_rgb = (get_rgb_image(img_map))

        layer_name = 'elevation'
        map_elev = map_layer_to_numpy(msg, layer_name)


        if self.save_maps:
            np.save(self.data_folder + 'full_map.npy', map_rgb)
            np.save(self.data_folder + 'full_map_elev.npy', map_elev)
            return

        self.get_logger().info('Recompute...')
        computed_map = self.analyzer_.recompute_transversality(
            map_rgb, map_elev, threshold=0.75, alpha=0.8)
        self.get_logger().info('             ...Done!')

        self.set_layer_data(msg, 'transversality',
                            np.array(computed_map).flatten().tolist())

        self.pub.publish(msg)


def main(args=None):
    """Initialize the ROS2 node."""
    rclpy.init(args=args)
    grid_map_subscriber = GridMapSubscriber()
    rclpy.spin(grid_map_subscriber)
    grid_map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
