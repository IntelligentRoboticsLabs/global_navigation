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

from grid_map_msgs.msg import GridMap as GridMapMsg

from ground_analyzer import GroundAnalyzer

import numpy as np

import rclpy
from rclpy.node import Node


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

        # Set to True to enable learning during Teleoperation
        self.learning = True

        # Set mode:
        # 'HC' for Hand Crafted Features
        # 'VAE' for VAE features
        self.mode = 'HC'
        # self.mode = 'VAE'

        self.analyzer_ = GroundAnalyzer(img_mode=self.mode)

        if not self.learning:
            self.analyzer_.load_features()
            self.get_logger().info('Params Loaded')

        self.grid_map_sub = self.create_subscription(
            GridMapMsg,
            '/grid_map',
            self.grid_map_callback,
            1
        )

        self.subgrid_map_sub = self.create_subscription(
            GridMapMsg,
            '/subgrid_map',
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

    def combine_transversality(self, mask_1, mask_2, alpha=0.5):
        """
        Combine two transversality masks using a weighted average.

        Args:
            mask_1 (numpy.ndarray): The first transversality mask.
            mask_2 (numpy.ndarray): The second transversality mask.
            alpha (float, optional): The weight for the first mask.
        Returns:
            numpy.ndarray: The combined transversality mask.
        """
        nav_map_fusion = (mask_1 * alpha + mask_2 * (1 - alpha))

        return nav_map_fusion

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
        if self.learning:
            if self.mode == 'VAE':
                self.analyzer_.insert_sample_vae(msg)

            elif self.mode == 'HC':
                self.analyzer_.insert_sample_img(msg)
                self.analyzer_.insert_sample_elev(msg)

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
        if self.mode == 'VAE':
            computed_map = self.analyzer_.recompute_transversality_vae(
                msg, threshold=5.0)
            self.set_layer_data(msg, 'transversality',
                                np.array(computed_map).flatten().tolist())
            self.pub.publish(msg)

        elif self.mode == 'HC':
            map_img = self.analyzer_.recompute_transversality_img(
                msg, threshold=1.5)
            map_elev = self.analyzer_.recompute_transversality_elev(
                msg, threshold=0.1)
            computed_map = self.combine_transversality(map_img, map_elev,
                                                       alpha=0.8)
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
