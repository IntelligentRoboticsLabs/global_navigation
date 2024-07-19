import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap as GridMapMsg
from geometry_msgs.msg import PoseStamped

from ground_analyzer import GroundAnalyzer

import numpy as np


class GridMapSubscriber(Node):
    def __init__(self):
        super().__init__('nav_analyzer_node')

        # Set mode:
        # 'img' for image only
        # 'elev' for elevation only
        # 'both' for an average of both
        self.mode = 'both'

        # Set img analyzer mode (only if mode is 'img' or 'both')
        # 'gabor' for Gabor and RGB features
        # 'vae' for VAE features
        self.analyzer_ = GroundAnalyzer(img_mode='gabor')

        

        self.grid_map_sub = self.create_subscription(
            GridMapMsg,
            '/grid_map',
            self.grid_map_callback,
            1
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            1
        )

        self.pose_ = [0, 0]

        self.pub = self.create_publisher(GridMapMsg, 'grid_map_topic_out', 10)


    def set_layer_data(self, msg, layer_name, data):
        layer_index = msg.layers.index(layer_name)
        msg.data[layer_index].data = data
        return msg
    
    def grid_map_callback(self, msg):
        
        if self.mode == 'img':
            self.analyzer_.insert_sample_img(msg, self.pose_)
            self.set_layer_data(msg, 'transversality', self.analyzer_.recompute_transversality_img(msg))
            self.pub.publish(msg)
            
        elif self.mode == 'elev':
            self.analyzer_.insert_sample(msg, self.pose_)
            self.set_layer_data(msg, 'transversality', self.analyzer_.recompute_transversality(msg))
            self.pub.publish(msg)

        elif self.mode == 'both':
            self.analyzer_.insert_sample_img(msg, self.pose_)
            self.analyzer_.insert_sample(msg, self.pose_)
            map_img = self.analyzer_.recompute_transversality_img(msg)
            map_elev = self.analyzer_.recompute_transversality(msg)
            map = (np.array(map_img) + np.array(map_elev))/2
            self.set_layer_data(msg, 'transversality', map.tolist())
            self.pub.publish(msg)

    def pose_callback(self, msg):
        
        self.pose_ = [msg.pose.position.x, msg.pose.position.y]


def main(args=None):
    rclpy.init(args=args)
    grid_map_subscriber = GridMapSubscriber()
    rclpy.spin(grid_map_subscriber)
    grid_map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()