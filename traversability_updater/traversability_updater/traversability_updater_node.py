import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap as GridMapMsg
from geometry_msgs.msg import PoseStamped

from ground_analyzer import GroundAnalyzer

import numpy as np


class GridMapSubscriber(Node):
    def __init__(self):
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
        layer_index = msg.layers.index(layer_name)
        msg.data[layer_index].data = data
        return msg

    def combine_transversality(self, mask_1, mask_2, alpha=0.5):

        nav_map_fusion = (mask_1 * alpha + mask_2 * (1 - alpha))

        return nav_map_fusion
    
    def subgrid_map_callback(self, msg):
        
        if self.learning:
            if self.mode == 'VAE':
                self.analyzer_.insert_sample_vae(msg)

            elif self.mode == 'HC':
                self.analyzer_.insert_sample_img(msg)
                self.analyzer_.insert_sample_elev(msg)
    
    def grid_map_callback(self, msg):
        
        if self.mode == 'VAE':
            map = self.analyzer_.recompute_transversality_vae(msg, threshold=5.0)
            self.set_layer_data(msg, 'transversality', np.array(map).flatten().tolist())
            self.pub.publish(msg)

        elif self.mode == 'HC':
            map_img = self.analyzer_.recompute_transversality_img(msg, threshold=1.5)
            map_elev = self.analyzer_.recompute_transversality_elev(msg, threshold=0.1)
            map = self.combine_transversality(map_img, map_elev, alpha=0.8)
            self.set_layer_data(msg, 'transversality', np.array(map).flatten().tolist())
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    grid_map_subscriber = GridMapSubscriber()
    rclpy.spin(grid_map_subscriber)
    grid_map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()