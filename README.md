
# Global Navigation

This repository introduces an innovative approach for outdoor navigation using ROS2.

## Packages

This repository comprises two packages:

- **local_navigation**: Reconstructs the navigated area into a grid map incorporating elevation and RGB data.
- **traversability_updater**: Computes a navigability score based on previously traversed areas.

The animation below highlights highly navigable areas in yellow and green. As the robot traverses the grass, all mapped zones with similar characteristics are marked as highly navigable.

![Navigation Demo](media/navigation_demo.gif)

## Usage

To reproduce our results, follow these instructions:

### Prerequisites

- Install ROS2 Rolling.
- Clone [Lidarslam package](https://github.com/rsasaki0109/lidarslam_ros2) to your workspace.

### Launch Demo

1. Download and unzip the [demo bagfile](https://urjc-my.sharepoint.com/:u:/g/personal/juancarlos_serrano_urjc_es/EQI9T9RNYuFJg6reV-pq-7IBjMEeEo7RxaJCudMs9IyRTg?e=hSNyQB).
2. In a terminal, play the downloaded bag:
    ```sh
    cd bagfile_folder/
    ros2 bag play *.db3 --clock -p
    ```
3. In another terminal, execute:
    ```sh
    ros2 launch local_navigation demo.launch.py
    ```




