[![rolling](https://github.com/midemig/global_navigation/actions/workflows/rolling.yaml/badge.svg)](https://github.com/midemig/global_navigation/actions/workflows/rolling.yaml)

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

### Installation

- Requirements
  - Ubuntu 24.04
  - Ros2 Rolling
  - Python 3.12.3


- Create a workspace and clone the repository

    ```sh
    mkdir -p global_navigation_ws/src
    cd global_navigation_ws/src
    git clone https://github.com/midemig/global_navigation -b rolling
    ```

- Install dependences and build workspace

    ```sh
    sudo apt install libg2o-dev
    rosdep update
    vcs import --recursive . < global_navigation/dependencies.repos
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install 
    ```

- Install python dependences

    ```sh
    sudo apt install python3.12-venv
    python3 -m venv global_nav_env
    source global_nav_env/bin/activate
    pip3 install -r src/global_navigation/requirements.txt
    ```

### Launch Demo

1. Download and unzip the [demo bagfile](https://urjc-my.sharepoint.com/:u:/g/personal/juancarlos_serrano_urjc_es/EQI9T9RNYuFJg6reV-pq-7IBjMEeEo7RxaJCudMs9IyRTg?e=hSNyQB).
2. In a terminal, play the downloaded bag:

    ```sh
    ros2 bag play cesped_00/ --clock -p
    ```

3. In another terminal, execute:

    ```sh
    source global_nav_env/bin/activate
    export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
    ros2 launch local_navigation demo.launch.py
    ```

## Authors

- [Francisco Martín Rico](github.com/fmrico)
- [Miguel Ángel de Miguel](github.com/midemig)
- [Juan Sebastián Cely](github.com/juanscelyg)
- [Juan Carlos Manzanares](github.com/Juancams)
- [Alberto García](github.com/aaggj)
