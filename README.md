[![jazzy](https://github.com/midemig/global_navigation/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/midemig/global_navigation/actions/workflows/jazzy.yaml)

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
  - Ros2 Jazzy
  - Python 3.12.3

<!-- - Or use [distrobox](https://github.com/89luca89/distrobox)

    ```sh
    distrobox create --image ghcr.io/sloretz/ros:jazzy-desktop --name jazzy-desktop --home /path_to_container_home
    distrobox enter jazzy-desktop
    ``` -->

- Create a workspace and clone the repository

    ```sh
    mkdir -p global_navigation_ws/src
    cd global_navigation_ws/src
    git clone https://github.com/midemig/global_navigation -b jazzy
    ```

- Install dependences and build workspace

    ```sh
    sudo apt install libg2o-dev
    rosdep update
    vcs import --recursive . < global_navigation/dependencies.repos
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    source /opt/ros/jazzy/setup.bash
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

1. Download and unzip the [demo bagfile](https://urjc-my.sharepoint.com/:u:/g/personal/miguelangel_demiguel_urjc_es/IQCCUBCLxZOsQKuwxqeLw5i5ARZLa8Z3k4lTLb0z8eX362s?e=j1npwV).
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

## Citing This Work

If you use this code in your research, please cite the corresponding paper:

**BibTeX:**
```bibtex
@misc{dmiguel2025,
  title={I Move Therefore I Learn: Experience-Based Traversability in Outdoor Robotics},
  author={Miguel Ángel de Miguel, Jorge Beltrán, Juan S. Cely, Francisco Martín, Juan Carlos Manzanares, Alberto García},
  year={2025},
  eprint={2507.00882},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  note={\url{https://doi.org/10.48550/arXiv.2507.00882}}
}
```
    

## Authors

- [Miguel Ángel de Miguel](github.com/midemig)
- [Jorge Beltrán](github.com/beltransen)
- [Juan Sebastián Cely](github.com/juanscelyg)
- [Francisco Martín Rico](github.com/fmrico)
- [Juan Carlos Manzanares](github.com/Juancams)
- [Alberto García](github.com/aaggj)
