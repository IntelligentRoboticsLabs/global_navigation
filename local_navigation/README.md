# local_navigation

This package contains our work about traversity

### How to run:

1. In terminal 1 play a rosbag:

```
ros2 bag play rosbags/summit/test_1_1 --clock -p --remap /tf_static:=/rosbag/tf_static /tf:=/rosbag/tf 
ros2 run local_navigation summit_correct_zed_tf --ros-args -p use_sim_time:=True

```

2. In terminal 2 launch the nodes:

(Remember to check remaps in launcher, this is for Summit XL)

```
ros2 launch local_navigation local_navigation.launch.py
```

3. In terminal 2 visualize:

```
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```
