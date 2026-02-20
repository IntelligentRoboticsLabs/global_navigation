# local_navigation

This package contains our work about traversity

### How to run:

1. In terminal 1 play a rosbag:

```
ros2 bag play rosbags/trial_0 --clock -p
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
