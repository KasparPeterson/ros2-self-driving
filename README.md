# Self driving robot

Robot drives to a given coordinates.

## Setup

```shell
rosdep install -r --from-paths src -i -y --rosdistro jazzy
colcon build --packages-select self_driving
source install/setup.bash
```

### Usage

```shell
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.1}"

ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
ros2 run self_driving self_driver
```