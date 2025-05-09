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
ros2 launch self_driving self_driving_world.launch.py
ros2 run self_driving self_driver
```