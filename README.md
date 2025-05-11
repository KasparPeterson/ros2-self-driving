# Self driving robot

Robot drives to a given coordinates.

- [x] drives to a given CLI coordinates
- [ ] subscribe to a target coordinates
- [ ] avoid obstacles from the lidar

## Setup

```shell
export GZ_SIM_RESOURCE_PATH=/mnt/utm/self_driving/src/self_driving/models:$GZ_SIM_RESOURCE_PATH
rosdep install -r --from-paths src -i -y --rosdistro jazzy
colcon build --packages-select self_driving
source install/setup.bash
```

### Usage

```shell
ros2 launch self_driving self_driving_world.launch.py
ros2 run self_driving self_driver
```