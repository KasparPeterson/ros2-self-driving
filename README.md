# Self driving robot

Robot drives to a given coordinates.

- [x] drives to a given CLI coordinates
- [ ] give coordinates dynamically and subscribe to a target coordinates
- [ ] avoid obstacles with the lidar
- [ ] avoid obstacles with camera
- [ ] learn the environment SLAM

## Demo

<video src="assets/demo_first_iteration.mp4" controls width="600"></video>

## Setup

```shell
export GZ_SIM_RESOURCE_PATH=/mnt/utm/self_driving/src/self_driving/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=/media/psf/UTMCodeSharedUbuntu/self_driving/src/self_driving/models:$GZ_SIM_RESOURCE_PATH
export GZ_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gz-sim-8/plugins:$GZ_PLUGIN_PATH
export GZ_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gz-gui-8/plugins:$GZ_PLUGIN_PATH


rosdep install -r --from-paths src -i -y --rosdistro jazzy
colcon build --packages-select self_driving
source install/setup.bash
```

### Usage

```shell
ros2 launch self_driving self_driving_world.launch.py
ros2 run self_driving self_driver 20 0
```