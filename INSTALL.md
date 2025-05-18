# Install

Need Gazebo Harmonic and ROS2 Jazzy

### Gazebo Harmonic

```shell
# Gazebo Harmonic
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
# Uninstall
sudo apt remove gz-harmonic && sudo apt autoremove
```

### ROS2 Jazzy

```shell
# ROS2 Jazzy
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
```

### ROS2 extra

```shell
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-joint-state-publisher
sudo apt-get install ros-jazzy-tf-transformations
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-gazebo-ros-pkgs
```