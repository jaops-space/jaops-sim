# JAOPS ROS2 Workspace
This folder includes packages and robot operation related software for JAOPS space robotics simulation.

<!-- ------------------------------------------------------ -->

## Table of Contents
* [Getting Started](#getting-started)
* [Welcome to the MOON](#welcome-to-the-moon)
* [Foxglove Studio](#foxglove-visualization)
* [Rerun.io](#rerun-visualization)
* [Trouble Shooting](#trouble-shooting)

<!-- ------------------------------------------------------ -->

## Getting Started
Before getting started, we assume that you have run through the main document of the [JAOPS sim](https://github.com/jaops-space/jaops-sim/blob/main/README.md) repository already.

### Prerequisites
* [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* [ros2-humble installation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
* Git LFS which can be installed as follows:
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
git lfs install
```

(Note: If you have installed Git LFS after cloning this repo, you can run `git lfs fetch` followed by `git lfs pull` to download all the `usd` files which are referenced via git lfs)

### Build & Install
* Source ros2 to the terminal

```bash
source /opt/ros/humble/setup.bash
```

* Install dependencies by [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

```bash
sudo apt update

cd jaops-sim/src/ros2_ws/

rosdep install --from-paths src --ignore-src -r -y
```

* Install program

```bash
# In the same directory
colcon build --symlink-install
source install/setup.bash
```

<!-- ------------------------------------------------------ -->

## Welcome to the MOON
To launch the lunar surface simulation demo:
```bash
ros2 launch rover_isaac sim_demo.launch.py
```
Or, if you want to specify a specific .usd:
```bash
ros2 launch rover_isaac sim_demo.launch.py gui:="~/jaops/jaops-sim/scenes/lunar_surface_demo_ros2.usd"
```
Or, if you want to star the simulation immediately after the environment is imported:
```bash
ros2 launch rover_isaac sim_demo.launch.py play_sim_on_start:=true
```
For rectangular trajectory motion.
```bash
ros2 launch rover_nav rectangle.launch.py side_length:=3.0 rounds:=1
```
<!-- ------------------------------------------------------ -->

## Foxglove visualization
### Prerequisites
Follow the ROS Foxglove bridge installation from [official document](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/).

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

### Usage
* Open and play the simulation following the [section](#welcome-to-the-moon).

* Launch the foxglove bridge by the following command.

```bash
ros2 launch rover_isaac foxglove_demo.launch.py
```
* Open Foxglove with [Websocket](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2/#foxglove-websocket) and import the layout for our simulation [here](https://github.com/jaops-space/jaops-sim/blob/dev/foxglove_rerun_integration/src/ros2_ws/demo_foxglove/foxglove_layout/pragyaan.json).

<p align="center">
  <img src="../../images/lunar_demo_foxglove.png" alt="lunar_demo_foxglove.png">
</p>

<!-- ------------------------------------------------------ -->

## Rerun visualization
### Prerequisites
Follow the python installation procedure in rerun.io [official document](https://rerun.io/docs/getting-started/quick-start/python).

```bash
pip3 install rerun-sdk
```

Create python environment
```bash
python3 -m venv venv
source venv/bin/active
(venv) $ pip install -r jaops-sim/src/ros2_ws/rerun_demo/requirements.txt
```

### Usage
* Open and play the simulation following the [section](#welcome-to-the-moon).

* Run the python script to launch rerun GUI
```bash
cd jaops-sim/src/ros2_ws/rerun_demo/
python3 lunar_demo_rerun.py
```

<p align="center">
  <img src="../../images/lunar_demo_rerun.png" alt="lunar_demo_rerun.png">
</p>

<!-- ------------------------------------------------------ -->

## Troubleshooting

### Isaacsim doesn't start

If you have installed isaac-sim to a location other than the default you can enter the path using the `install_path` argument for `sim_demo.launch.py`

### The Stage doesn't load

* If you have cloned this repo to a location other than $HOME, you can enter the path using the `gui` argument for `sim_demo.launch.py`

* Check that `lunar_surface_demo.usd` is not a pointer file. If it is, you may not have git lfs setup. See [section](#getting-started)

Please report bugs using [Issue Tracker](https://github.com/jaops-space/jaops-sim/issues).
