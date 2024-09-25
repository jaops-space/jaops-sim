# Foxglove visualization

## Prerequisties
Follow the ROS Foxglove bridge installation from [official document](https://docs.foxglove.dev/docs/connecting-to-data/ros-foxglove-bridge/) 

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge 
```

# Usage
Open Foxglove with [Websocket](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2/#foxglove-websocket)

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=1000000000 use_sim_time:=true
```

# Rerun visualization

## Prerequisties
Follow the python installation procedure in rerun.io [official document](https://rerun.io/docs/getting-started/quick-start/python) 

```bash
pip3 install rerun-sdk
```

Create python environment
```bash
python3 -m venv venv
source venv/bin/active
(venv) $ pip install -r jaops-sim/src/ros2_ws/rerun_demo/requirements.txt
```

## Usage
```bash
cd jaops-sim/src/ros2_ws/rerun_demo/
python3 lunar_demo_rerun.py
```