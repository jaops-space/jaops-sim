# JAOPS SIM

[![Ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-green.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ros2-humble installation](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
[![License](https://img.shields.io/badge/license-MIT-red.svg)](https://opensource.org/licenses/BSD-3-Clause)

<p align="center">
  <img alt="Lunar Surface Simulator Overview" src="./images/LunarSurface_overview.png" width="47%">
&nbsp;
  <img alt="On-Orbit Services Simulator Overview" src="./images/OnOrbitServices_overview.png" width="47%">
</p>


[JAOPS](https://www.jaops.com/) provides several types of simulation environments for spacecraft in orbit and lunar rovers on the Moon. This suite of tools is the backbone of modern iterative development and enables learning by doing. The simulator provides realistic sensor data, a telemetry/telecommand interface for integrated software testing, and an immersive experience for training operators.

[work in progress to bring previous work into this open source repo]

|Scenes| |
|----|----|
|Lunar Surface Simulator preview: six-wheel rocker-bogie rover in lunar lander garden [Youtube](https://www.youtube.com/watch?v=z7fS4HvoUb8) | <img alt="Lunar Surface Demo" src="./images/LunarSurface_demo.png" width="40%"> |
|Particle Simulation preview: Lunar Rover Wheel/Soil Interaction [Youtube](https://www.youtube.com/watch?v=96t_Y4Iza8Q)| <img alt="Particle Simulation Demo" src="./images/ParticleSimulation_demo.png" width="40%">|
|On-Orbit Services Simulation preview: servicer spacecraft approaching un-cooperative client [Youtube](https://www.youtube.com/watch?v=ziZgHS5BDNg) | <img alt="On-Orbit Services Demo" src="./images/OnOrbitServices_demo.png" width="40%">|

## System Requirement
* [Requirements for IsaaSim should be satisfied](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements)
* 12GB+ of free RAM space

## Setup

1. Install git LFS (only needed once per user per machine)
```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
git lfs install
```
2. Clone this git repository
```bash
git clone https://github.com/jaops-space/jaops-sim.git
```
3. Make sure that the LFS tracked files (e.g .USD and .PNG) are properly downloaded (not just pointers). For instance check that the .PNG images of this README display properly. If any issue, run
```bash
cd jaops-sim
git lfs fetch
git lfs pull
```

## Usage
1. Launch Omniverse IsaacSim and open any of the USD corresponding to the scenes shown in the table above. Explore the different assets, re-use them in your own simulations
2. See also the demos in the `src` folder. Each one has its own corresponding README. See for instance the ROS2 foxglove and rerun interfaces for the lunar simulation scene [here](./src/ros2_ws/README.md)
