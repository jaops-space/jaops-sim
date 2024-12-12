# Satellite Manipulation Demo

Simple demo showing a servicer satellite approaching an un-cooperative client spacecraft

Initial conditions:
- the Servicer is 3-axis stabilized, about 10 meters away from the client
- the Client is tumbling

Concept of Operations:
1. Rough approach: the Servicer locates the Client in the navigation camera and reduces the distance to the Client. During this phase, the robotic arm is stowed in ‘carry’ position
2. Common rate approach: the Servicer computes the relative pose of the Client and characterizes its angular rate using the Stereo Camera. The Servicer reduces the distance to the Client slowly while matching the angular rate of the Client
3. Final approach: the Servicer holds a fixed distance to the Client, and matches the angular rate. The robotic arm is extended towards the Client to be ready for capture. The ArmCam on the end effector shows the proximity to the client.

The contact and capture sequence itself is out of the scope of this demo.


## Usage

1. Launch IsaacSim (Make sure ROS2 is installed and sourced in .bashrc. Make sure that the Isaacsim extension `ROS2 Bridge` is enabled and set to 'autoload')
2. Open scenes/OnOrbitServices_Cosmic_scene.usd
3. Open the IsaacSim menu `Window/Script Editor` and open the file `scripts/satellite_combined_control.py`
4. Press Ctrl-Enter within the script editor window will start the simulation (which includes ROS2 publishing nodes) and control the satellites rigid bodies and robotic arm joint positions.
5. You can visualize the position and velocities of the servicer body, arm end effector and the passive client in Foxglove (see live visualization)


## Live Visualization
1. Install foxglove bridge `sudo apt install ros-$ROS_DISTRO-foxglove-bridge`
2. launch it in a separate terminal: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
3. launch foxglove and connect via Foxglove WebSocket protocol
4. import the layout `satelliteManipulation_foxglove.json`


## ROSBag Visualization
1. Launch foxglove (no need for the foxglove bridge)
2. (optional) in a separate terminal: `ros2 bag play bags/satellite_manipulation` this will decompress and play the bag. Stop the bag play with Ctrl-C.
2. From foxglove: click on Open Local File and select satellite_manipulation.db3

![Foxglove Visualization](./satellite_graphs_foxglove.png)
