# Solar Power Generation and Thermal Computation Demo

This demo showcases simple Solar Power Generation and Thermal Computation on the lunar surface.
It shows how to leverage IsaacSim's raytracing functionality for accurate computation of surface orientations and shadowing effects.

| <img alt="Solar Panel Power Timelapse" src="../../images/SolarPanelPower_timelapse.png" width="100%"> | <img alt="Thermal Computation Timelapse" src="../../images/ThermalComputation_timelapse.png" width="100%">  |
|:---:|:---:|
| Timelapse simulation of solar panel power generation from sunrise to solar noon. The color-coding of each solar cell of the solar panel shows the power generation: blue is 0 W while red is 60 W | Timelapse simulation of the temperatures of the lunar surface and solar panel from sunrise to sunset. The color-coding of each mesh vertex shows the temperature: blue is 200 K while red is 450 K|

See the full video sequences at https://youtu.be/sR3g4kOGc6k


The demo is based on the following python APIs:
- `omni.isaac.core` to setup the scene and access prim attributes
- `get_physx_scene_query_interface().raycast_closest()` to cast rays from a position towards a direction and report any hit with a mesh (see [doc](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/index.html#raycast) and [Python Bindings API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/index.html?highlight=raycast#omni.physx.bindings._physx.PhysXSceneQuery.raycast_closest))


## Usage

**mesh_raycast_utils.py**: This module provides utility functions for working with meshes, raycasting, and lighting in an IsaacSim 3D stage. These utilities are used in the power_thermal_computation.py script to compute the orientation and shadowing of surface with respect to the Sun or the sky.

**power_thermal_computations.py**: This module provides functions to compute the power generation of solar cells and the temperature of assets on the lunar surface. Common solar power and thermal constants are also provided.

**power_thermal_demo.ipynb**: This is the main demo that uses the above two modules to produce power and thermal computations, animate the results in IsaacSim and plot the results on graphs.

1. Prerequisite: familiarize yourself with the [IsaacSim jupyter workflow](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_jupyter.html)
2. Launch IsaacSim via the Omniverse launcher
3. Open the .usd that will be synced with the jupyter notebook: in our case `omniverse://localhost/Users/{user}/power_thermal_demo.usd"`
4. Click on the `LIVE` button at the top right of the IsaacSim window to start the session
5. Execute the cells in the notebook, the simulated scene will be displayed in the IsaacSim window


## Further information:

See the paper published at IAC in November 2024:

> Burtz, L.-J., Sinsunthorn, T., & Sela, A. (2024). Lunar Surface Visual Rendering, Dynamics, Solar Power and Thermal Simulation for the Operations of Lunar Rover Missions. IAC 2024
[Available Online](https://www.researchgate.net/publication/386223716_Lunar_Surface_Visual_Rendering_Dynamics_Solar_Power_and_Thermal_Simulation_for_the_Operations_of_Lunar_Rover_Missions)


|![alt text](../../images/SolarPanelPower_graph.png)| ![alt text](../../images/ThermalComputation_graph.png)|
|:---:|:---:|
| Sum of the power generation of each solar cell of the solar panel across the lunar day for a static solar panel facing upwards | Plot of the evolution of the computed temperatures during the lunar day of the lunar surface, solar panel and two supports |
