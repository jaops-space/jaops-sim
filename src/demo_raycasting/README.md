# Raycasting Demo

This is a simple demo jupyter notebook to show :
- The jupyter workflow with IsaacSim. See [here](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_jupyter.html) for a tutorial
- Using IsaacSim headless, and getting data from the simulation (see for example the image data on the left of the figure below)
- Using the raycasting API python Bindings. `get_physx_scene_query_interface().raycast_closest()` to cast rays from a position towards a direction and report any hit with a mesh (see [doc](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/index.html#raycast) and [Python Bindings API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/index.html?highlight=raycast#omni.physx.bindings._physx.PhysXSceneQuery.raycast_closest)) (see for example the Sun angle map computed on the right of the figure below)

| <img alt="Solar Panel Power Timelapse" src="../../images/raycasting_sceneOverview.png" width="100%">  | <img alt="Solar Panel Power Timelapse" src="../../images/raycasting_sunAngleMap.png" width="75%"> |
|:---:|:---:|

For more concrete usage of raycasting, see the other demo where it is used to compute solar power generation and the temperature of assets on the lunar surface: [demo_power_thermal](../demo_power_thermal/)
