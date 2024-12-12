import asyncio
from omni.isaac.dynamic_control import _dynamic_control
import time

dc = _dynamic_control.acquire_dynamic_control_interface()

# concept of operations timeline
ftotal = 600
fpause = 100
fmax = (ftotal - fpause) / 2

# robotic arm positions
stowed_positions = [0, -90, -90, 60, 0, -180]
extended_positions = [0, 0, -44, -40, 65, 0]
stowed_positions = [p * 3.14 / 180 for p in stowed_positions]
extended_positions = [p * 3.14 / 180 for p in extended_positions]


# define velocities for Servicer and Client
angular_vel = (-0.5, 0, 0)
linear_vel = (0.55, 0, 0)
linear_vel_pause = (0.25, 0, 0)
servicer_handle = dc.get_rigid_body("/scene/COSMIC/Servicer/Body")
client_handle = dc.get_rigid_body("/scene/COSMIC/Client")


async def combined_control():
    await omni.kit.app.get_app().next_update_async()
    print("SLEEP")  # time to start `ros2 bag record`
    await asyncio.sleep(3)
    
    # set initial conditions: Client is tumbling
    dc.wake_up_rigid_body(client_handle)
    dc.set_rigid_body_angular_velocity(client_handle, angular_vel)

    # articulation and revolute joint handles
    art = dc.get_articulation("/scene/COSMIC/Servicer/Body")
    dofs = [dc.find_articulation_dof(art, f"a{i}RevJoint") for i in range(1, 7)]
    print(art, dofs)

    print("RUN")

    for frame in range(int(ftotal * 1.5)):
        await omni.kit.app.get_app().next_update_async()
        # This should be called each frame of simulation if state on the articulation is being changed.
        dc.wake_up_articulation(art)
        dc.wake_up_rigid_body(servicer_handle)
        for dof, stowed_pos, extended_pos in zip(
            dofs, stowed_positions, extended_positions
        ):
            if frame < fmax:
                # rough approach: reduce distance to client and fold the arm in stowed position
                dc.set_dof_position_target(dof, frame * stowed_pos / fmax)
                dc.set_rigid_body_linear_velocity(servicer_handle, linear_vel)
            elif frame < fmax + fpause:
                # common rate approach slowly and match angular velocity of client
                dc.set_rigid_body_linear_velocity(servicer_handle, linear_vel_pause)
                dc.set_rigid_body_angular_velocity(servicer_handle, angular_vel)
            elif frame < ftotal:
                # final approach: hold relative position and extend the arm
                dc.set_rigid_body_linear_velocity(servicer_handle, (0, 0, 0))
                dc.set_rigid_body_angular_velocity(servicer_handle, angular_vel)
                dc.set_dof_position_target(
                    dof,
                    stowed_pos
                    + (frame - fmax - fpause) / fmax * (extended_pos - stowed_pos),
                )
            else:
                # hold relative position
                dc.set_rigid_body_linear_velocity(servicer_handle, (0, 0, 0))
                dc.set_rigid_body_angular_velocity(servicer_handle, angular_vel)

    print("DONE")


omni.timeline.get_timeline_interface().play()
asyncio.ensure_future(combined_control())


