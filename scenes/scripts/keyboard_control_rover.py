import asyncio
from omni.isaac.dynamic_control import _dynamic_control
import carb

STRAIGHT_SPEED = 2
TURN_SPEED = 3
INCREMENT_SPEED = 1

def keyboard_event(event, *args, **kwargs):
	global STRAIGHT_SPEED, TURN_SPEED
	
	if event.type == carb.input.KeyboardEventType.KEY_PRESS:
		dc.wake_up_articulation(art)
		if event.input == carb.input.KeyboardInput.SPACE:
			print("SPACE")
			for dof in dofs:
				dc.set_dof_velocity_target(dof, 0)
		elif event.input == carb.input.KeyboardInput.UP:
			for dof in dofs:
				dc.set_dof_velocity_target(dof, STRAIGHT_SPEED)
		elif event.input == carb.input.KeyboardInput.DOWN:
			for dof in dofs:
				dc.set_dof_velocity_target(dof, -STRAIGHT_SPEED)
		elif event.input == carb.input.KeyboardInput.LEFT:
			print("LEFT")
			for dof, name in zip(dofs, wheel_names):
				dc.set_dof_velocity_target(dof, TURN_SPEED if  name[-1] == 'r' else -TURN_SPEED )
		elif event.input == carb.input.KeyboardInput.RIGHT:
			for dof, name in zip(dofs, wheel_names):
				dc.set_dof_velocity_target(dof, TURN_SPEED  if  name[-1] == 'l' else -TURN_SPEED )
		elif event.input == carb.input.KeyboardInput.A:
			TURN_SPEED -= INCREMENT_SPEED
		elif event.input == carb.input.KeyboardInput.D:
			TURN_SPEED += INCREMENT_SPEED
		elif event.input == carb.input.KeyboardInput.W:
			STRAIGHT_SPEED += INCREMENT_SPEED
		elif event.input == carb.input.KeyboardInput.S:
			STRAIGHT_SPEED -= INCREMENT_SPEED

dc = _dynamic_control.acquire_dynamic_control_interface()
wheel_names = [
	"wheel_fl",  # front left
	"wheel_ml",  
	"wheel_bl",
	"wheel_fr",
	"wheel_mr",
	"wheel_br",  # back right
]

art = dc.get_articulation("/Root/lunarAssets/pragyaan_rover_01/Rover")
dofs = [dc.find_articulation_dof(art, w) for w in wheel_names]
print(art, dofs)

# subscribe to keyboard event
appwindow = omni.appwindow.get_default_app_window()
input = carb.input.acquire_input_interface()
input.subscribe_to_keyboard_events(appwindow.get_keyboard(), keyboard_event)

dc.wake_up_articulation(art)
for dof in dofs:
	dc.set_dof_velocity_target(dof, 1)








