import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the model from the XML file
model = mujoco.MjModel.from_xml_path('asset/control_demo.xml')
data = mujoco.MjData(model)

# --- Actuator ID's ---
# It's good practice to get actuator IDs by name instead of assuming their order.
torque_act_id = model.actuator('torque_actuator').id
pos_act_id = model.actuator('position_actuator').id
vel_act_id = model.actuator('velocity_actuator').id

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
  start_time = time.time()
  
  while viewer.is_running():
    step_start = time.time()

    # Step the simulation
    mujoco.mj_step(model, data)

    # Render the scene in the viewer
    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)