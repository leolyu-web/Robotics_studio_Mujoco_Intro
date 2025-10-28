import mujoco
import mujoco.viewer
import numpy as np
import time

# XML model string for the two boxes
xml = "asset/two_bars.xml"

# Load the model and data
model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
  start_time = time.time()
  
  # Run for 10 seconds
  while viewer.is_running(): 
    step_start = time.time()

    # Step the simulation
    mujoco.mj_step(model, data)

    # Sync the viewer with the simulation
    viewer.sync()

    # Wait to maintain a reasonable simulation speed
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

print("Simulation finished.")