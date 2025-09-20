import gymnasium.envs.mujoco.assets
import os
import mujoco
import mujoco.viewer
import time

# --- Main script ---
xml_path = "ant_pos_ctrl.xml"

# 3. Load the model from the file path string
print(f"Loading model from: {xml_path}")
m = mujoco.MjModel.from_xml_path(xml_path)
d = mujoco.MjData(m)

# 4. Launch the passive viewer
with mujoco.viewer.launch_passive(m, d) as viewer:
    print("\nViewer launched. Press Ctrl+C in the terminal to exit.")

    # Run a simple simulation loop
    start = time.time()
    while viewer.is_running():
        step_start = time.time()

        # Step the simulation
        mujoco.mj_step(m, d)

        # Sync the viewer with the simulation data
        viewer.sync()

        # Wait for the next frame time
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)