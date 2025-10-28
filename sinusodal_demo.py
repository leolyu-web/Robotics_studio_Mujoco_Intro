import mujoco 
import mujoco.viewer 
import time
import math # Import the math library for the sine function

# Define the file path for your MuJoCo model file (in XML format).
xml_path = "asset/ant_pos_ctrl.xml" 

# Load the model from the specified XML file into an MjModel object.
model = mujoco.MjModel.from_xml_path(xml_path)

# The MjData object contains the dynamic state of the simulation: positions, velocities, forces, sensor data, etc.
data = mujoco.MjData(model)

# Define motion parameters
amplitude = 0.5  # Controls the range of motion (in radians)
frequency = 10   # Controls the speed of the oscillation

with mujoco.viewer.launch_passive(model, data) as viewer:

    # Run the simulation loop as long as the viewer window is open.
    while viewer.is_running():
        step_start = time.time()

        # Apply a sinusoidal control signal to the first actuator (front-left hip).
        data.ctrl[0] = amplitude * math.sin(frequency * data.time)

        # Step the simulation
        mujoco.mj_step(model, data)

        # Update the viewer's display with the new state from the d (MjData) object.
        viewer.sync()

        #Optional: To maintain a real-time simulation speed. m.opt.timestep is the desired duration of one step.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)