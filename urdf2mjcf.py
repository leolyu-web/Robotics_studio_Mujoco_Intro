import mujoco

#The path
input_urdf = ""
output_mjcf = ""

# 1. Load the URDF model using the built-in function
# This parses the URDF and compiles it into an mjModel object
model = mujoco.MjModel.from_xml_path(input_urdf)

# 2. Save the compiled model as an MJCF XML file
# This function saves the in-memory model to the MJCF format
mujoco.mj_saveLastXML(output_mjcf, model)
