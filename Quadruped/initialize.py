import mujoco
import mujoco.viewer
from IK import ik

xml_path = "QTA.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Constants
dt = 0.001
damping = 1e-4
max_angvel = 8.0
integration_dt = dt
# Override the simulation timestep.
model.opt.timestep = dt


def initialpos():
    x = [0.04, 0.04, 0.04, 0.04]
    y = [0.0, 0.0, 0.0, 0.0]
    z = [-0.08, -0.08, -0.10, -0.10]
    angles = ik(x,y,z)
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")