import mujoco
import numpy as np
import mujoco.viewer
import time

xml_path = "QTA.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

def print_relative_foot_positions():
    # Get the ID for the base_link body
    base_link_id = model.body("base_link").id
    
    # Get the position of the base_link in the global coordinate frame
    base_position = data.xpos[base_link_id]

    foot_sites = ["Lf3_foot", "Rf3_foot", "Lb3_foot", "Rb3_foot"]

    for foot_name in foot_sites:
        site_id = model.site(foot_name).id
        foot_position_global = data.site_xpos[site_id]
        
        # Calculate the position relative to the base_link
        foot_position_relative = foot_position_global - base_position
        
        print(f"{foot_name} relative position: x={foot_position_relative[0]:.4f}, y={foot_position_relative[1]:.4f}, z={foot_position_relative[2]:.4f}")

def main():
    viewer = mujoco.viewer.launch_passive(model=model, data=data)
    
    if viewer is None:
        print("Failed to launch viewer.")
        return

    mujoco.mj_resetData(model, data)
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    while viewer.is_running():
        # 每次步进后，输出四足端点相对base_link的位置
        print_relative_foot_positions()

        mujoco.mj_step(model, data)
        viewer.sync()

if __name__ == "__main__":
    main()
