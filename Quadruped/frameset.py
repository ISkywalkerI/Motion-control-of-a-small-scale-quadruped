import mujoco
import numpy as np
import mujoco.viewer
import time

xml_path = "QTA.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Constants
dt = 0.001
max_angvel = 8.0
integration_dt = dt
damping = 1e-4

def set_foot_positions_with_ik(desired_positions):
    base_link_id = model.body("base_link").id
    base_position = data.xpos[base_link_id]

    for foot_name, target_position in desired_positions.items():
        site_id = model.site(foot_name).id

        for _ in range(10):  # Iterate to converge to the desired position
            # Initialize the Jacobian matrix for the site
            jac = np.zeros((6, model.nv))  # 6xN Jacobian matrix for position and orientation
            mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
            
            # Calculate the current position error
            current_position = data.site_xpos[site_id]
            position_error = target_position - (current_position - base_position)
            jac_pos = jac[:3, :]
            
            # Solve system of equations: J @ dq = error.
            diag = damping * np.eye(3)  # Regularization term
            dq = jac_pos.T @ np.linalg.solve(jac_pos @ jac_pos.T + diag, position_error)
            
            # Scale down joint velocities if they exceed maximum.
            if max_angvel > 0:
                dq_abs_max = np.abs(dq).max()
                if dq_abs_max > max_angvel:
                    dq *= max_angvel / dq_abs_max

            # Integrate joint velocities to obtain joint positions.
            q = data.qpos[:model.nq].copy()  # Only copy relevant part of qpos
            mujoco.mj_integratePos(model, q, dq, integration_dt)

            # Apply joint limits and update control signals
            actuator_names = ["lf1", "lf2", "lf3", "rf1", "rf2", "rf3", "lb1", "lb2", "lb3", "rb1", "rb2", "rb3"]
            actuator_ids = [model.actuator(name).id for name in actuator_names]
            dof_ids = [model.jnt_dofadr[model.actuator(actuator_id).trnid[0]] for actuator_id in actuator_ids]
            jnt_ranges = model.jnt_range[dof_ids]
            
            # Clip the joint positions to their limits
            np.clip(q[dof_ids], jnt_ranges[:, 0], jnt_ranges[:, 1], out=q[dof_ids])

            # Set the control signal for actuators
            data.ctrl[actuator_ids] = q[dof_ids]
            print(f"Setting {foot_name} related actuators to positions: {q[dof_ids]} radians")

def main():
    viewer = mujoco.viewer.launch_passive(model=model, data=data)
    
    if viewer is None:
        print("Failed to launch viewer.")
        return

    mujoco.mj_resetData(model, data)
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    # Example: Define desired foot positions relative to base_link
    desired_positions = {
        "Lf3_foot": np.array([0.0547, 0.0468, -0.15]),  # Front-left foot
        "Rf3_foot": np.array([0.0547, -0.0468, -0.15]),  # Front-right foot
        "Lb3_foot": np.array([-0.0875, 0.05, -0.1074]),  # Back-left foot
        "Rb3_foot": np.array([-0.0875, -0.05, -0.1074]),  # Back-right foot
    }

    while viewer.is_running():
        # Set the foot positions via inverse kinematics each time step
        set_foot_positions_with_ik(desired_positions)

        mujoco.mj_step(model, data)
        viewer.sync()

if __name__ == "__main__":
    main()
