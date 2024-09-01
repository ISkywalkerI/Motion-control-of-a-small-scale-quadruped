import mujoco
import numpy as np
import mujoco.viewer
import time

xml_path = "QTA.xml"



# Constants
dt = 0.001  # Simulation timestep
damping = 1e-4  # Damping for Jacobian pseudo-inverse
max_angvel = 8.0  # Maximum angular velocity for scaling
integration_dt = dt  # Integration timestep
amplitude = 0.03  # Amplitude of the leg motion in meters
frequency = 1  # Frequency of the gait in Hz



def main():
    model = mujoco.MjModel.from_xml_path(xml_path)  # MuJoCo model
    data = mujoco.MjData(model)  # MuJoCo data

    # Override the simulation timestep.
    model.opt.timestep = dt
    # End-effector site we wish to control, e.g., one of the foot sites.
    site_names = [
        "Lf3_foot",  # Left front leg foot
        "Rf3_foot",  # Right front leg foot
        "Lb3_foot",  # Left back leg foot
        "Rb3_foot",  # Right back leg foot
        ]
    
    site_ids = [model.site(name).id for name in site_names]
    # Name of joints we wish to control.
    joint_names = [
        "servo1",
        "servo2",
        "servo3",
        "servo4",
        "servo5",
        "servo6",
        "servo7",
        "servo8",
        "servo9",
        "servo10",
        "servo11",
        "servo12",
    ]

    
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    actuator_names = [
    "lf1", "lf2", "lf3", "rf1",
    "rf2", "rf3", "lb1", "lb2",
    "lb3", "rb1", "rb2", "rb3",
    ]
    actuator_ids = np.array([model.actuator(name).id for name in actuator_names])

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    diag = damping * np.eye(6)
    error = np.zeros(6)
    error_pos = error[:3]
    error_ori = error[3:]
    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)

    # def walk_gait(t: float, leg_index: int, amplitude: float, frequency: float) -> np.ndarray:
        # """
        # Generate the walking gait for quadruped robot legs.

        # Parameters:
        # - t: current time
        # - leg_index: index of the leg (0 to 3)
        # - amplitude: motion amplitude
        # - frequency: motion frequency

        # Returns:
        # - Target position [x, y, z]
        # """
        # # Define phase shifts for legs in a walk gait
        # phase_shifts = [0, 0.5, 0.25, 0.75]  # Front left, Front right, Back left, Back right
        # phase = phase_shifts[leg_index]
        
        # cycle_time = 1.0 / frequency
        # cycle_progress = (t + phase * cycle_time) % cycle_time
        
        # # Lifting phase
        # if cycle_progress < 0.5 * cycle_time:
        #     z = amplitude * np.sin(2 * np.pi * frequency * cycle_progress)
        # else:  # Stance phase (foot on the ground)
        #     z = 0.0
        
        # # x motion is continuous, representing forward motion
        # x = amplitude * np.sin(2 * np.pi * frequency * (t + phase))
        # y = 0.0  # No lateral movement in a typical walk gait
    #     return np.array([x, y, z])


    # Define manual target positions for each leg (you can change these values)
    manual_targets = {
        "Lf3_foot": np.array([0.05328, 0.0467, -0.02]),
        "Rf3_foot": np.array([0.05328, -0.0467, -0.02]),
        "Lb3_foot": np.array([-0.1, 0.1, -0.02]),
        "Rb3_foot": np.array([-0.1, -0.1, -0.02]),
    }

    with mujoco.viewer.launch_passive(model=model, data=data,) as viewer:
        # Reset the simulation to the initial state.
        mujoco.mj_resetData(model, data)

        # Initialize the camera view.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Toggle site frame visualization.
        # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        while viewer.is_running():
            step_start = time.time()



            for leg_index, site_id in enumerate(site_ids):
                site_name = site_names[leg_index]
                target_pos = manual_targets[site_name]

                foot_position = data.site_xpos[site_id]
                print(f"Current position of {site_name}: {foot_position}")

                # print(f"Target position for {site_name}: {target_pos}")
                
                # # Calculate position error based on target position and current site position
                # error_pos[:] = target_pos - data.site(site_id).xpos
                # print(f"Position error for {site_name}: {error_pos}")

                # # Get the Jacobian with respect to the end-effector site
                # mujoco.mj_jacSite(model, data, jac[:3], None, site_id)

                # # Only use the positional part of the Jacobian
                # jac_pos = jac[:3]

                # # Solve system of equations: J @ dq = error_pos.
                # dq = jac_pos.T @ np.linalg.solve(jac_pos @ jac_pos.T + diag[:3, :3], error_pos)

                # # Scale down joint velocities if they exceed maximum
                # if max_angvel > 0:
                #     dq_abs_max = np.abs(dq).max()
                #     if dq_abs_max > max_angvel:
                #         dq *= max_angvel / dq_abs_max

                # # Integrate joint velocities to obtain joint positions
                # q = data.qpos.copy()  # Copy the entire qpos array
                # mujoco.mj_integratePos(model, q, dq, integration_dt)

                # # Apply the calculated joint positions to the actuators directly
                # for i in range(len(actuator_ids)):
                #     data.ctrl[actuator_ids[i]] = q[dof_ids[i]]

            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()