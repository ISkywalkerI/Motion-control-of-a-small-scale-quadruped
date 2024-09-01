import mujoco
import numpy as np
import mujoco.viewer
import matplotlib.pyplot as plt
from IK import ik
import gait

xml_path = "QTA.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
speed = 0.05
# Constants
dt = 0.001
model.opt.timestep = dt
model.opt.gravity = [0, 0, -9.81]
model.geom_friction[:] = np.array([1.0, 0.5, 0.5])

t = 0
x_rel_traj = []
z_rel_traj = []

x_gait_traj = []
z_gait_traj = []

x_diff_traj = []
z_diff_traj = []

paused = False

def initialpos():
    x = [0.0, 0.0, 0.0, 0.0]
    z = [-0.1, -0.1, -0.1, -0.1]
    angles = ik(x, [0.0]*4, z)  # Pass a dummy y array with zero values
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")

def plot_difference_trajectory(x_diff_traj, z_diff_traj):
    plt.figure()
    
    # Plot the difference in the trajectory
    plt.plot(x_diff_traj, z_diff_traj, label='Difference in Trajectory', color='green')
    
    plt.xlabel('Difference in X')
    plt.ylabel('Difference in Z')
    
    # Move the legend to the upper-right corner
    plt.legend(loc='upper right')
    
    output_path = "trajectory_difference_2d.png"
    plt.savefig(output_path)
    print(f"Trajectory difference plot saved as {output_path}")

def plot_relative_and_gait_trajectory(x_rel_traj, z_rel_traj, x_gait_traj, z_gait_traj):
    plt.figure()
    
    # Plot the relative trajectory
    plt.plot(x_rel_traj, z_rel_traj, label='Simulate Trajectory', color='blue')
    
    # Plot the gait trajectory
    plt.plot(x_gait_traj, z_gait_traj, label='Expect Trajectory', color='red')
    
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.legend(loc='upper right')
    
    output_path = "combined_trajectory_2d.png"
    plt.savefig(output_path)
    print(f"Combined trajectory plot saved as {output_path}")

def set_joint_angles_with_actuators(clock):
    global t
    xf, xs = 0.04, 0.0
    zs = -0.10
    height = 0.02
    Ts = 2.5

    lf3_foot_pos = data.site("Lf3_foot").xpos
    lf1_joint_site_pos = data.site("Lf2_joint_site").xpos

    # Calculate relative position
    x_rel = lf3_foot_pos[0] - lf1_joint_site_pos[0]
    z_rel = lf3_foot_pos[2] - lf1_joint_site_pos[2]

    # Get the gait trajectory
    x, _, z = gait.trot(xf, xs, 0.0, 0.0, zs, height, t, Ts, 0)  # Ignore y-axis values
    x_diff = x_rel - x[0]
    z_diff = z_rel - z[0]

    # Only record data if clock > 15 seconds
    if clock > 15 and clock < 30:
        x_rel_traj.append(x_rel)
        z_rel_traj.append(z_rel)
        x_diff_traj.append(x_diff)
        z_diff_traj.append(z_diff)
    # Only record data if clock > 5 seconds
    if clock > 5:
        x_gait_traj.append(x[0])
        z_gait_traj.append(z[0])

    t = t + 0.01
    if t >= Ts:
        t = 0

    angles = ik(x, [0.0]*4, z)  # Pass a dummy y array with zero values
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle

def key_callback(keycode):
    global paused
    if chr(keycode) == ' ':
        paused = not paused

def main():
    viewer = mujoco.viewer.launch_passive(model=model, data=data, key_callback=key_callback)
    
    if viewer is None:
        print("Failed to launch viewer.")
        return

    mujoco.mj_resetData(model, data)
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    clock = 0

    initialpos()
   
    while viewer.is_running():
        clock = clock + 0.01
        print(f"clock {clock}")

        if not paused:
            if clock > 5:
                set_joint_angles_with_actuators(clock)

            
            mujoco.mj_step(model, data)
            viewer.sync()

    plot_relative_and_gait_trajectory(x_rel_traj, z_rel_traj, x_gait_traj, z_gait_traj)

    #plot_difference_trajectory(x_diff_traj, z_diff_traj)

if __name__ == "__main__":
    main()
