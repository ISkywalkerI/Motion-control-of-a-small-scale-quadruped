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

paused = False
t = 0

# Initialize trajectory and sensor data lists
x_traj = []
y_traj = []
z_traj = []

sensor_vel_x = []
sensor_vel_y = []
sensor_vel_z = []

def initialpos():
    x = [0.0, 0.0, 0.0, 0.0]
    y = [0.0, 0.0, 0.0, 0.0]
    z = [-0.1, -0.1, -0.1, -0.1]
    angles = ik(x, y, z)
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")

def plot_trajectory(x_traj, y_traj, z_traj, mode='3d'):
    if mode == '3d':
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_traj, y_traj, z_traj, label='Robot Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    elif mode == '2d':
        plt.figure()
        plt.plot(x_traj, y_traj, label='Robot Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
    else:
        raise ValueError("Mode must be '2d' or '3d'")
    
    plt.legend()
    output_path = f"robot_trajectory_{mode}.png"
    plt.savefig(output_path)
    print(f"Trajectory plot saved as {output_path}")

def calculate_and_print_average_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z):
    avg_vel_x = np.mean(sensor_vel_x)
    avg_vel_y = np.mean(sensor_vel_y)
    avg_vel_z = np.mean(sensor_vel_z)

    print(f"Average Velocity X: {avg_vel_x:.4f} m/s")
    print(f"Average Velocity Y: {avg_vel_y:.4f} m/s")
    print(f"Average Velocity Z: {avg_vel_z:.4f} m/s")
    
def plot_sensor_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z):
    plt.figure()
    plt.plot(sensor_vel_x, label='Velocity X')
    #plt.plot(sensor_vel_y, label='Velocity Y')
    #plt.plot(sensor_vel_z, label='Velocity Z')
    plt.xlabel('Timestep')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    
    output_path = "sensor_velocity.png"
    plt.savefig(output_path)
    print(f"Sensor velocity plot saved as {output_path}")

def set_joint_angles_with_actuators(clock):
    global t
    xf, xs = 0.04, 0.0
    yf, ys = 0.0, 0.0
    zs = -0.100
    height = 0.03
    Ts = 3

    # Store the base link's position for trajectory tracking
    if clock > 10:
        x_traj.append(data.body("base_link").xpos[0])
        y_traj.append(data.body("base_link").xpos[1])
        z_traj.append(data.body("base_link").xpos[2])

        # Retrieve the sensor data for velocity (in local frame)
        velocimeter_start_index = model.sensor("Body_velo").adr
        sensor_vel_x.append(data.sensordata[velocimeter_start_index])
        sensor_vel_y.append(data.sensordata[velocimeter_start_index + 1])
        sensor_vel_z.append(data.sensordata[velocimeter_start_index + 2])

    # Calculate the gait trajectory
    x, y, z = gait.trot(xf, xs, yf, ys, zs, height, t, Ts, 0)

    t = t + 0.01
    if t >= Ts:
        t = 0

    print(f"t = {t}")
    angles = ik(x, y, z)
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")

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

    plot_mode = '3d'
    plot_trajectory(x_traj, y_traj, z_traj, mode=plot_mode)
    plot_sensor_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z)
    calculate_and_print_average_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z)
if __name__ == "__main__":
    main()
