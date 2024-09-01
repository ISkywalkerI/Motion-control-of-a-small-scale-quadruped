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
# damping = 1e-4
# max_angvel = 8.0
# integration_dt = dt
# Override the simulation timestep.
model.opt.timestep = dt
model.opt.gravity = [0, 0, -9.81]
model.geom_friction[:] = np.array([1.0, 0.5, 0.5])
# x = [0.05, -0.05, -0.05, 0.05]
# y = [0.05, -0.05, -0.05, 0.05]
# z = [-0.06, -0.07, -0.08, -0.10]
paused = False
t=0
# xf, xs = 0.02, 0.0
# yf, ys = 0.0, 0.0
# zs = -0.08
# h = 0.02
# Ts = 1.0
x_traj = []
y_traj = []
z_traj = []
Ts = 1.4
sensor_vel_x = []
sensor_vel_y = []
sensor_vel_z = []
def initialpos():
    x = [0.0, 0.0, 0.0, 0.0]
    y = [0.0, 0.0, 0.0, 0.0]
    z = [-0.1, -0.1, -0.1, -0.1]
    angles = ik(x,y,z)
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")

def plot_trajectory(x_traj, y_traj, z_traj, mode='3d'):
    if mode == '3d':
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_traj, y_traj, z_traj, label='Robot Trajectory')
        ax.set_xlabel('X (m)')
        #ax.set_xticks([0, 0.2, 0.4, 0.6, 0.8, 1])
        ax.set_ylabel('Y (m)')
        ax.set_yticks([-0.4, -0.2, 0, 0.2, 0.4])
        ax.set_zlabel('Z (m)')
        ax.set_zticks([0, 0.03, 0.06, 0.09, 0.12, 0.15])
    elif mode == '2d':
        plt.figure()
        plt.plot(x_traj, y_traj, label='Robot Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
    else:
        raise ValueError("Mode must be '2d' or '3d'")
    
    plt.legend()
    
    # 保存图像为文件
    output_path = f"robot_trajectory_{mode}.png"  # 根据模式命名文件
    plt.savefig(output_path)
    print(f"Trajectory plot saved as {output_path}")

def calculate_and_print_average_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z):
    avg_vel_x = np.mean(sensor_vel_x)/5
    avg_vel_y = np.mean(sensor_vel_y)
    avg_vel_z = np.mean(sensor_vel_z)
    global Ts
    print(f"T: {Ts}   Average Velocity X: {avg_vel_x:.4f} m/s")
    #print(f"Average Velocity Y: {avg_vel_y:.4f} m/s")
    #print(f"Average Velocity Z: {avg_vel_z:.4f} m/s")

def set_joint_angles_with_actuators(clock):


    global t
    x_target = 0.04
    z_target = -0.09
    xf, xs = 0.04, 0.0
    yf, ys = 0.0, 0.0
    zs = -0.100
    height = 0.02
    global Ts
    if clock > 10:
        x_traj.append(data.body("base_link").xpos[0])  # X 坐标
        y_traj.append(data.body("base_link").xpos[1])  # Y 坐标
        z_traj.append(data.body("base_link").xpos[2])  # Z 坐标

    if clock > 20 and clock < 50:


        velocimeter_start_index = model.sensor("Body_velo").adr
        sensor_vel_x.append(data.sensordata[velocimeter_start_index])
        sensor_vel_y.append(data.sensordata[velocimeter_start_index + 1])
        sensor_vel_z.append(data.sensordata[velocimeter_start_index + 2])

    #x,y,z = gait.walkt(t,x_target,z_target)
    #x,y,z = gait.trot(xf, xs, yf, ys, zs, height, t, Ts,0.0)
    #x,y,z = gait.walktest(xf, xs, yf, ys, zs, height, t, Ts)
    x,y,z = gait.walk_rotate(xf, xs, yf, ys, zs, height, t, Ts, 0)
    print(x,y,z)

    t = t + 0.01
    if t>=Ts:
        t=0

    print(f"t = {t}")
    angles = ik(x,y,z)
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
        # Set the joint angles via actuators each time step
        clock = clock + 0.01
        print (f"clock {clock}")
        if not paused:
            if clock > 3:
                set_joint_angles_with_actuators(clock)

            
            mujoco.mj_step(model, data)
            viewer.sync()

    plot_mode = '3d'
    plot_trajectory(x_traj, y_traj, z_traj, mode=plot_mode)
    calculate_and_print_average_velocity(sensor_vel_x, sensor_vel_y, sensor_vel_z)
    


if __name__ == "__main__":
    main()