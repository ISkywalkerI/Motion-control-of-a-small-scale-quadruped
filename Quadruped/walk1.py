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
t=0
# xf, xs = 0.02, 0.0
# yf, ys = 0.0, 0.0
# zs = -0.08
# h = 0.02
# Ts = 1.0
x_traj = []
y_traj = []
z_traj = []
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
    
    # 保存图像为文件
    output_path = f"robot_trajectory_{mode}.png"  # 根据模式命名文件
    plt.savefig(output_path)
    print(f"Trajectory plot saved as {output_path}")

def set_joint_angles_with_actuators():


    global t
    x_target = 0.04
    z_target = -0.09
    xf, xs = 0.04, 0.0
    yf, ys = 0.0, 0.0
    zs = -0.10
    height = 0.03
    Ts = 3

    x_traj.append(data.body("base_link").xpos[0])  # X 坐标
    y_traj.append(data.body("base_link").xpos[1])  # Y 坐标
    z_traj.append(data.body("base_link").xpos[2])  # Z 坐标

    #x,y,z = gait.walkt(t,x_target,z_target)
    #x,y,z = gait.trot(xf, xs, yf, ys, zs, height, t, Ts,0.3)
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

def main():
    viewer = mujoco.viewer.launch_passive(model=model, data=data)
    
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
        if clock > 5:
            set_joint_angles_with_actuators()
            print(f"t = {t}")
        mujoco.mj_step(model, data)
        viewer.sync()

    plot_mode = '2d'
    plot_trajectory(x_traj, y_traj, z_traj, mode=plot_mode)

    output_path = "robot_trajectory.png"  # 保存路径
    plt.savefig(output_path)


if __name__ == "__main__":
    main()