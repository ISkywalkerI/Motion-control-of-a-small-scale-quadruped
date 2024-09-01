import mujoco
import numpy as np
import mujoco.viewer
import time
from IK import ik
from scipy.spatial.transform import Rotation as R
import math
import matplotlib.pyplot as plt

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

Kp = 0.5
Ki = 0.05
Kd = 0.01

integral_pitch = 0.0
integral_roll = 0.0
previous_error_pitch = 0.0
previous_error_roll = 0.0
t=0
# x = [0.0, 0.0, 0.0, 0.0]
# y = [0.0, 0.0, 0.0, 0.0]
# z = [-0.09, -0.09, -0.09, -0.09]
# Lists to store pitch and roll values over time
time_steps = []
pitch_values = []
roll_values = []
Roll= 0
Pitch = 0

def pid_control(target, current, integral, previous_error):
    error = target - current
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output*0.05, integral, error

def extract_pitch_roll_from_quat(quat):
    r = R.from_quat(quat)
    euler = r.as_euler('xyz', degrees=False)
    # euler[0] = euler[0] + math.p
    return euler[0], euler[1], euler[2]  # Pitch, Roll (assuming 'xyz' order)

def apply_dead_zone(value, threshold):
    if abs(value) < threshold:
        return 0
    return value


def adjust_leg_positions_for_balance(current_time):
    # Extract the quaternion representing the body's orientation
    global integral_pitch, integral_roll, previous_error_pitch, previous_error_roll,t
    quat = data.sensordata[:4]  # Assuming Body_Quat starts at index 0
    print (f"quat = {quat}")
    pitch, roll = extract_pitch_roll_from_quat(quat)


    # Compute PID control for pitch and roll
    base_z = [-0.09, -0.1, -0.09, -0.1]

    time_steps.append(current_time)

    pitch_values.append(pitch)
    roll_values.append(roll)
    pitch_adjustment, integral_pitch, previous_error_pitch = pid_control(0, pitch, integral_pitch, previous_error_pitch)
    roll_adjustment, integral_roll, previous_error_roll = pid_control(0, roll, integral_roll, previous_error_roll)
    print(f"pitch = {pitch} roll = {roll} pitch_adjustment = {pitch_adjustment} roll_adjustment = {roll_adjustment}" )
    # Adjust z coordinates of the legs based on pitch and roll adjustments

    pitch_adjustment = apply_dead_zone(pitch_adjustment, 0.0005)
    roll_adjustment = apply_dead_zone(roll_adjustment, 0.0005)

    # z[0] = z[0] - pitch_adjustment  # Front-left leg
    # z[1] = z[1] - pitch_adjustment  # Front-right leg
    # z[2] = z[2] + pitch_adjustment  # Back-left leg
    # z[3] = z[3] + pitch_adjustment  # Back-right leg

    # z[0] = base_z[0] - pitch_adjustment + roll_adjustment  # Front-left leg
    # z[1] = base_z[1] - pitch_adjustment - roll_adjustment  # Front-right leg
    # z[2] = base_z[2] + pitch_adjustment + roll_adjustment  # Back-left leg
    # z[3] = base_z[3] + pitch_adjustment - roll_adjustment  # Back-right leg

    print(f"z = {z}")


def rotation_matrix(R, P, Y):
    # 定义旋转矩阵的余弦和正弦值
    cR, sR = np.cos(R), np.sin(R)
    cP, sP = np.cos(P), np.sin(P)
    cY, sY = np.cos(Y), np.sin(Y)

    # 定义每个旋转矩阵
    rot_x = np.array([
        [1, 0, 0],
        [0, cR, -sR],
        [0, sR, cR]
    ])
    
    rot_y = np.array([
        [cP, 0, sP],
        [0, 1, 0],
        [-sP, 0, cP]
    ])

    rot_z = np.array([
        [cY, -sY, 0],
        [sY, cY, 0],
        [0, 0, 1]
    ])
    
    
    
    # 进行矩阵乘法
    #A = np.dot(rot_x, np.dot(rot_y, rot_z))
    A = rot_x @ rot_y @ rot_z
    print(f"A = {A}")
    return A

def site_pos (sitename):
    siteid = model.site(sitename).id
    posworld = data.site_xpos[siteid]


    print(f"{sitename} posworld is {posworld}" )
    return posworld

def balance():
    global Roll,Pitch
    quat = data.sensordata[:4]  # Assuming Body_Quat starts at index 0
    yaw, pitch, roll = extract_pitch_roll_from_quat(quat)
    print(f"pitch = {pitch} roll = {roll} yall = {yaw}")

    Roll = Roll - 0.2 * roll
    Pitch = Pitch - 0.2 * pitch 

    A = rotation_matrix(Roll, Pitch, 0)
    A_inv = np.linalg.inv(A)
    OQ4 = np.array([-0.09775,-0.03585,0])
    OQ3 = np.array([-0.09775,0.03585,0])
    OQ2 = np.array([0.04475,-0.03585,0])
    OQ1 = np.array([0.04475,0.03585,0])
    OQ11 = np.dot(A_inv, OQ1)
    OQ22 = np.dot(A_inv, OQ2)
    OQ33 = np.dot(A_inv, OQ3)
    OQ44 = np.dot(A_inv, OQ4)
    O = site_pos("imu")
    P1 = site_pos("Lf3_foot")
    P2 = site_pos("Rf3_foot")
    P3 = site_pos("Lb3_foot")
    P4 = site_pos("Rb3_foot")
    OP1 = P1 - O
    OP2 = P2 - O
    OP3 = P3 - O
    OP4 = P4 - O
    QP1 = OP1 - OQ11
    QP2 = OP2 - OQ22
    QP3 = OP3 - OQ33
    QP4 = OP4 - OQ44
    QP11 = np.dot(A, QP1)
    QP22 = np.dot(A, QP2)
    QP33 = np.dot(A, QP3)
    QP44 = np.dot(A, QP4)
    x = [QP11[0], QP22[0], QP33[0], QP44[0]]
    y = [QP11[1], QP22[1], QP33[1], QP44[1]]
    z = [QP11[2], QP22[2], QP33[2], QP44[2]]

    print(f"QP1 = {QP11} QP = {QP1} ")
    return x,y,z




def save_pitch_roll_plot():
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, pitch_values, label="Pitch", color="blue")
    plt.plot(time_steps, roll_values, label="Roll", color="red")
    plt.title("Pitch and Roll Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (radians)")
    plt.legend()
    plt.grid(True)
    plt.savefig("pitch_roll_plot.png")
    print("Pitch and Roll plot saved as 'pitch_roll_plot.png'")

def set_joint_angles_with_actuators():
    # Define desired joint angles in radians
    # desired_angles = {
    #     "lf1": 0.5,  # Example angles
    #     "lf2": -0.3,
    #     "lf3": 0.2,
    #     "rf1": -0.5,
    #     "rf2": 0.3,
    #     "rf3": -0.2,
    #     "lb1": 0.4,
    #     "lb2": -0.4,
    #     "lb3": 0.1,
    #     "rb1": -0.1,
    #     "rb2": 0.6,
    #     "rb3": -0.6,
    # }
    x,y,z = balance()
    print(f"xyz = {x,y,z}")
    angles = ik(x,y,z)
    for actuator_name, angle in angles.items():
        actuator_id = model.actuator(actuator_name).id
        data.ctrl[actuator_id] = angle
        print(f"Setting {actuator_name} to {angle} radians (ctrl: {data.ctrl[actuator_id]})")

    # Check if the qpos changes accordingly
    #print(f"Current qpos: {data.qpos}")

def main():
    viewer = mujoco.viewer.launch_passive(model=model, data=data)
    
    if viewer is None:
        print("Failed to launch viewer.")
        return
    
    mujoco.mj_resetData(model, data)
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    current_time = 0.0
    clock = 0   
    while viewer.is_running():
        # Set the joint angles via actuators each time step
        clock = clock + 0.01
        print (f"clock {clock}")
        if clock > 3:

            #adjust_leg_positions_for_balance(current_time)
           #site_pos(sitename="imu")

            set_joint_angles_with_actuators()
           
            current_time += dt
            
        mujoco.mj_step(model, data)
        viewer.sync()

    save_pitch_roll_plot()

if __name__ == "__main__":
    main()