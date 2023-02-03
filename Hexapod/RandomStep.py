import math
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from time import sleep

INIT_POSITION = [0, 0, 0.2]
INIT_ORIENTATION = [0, 0, 0, 1]
TARGET_POSITION = [-1, 1, 0.1]

def seed():
    np.random.seed(None)

time_step = 0.05
frameskip = 12
max_velocity = 59*2*math.pi/60
max_torque = 1.50041745
client = p.connect(p.GUI)
num_motors = 18
num_legs = num_motors / 3
n_pos_actions = num_motors
n_vel_actions = num_motors
action_pos_space = spaces.Box(low = -1.5, high = 1.5, shape = (n_pos_actions,), dtype = "float32")
action_vel_space = spaces.Box(low = -6, high = 6, shape = (n_vel_actions,), dtype = "float32")
n_observations = num_motors*3 + 6 + 3
observation_space = spaces.Box(low = -1, high = 1, shape = (n_observations,), dtype = "float32")
observation = np.zeros(n_observations, dtype = "float32")
dt = time_step
servo_max_speed = max_velocity
servo_max_torque = max_torque
target_position = np.array([0, 0, 0.2])
last_target_distance = 0
seed()
p.setTimeStep(time_step / frameskip)
p.resetSimulation()
p.setGravity(0, 0, -9.81, physicsClientId = client)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.stepSimulation()
p.setRealTimeSimulation(0)
Hexapod = p.loadURDF("Hexapod.urdf", basePosition = [0, 0, 0.2])
Plane = p.loadURDF("Plane.urdf")
joint_list = [j for j in range(p.getNumJoints(Hexapod)) if p.getJointInfo(Hexapod, j)[2] == p.JOINT_REVOLUTE]

def update_observation():
    all_states = p.getJointStates(Hexapod, joint_list)
    for i, (pos, vel, _, tor) in enumerate(all_states):
        observation[3*i:3*i + 3] = [pos * 2 / math.pi, np.clip(vel / servo_max_speed, -1, 1), np.clip(tor / servo_max_torque, -1, 1)]
    pos, ori = p.getBasePositionAndOrientation(Hexapod)
    observation[-9:-3] = list(pos) + list(p.getEulerFromQuaternion(ori))
    observation[-6:-3] /= math.pi

def reset():
    p.resetBasePositionAndOrientation(Hexapod, [0, 0, 0.2], [0, 0, 0, 1])
    for j in joint_list:
        p.resetJointState(Hexapod, j, np.random.uniform(low = -math.pi/4, high = math.pi/4))
    target_position = np.array([0, 0, 0.2])
    observation[-3:] = target_position
    p.removeAllUserDebugItems()
    p.addUserDebugLine(target_position - [0.01, 0.01, 0.01], target_position + [0.01, 0.01, 0.01], [0, 0, 0], 2)
    p.setGravity(0, 0, -9.81, physicsClientId = client)
    position, _ = p.getBasePositionAndOrientation(Hexapod)
    update_observation()
    return observation

def get_reward():
    w1 = 20
    w2 = 0.2
    w3 = 0.2
    w4 = 0.2
    position, orientation = p.getBasePositionAndOrientation(Hexapod)
    ori = p.getEulerFromQuaternion(orientation)
    speeds = observation[1:-6:3]
    torques = observation[2:-6:3]
    consuption = dt * abs(sum(speeds * torques))
    reward = w1 * (math.sqrt(TARGET_POSITION[0] ** 2 + TARGET_POSITION[1] ** 2) - math.sqrt((TARGET_POSITION[0] - position[0]) ** 2 + (position[1] - TARGET_POSITION[1]) ** 2)) - w2 * (abs(position[2] - TARGET_POSITION[2])) - w3 * consuption - w4 * (abs(ori[0]) + abs(ori[1]))
    reward *= 0.01
    return reward

def step(pos):
    transformed_pos = np.array(pos) * math.pi/2
    max_torques = [servo_max_torque] * 18
    p.setJointMotorControlArray(bodyIndex = Hexapod, jointIndices = joint_list, controlMode = p.POSITION_CONTROL, targetPositions = transformed_pos, forces = max_torques)
    for _ in range(frameskip):
        p.stepSimulation()
        sleep(dt / frameskip)
    update_observation()
    reward = get_reward()
    position, _ = p.getBasePositionAndOrientation(Hexapod)
    done = bool(position[2] < 0.08)
    return observation, reward, done, {}

def render(mode = 'human'):
    if mode != "rgb_array":
        return np.array([])
    position = p.getBasePositionAndOrientation(Hexapod)[0]
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition = position, distance = 0.6, yaw = 30, pitch = -30, roll = 0, upAxisIndex = 2)
    proj_matrix = p.computeProjectionMatrixFOV(fov = 60, aspect = 960./720, nearVal = 0.1, farVal = 100.0)
    px = p.getCameraImage(width = 960, height = 720, viewMatrix = view_matrix, projectionMatrix = proj_matrix, renderer = p.ER_TINY_RENDERER)[2]
    rgb_array = np.array(px)
    rgb_array = rgb_array[:, :, :3]
    return rgb_array

reset()
while(1):
    render()
    observation, reward, done, _ = step(action_pos_space.sample())
    #print(observation, reward, done)
    p.stepSimulation()
    p.setRealTimeSimulation(0)