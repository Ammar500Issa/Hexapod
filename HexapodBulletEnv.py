import math
import numpy as np
import pybullet as p
from gym import Env
from gym import spaces
from time import sleep
from pybullet_data import getDataPath

INIT_POSITION = [0, 0, 0.2]
INIT_ORIENTATION = [0, 0, 0, 1]
TARGET_POSITION = [-1, 1, 0.1]

class HexapodBulletEnv(Env):
    def __init__(self, client, time_step = 0.05, frameskip = 12, render = False,  max_velocity = 59*2*math.pi/60, max_torque = 1.50041745):
        super().__init__()
        self._render = render
        if self._render:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.client = client
        self.num_motors = 18
        self.num_legs = self.num_motors / 3
        self.n_pos_actions = 18
        self.action_pos_space = spaces.Box(low = -math.pi/6, high = math.pi/6, shape = (self.n_pos_actions,), dtype = "float64")
        self.n_vel_actions = 18
        self.action_vel_space = spaces.Box(low = -6, high = 6, shape = (self.n_vel_actions,), dtype = "float64")
        self.action_space = self.action_pos_space
        self.n_observations = 18*3 + 6 + 3
        self.observation_space = spaces.Box(low = -1, high = 1, shape = (self.n_observations,), dtype = "float64")
        self.observation = np.zeros(self.n_observations, dtype = "float64")
        self.dt = time_step
        self.frameskip = frameskip
        self.servo_max_speed = max_velocity
        self.servo_max_torque = max_torque
        self.target_position = np.array(TARGET_POSITION)
        self.seed()
        p.setTimeStep(time_step / frameskip)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81, physicsClientId = client)
        p.setAdditionalSearchPath(getDataPath())
        p.setRealTimeSimulation(0)
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
        self.Hexapod = p.loadURDF("Hexapod.urdf", basePosition = INIT_POSITION, flags = flags)
        self.Plane = p.loadURDF("Plane.urdf")
        self.joint_list = [j for j in range(p.getNumJoints(self.Hexapod)) if p.getJointInfo(self.Hexapod, j)[2] == p.JOINT_REVOLUTE]

    @staticmethod
    def seed(seed = None):
        seed = np.random.seed(seed)
        return [seed]

    def _update_observation(self):
        all_states = p.getJointStates(self.Hexapod, self.joint_list)
        for i, (pos, vel, _, tor) in enumerate(all_states):
            self.observation[3*i:3*i + 3] = [pos * 2 / math.pi, np.clip(vel / self.servo_max_speed, -1, 1), np.clip(tor / self.servo_max_torque, -1, 1)]
        pos, ori = p.getBasePositionAndOrientation(self.Hexapod)
        self.observation[-9:-3] = list(pos) + list(p.getEulerFromQuaternion(ori))
        self.observation[-6:-3] /= math.pi
        for i in range(0, len(self.observation)):
            self.observation[i] += (np.random.standard_normal() / 10)

    def reset(self):
        p.resetBasePositionAndOrientation(self.Hexapod, INIT_POSITION, INIT_ORIENTATION)
        for j in self.joint_list:
            p.resetJointState(self.Hexapod, j, np.random.uniform(low = -math.pi / 18, high = math.pi / 18))
        self.target_position = np.array(TARGET_POSITION)
        self.observation[-3:] = self.target_position
        p.removeAllUserDebugItems()
        p.addUserDebugLine(self.target_position - [0.01, 0.01, 0.01], self.target_position + [0.01, 0.01, 0.01], [0, 0, 0], 2)
        p.setGravity(0, 0, -9.81, physicsClientId = self.client)
        self._update_observation()
        return self.observation

    def _get_reward(self):
        w1 = 20
        w2 = 0.2
        w3 = 0.2
        w4 = 0.2
        position, orientation = p.getBasePositionAndOrientation(self.Hexapod)
        ori = p.getEulerFromQuaternion(orientation)
        speeds = self.observation[1:-6:3]
        torques = self.observation[2:-6:3]
        consuption = self.dt * abs(sum(speeds * torques))
        reward = w1 * (math.sqrt(TARGET_POSITION[0] ** 2 + TARGET_POSITION[1] ** 2) - math.sqrt((TARGET_POSITION[0] - position[0]) ** 2 + (position[1] - TARGET_POSITION[1]) ** 2)) - w2 * (abs(position[2] - TARGET_POSITION[2])) - w3 * consuption - w4 * (abs(ori[0]) + abs(ori[1]))
        reward *= 0.01
        return reward

    def step(self, action):
        transformed_pos = np.array(action)
        for i in range(0, len(transformed_pos)):
            transformed_pos[i] += (np.random.standard_normal() / 100)
        """
        transformed_vel = np.array(action)
        for i in range(0, len(transformed_vel)):
            transformed_vel[i] += (np.random.standard_normal() / 100)
        """
        max_torques = [self.servo_max_torque - abs(np.random.standard_normal() / 10)] * 18
        """
        for i in range(len(self.joint_list)):
            p.setJointMotorControl2(bodyUniqueId = self.Hexapod, 
                                    jointIndex = self.joint_list[i], 
                                    controlMode = p.POSITION_CONTROL, 
                                    targetPosition = transformed_pos[i],
                                    force = max_torques[i],
                                    maxVelocity = self.servo_max_speed)
        """
        p.setJointMotorControlArray(bodyIndex = self.Hexapod, 
                                    jointIndices = self.joint_list, 
                                    controlMode = p.POSITION_CONTROL, 
                                    targetPositions = transformed_pos, 
                                    forces = max_torques)
        """
        p.setJointMotorControlArray(bodyIndex = self.Hexapod, 
                                    jointIndices = self.joint_list, 
                                    controlMode = p.VELOCITY_CONTROL, 
                                    targetVelocities = transformed_vel, 
                                    forces = max_torques)
        """
        for _ in range(self.frameskip):
            p.stepSimulation()
            sleep(self.dt / self.frameskip)
        self._update_observation()
        reward = self._get_reward()
        position, orientation = p.getBasePositionAndOrientation(self.Hexapod)
        ori = p.getEulerFromQuaternion(orientation)
        done = bool(position[2] < 0.03) | bool(ori[1] > math.pi/6) | bool(ori[1] < -math.pi/6) | bool(ori[0] > math.pi/6) | bool(ori[0] < -math.pi/6) | bool(math.sqrt((TARGET_POSITION[0] - position[0]) ** 2 + (position[1] - TARGET_POSITION[1]) ** 2) > 2 * math.sqrt(TARGET_POSITION[0] ** 2 + TARGET_POSITION[1] ** 2))
        return self.observation, reward, done, {}

    def render(self, mode = 'human'):
        if mode != "rgb_array":
            return np.array([])
        position = p.getBasePositionAndOrientation(self.Hexapod)[0]
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition = position, distance = 0.01, yaw = 30, pitch = -30, roll = 0, upAxisIndex = 2)
        proj_matrix = p.computeProjectionMatrixFOV(fov = 60, aspect = 960./720, nearVal = 0.1, farVal = 100.0)
        px = p.getCameraImage(width = 960, height = 720, viewMatrix = view_matrix, projectionMatrix = proj_matrix, renderer = p.ER_TINY_RENDERER)[2]
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        pass

    def apply_action(self, action):
        pass

    def get_observation(self):
        pass
