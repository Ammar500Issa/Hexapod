import pybullet as p
import pybullet_data
hexaid = p.connect(p.GUI)
p.setGravity(0, 0, -10, physicsClientId = hexaid)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
HexaPod = p.loadURDF("Hexapod.urdf", basePosition = [0, 0, 0.2])
Plane = p.loadURDF("Plane.urdf")
position, orientation = p.getBasePositionAndOrientation(HexaPod)
print(p.getEulerFromQuaternion(orientation))
print(position, orientation)
number_of_joints = p.getNumJoints(HexaPod)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(HexaPod, joint_number)
    print(info[0], ": ", info[1])
_link_name_to_index = {p.getBodyInfo(HexaPod)[0].decode('UTF-8'):-1,}  
for _id in range(number_of_joints):
    _name = p.getJointInfo(HexaPod, _id)[12].decode('UTF-8')
    _link_name_to_index[_name] = _id
    print(_id, ": ", _name)
# Defining the indices of each leg
leg1_indices = [1, 2, 4]
leg2_indices = [7, 8, 10]
leg3_indices = [13, 14, 16]
leg4_indices = [19, 20, 22]
leg5_indices = [25, 26, 28]
leg6_indices = [31, 32, 34]
# Manually testing of the Position control of each motor
angle_1 = p.addUserDebugParameter('Steering1_1', -1.5, 1.5, 0)
angle_2 = p.addUserDebugParameter('Steering1_2', -1.5, 1.5, 0)
angle_3 = p.addUserDebugParameter('Steering1_3', -1.5, 1.5, 0)
angle_4 = p.addUserDebugParameter('Steering2_1', -1.5, 1.5, 0)
angle_5 = p.addUserDebugParameter('Steering2_2', -1.5, 1.5, 0)
angle_6 = p.addUserDebugParameter('Steering2_3', -1.5, 1.5, 0)
angle_7 = p.addUserDebugParameter('Steering3_1', -1.5, 1.5, 0)
angle_8 = p.addUserDebugParameter('Steering3_2', -1.5, 1.5, 0)
angle_9 = p.addUserDebugParameter('Steering3_3', -1.5, 1.5, 0)
angle_10 = p.addUserDebugParameter('Steering4_1', -1.5, 1.5, 0)
angle_11 = p.addUserDebugParameter('Steering4_2', -1.5, 1.5, 0)
angle_12 = p.addUserDebugParameter('Steering4_3', -1.5, 1.5, 0)
angle_13 = p.addUserDebugParameter('Steering5_1', -1.5, 1.5, 0)
angle_14 = p.addUserDebugParameter('Steering5_2', -1.5, 1.5, 0)
angle_15 = p.addUserDebugParameter('Steering5_3', -1.5, 1.5, 0)
angle_16 = p.addUserDebugParameter('Steering6_1', -1.5, 1.5, 0)
angle_17 = p.addUserDebugParameter('Steering6_2', -1.5, 1.5, 0)
angle_18 = p.addUserDebugParameter('Steering6_3', -1.5, 1.5, 0)
"""
throttle_1 = p.addUserDebugParameter('Throttle1_1', -6, 6, 0)
throttle_2 = p.addUserDebugParameter('Throttle1_2', -6, 6, 0)
throttle_3 = p.addUserDebugParameter('Throttle1_3', -6, 6, 0)
throttle_4 = p.addUserDebugParameter('Throttle2_1', -6, 6, 0)
throttle_5 = p.addUserDebugParameter('Throttle2_2', -6, 6, 0)
throttle_6 = p.addUserDebugParameter('Throttle2_3', -6, 6, 0)
throttle_7 = p.addUserDebugParameter('Throttle3_1', -6, 6, 0)
throttle_8 = p.addUserDebugParameter('Throttle3_2', -6, 6, 0)
throttle_9 = p.addUserDebugParameter('Throttle3_3', -6, 6, 0)
throttle_10 = p.addUserDebugParameter('Throttle4_1', -6, 6, 0)
throttle_11 = p.addUserDebugParameter('Throttle4_2', -6, 6, 0)
throttle_12 = p.addUserDebugParameter('Throttle4_3', -6, 6, 0)
throttle_13 = p.addUserDebugParameter('Throttle5_1', -6, 6, 0)
throttle_14 = p.addUserDebugParameter('Throttle5_2', -6, 6, 0)
throttle_15 = p.addUserDebugParameter('Throttle5_3', -6, 6, 0)
throttle_16 = p.addUserDebugParameter('Throttle6_1', -6, 6, 0)
throttle_17 = p.addUserDebugParameter('Throttle6_2', -6, 6, 0)
throttle_18 = p.addUserDebugParameter('Throttle6_3', -6, 6, 0)
"""
while(1): 
    user_angle_1 = p.readUserDebugParameter(angle_1)
    user_angle_2 = p.readUserDebugParameter(angle_2)
    user_angle_3 = p.readUserDebugParameter(angle_3)
    p.setJointMotorControl2(HexaPod, leg1_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_1)
    p.setJointMotorControl2(HexaPod, leg1_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_2)
    p.setJointMotorControl2(HexaPod, leg1_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_3)
    user_angle_4 = p.readUserDebugParameter(angle_4)
    user_angle_5 = p.readUserDebugParameter(angle_5)
    user_angle_6 = p.readUserDebugParameter(angle_6)
    p.setJointMotorControl2(HexaPod, leg2_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_4)
    p.setJointMotorControl2(HexaPod, leg2_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_5)
    p.setJointMotorControl2(HexaPod, leg2_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_6)
    user_angle_7 = p.readUserDebugParameter(angle_7)
    user_angle_8 = p.readUserDebugParameter(angle_8)
    user_angle_9 = p.readUserDebugParameter(angle_9)
    p.setJointMotorControl2(HexaPod, leg3_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_7)
    p.setJointMotorControl2(HexaPod, leg3_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_8)
    p.setJointMotorControl2(HexaPod, leg3_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_9)
    user_angle_10 = p.readUserDebugParameter(angle_10)
    user_angle_11 = p.readUserDebugParameter(angle_11)
    user_angle_12 = p.readUserDebugParameter(angle_12)
    p.setJointMotorControl2(HexaPod, leg4_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_10)
    p.setJointMotorControl2(HexaPod, leg4_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_11)
    p.setJointMotorControl2(HexaPod, leg4_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_12)
    user_angle_13 = p.readUserDebugParameter(angle_13)
    user_angle_14 = p.readUserDebugParameter(angle_14)
    user_angle_15 = p.readUserDebugParameter(angle_15)
    p.setJointMotorControl2(HexaPod, leg5_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_13)
    p.setJointMotorControl2(HexaPod, leg5_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_14)
    p.setJointMotorControl2(HexaPod, leg5_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_15)
    user_angle_16 = p.readUserDebugParameter(angle_16)
    user_angle_17 = p.readUserDebugParameter(angle_17)
    user_angle_18 = p.readUserDebugParameter(angle_18)
    p.setJointMotorControl2(HexaPod, leg6_indices[0],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_16)
    p.setJointMotorControl2(HexaPod, leg6_indices[1],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_17)
    p.setJointMotorControl2(HexaPod, leg6_indices[2],
                                p.POSITION_CONTROL,
                                targetPosition = user_angle_18)
    """
    user_throttle_1 = p.readUserDebugParameter(throttle_1)
    user_throttle_2 = p.readUserDebugParameter(throttle_2)
    user_throttle_3 = p.readUserDebugParameter(throttle_3)
    p.setJointMotorControl2(HexaPod, leg1_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_1)
    p.setJointMotorControl2(HexaPod, leg1_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_2)
    p.setJointMotorControl2(HexaPod, leg1_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_3)
    user_throttle_4 = p.readUserDebugParameter(throttle_4)
    user_throttle_5 = p.readUserDebugParameter(throttle_5)
    user_throttle_6 = p.readUserDebugParameter(throttle_6)
    p.setJointMotorControl2(HexaPod, leg2_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_4)
    p.setJointMotorControl2(HexaPod, leg2_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_5)
    p.setJointMotorControl2(HexaPod, leg2_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_6)
    user_throttle_7 = p.readUserDebugParameter(throttle_7)
    user_throttle_8 = p.readUserDebugParameter(throttle_8)
    user_throttle_9 = p.readUserDebugParameter(throttle_9)
    p.setJointMotorControl2(HexaPod, leg3_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_7)
    p.setJointMotorControl2(HexaPod, leg3_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_8)
    p.setJointMotorControl2(HexaPod, leg3_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_9)
    user_throttle_10 = p.readUserDebugParameter(throttle_10)
    user_throttle_11 = p.readUserDebugParameter(throttle_11)
    user_throttle_12 = p.readUserDebugParameter(throttle_12)
    p.setJointMotorControl2(HexaPod, leg4_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_10)
    p.setJointMotorControl2(HexaPod, leg4_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_11)
    p.setJointMotorControl2(HexaPod, leg4_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_12)
    user_throttle_13 = p.readUserDebugParameter(throttle_13)
    user_throttle_14 = p.readUserDebugParameter(throttle_14)
    user_throttle_15 = p.readUserDebugParameter(throttle_15)
    p.setJointMotorControl2(HexaPod, leg5_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_13)
    p.setJointMotorControl2(HexaPod, leg5_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_14)
    p.setJointMotorControl2(HexaPod, leg5_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_15)
    user_throttle_16 = p.readUserDebugParameter(throttle_16)
    user_throttle_17 = p.readUserDebugParameter(throttle_17)
    user_throttle_18 = p.readUserDebugParameter(throttle_18)
    p.setJointMotorControl2(HexaPod, leg6_indices[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_16)
    p.setJointMotorControl2(HexaPod, leg6_indices[1],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_17)
    p.setJointMotorControl2(HexaPod, leg6_indices[2],
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle_18)
    """
    p.stepSimulation()
    p.setRealTimeSimulation(0)
