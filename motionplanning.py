import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import motionplanningpanda as panda_sim
import threading
import sys


p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)

# camera position (x,y,z)
camTargetPos = [0, -3, -0.5]
# camera left (neagtive) / right (positive)
yaw = 0
# camera up (positive) / down (negative)
pitch = -180
# camera zoom in / out
roll = 0
# camera placement on axis (Y = 1 , Z = 2)
upAxisIndex = 2
# camera far / close
camDistance = 4
# camera window size (pixel x pixel , hd......)
pixelWidth = 1080
pixelHeight = 720
# near and far plane
nearPlane = 0.05
farPlane = 1
lightDirection = [0, 1, 0]
lightColor = [1, 1, 1]  # optional argument
fov = 50

# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=38,
                             cameraPitch=-22, cameraTargetPosition=[0.35, -0.13, 0])

p.setAdditionalSearchPath(pd.getDataPath())

timeStep = 1./240.  # 240.
p.setTimeStep(timeStep)
p.setRealTimeSimulation(1)

p.setGravity(0, -9.8, 0)

panda = panda_sim.PandaSimAuto(p, [0, 0, 0], [0, 1, 2])
logId = panda.bullet_client.startStateLogging(
    panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
panda.bullet_client.submitProfileTiming("start")

temp = 0

time.sleep(1)
if temp == 0:
    viewMatrix = p.computeViewMatrixFromYawPitchRoll(
        camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov, aspect, nearPlane, farPlane)
    img_arr = p.getCameraImage(
        pixelWidth, pixelHeight, viewMatrix, projectionMatrix, lightDirection, lightColor)
    print(img_arr)
    temp = 1

while True:

    panda.bullet_client.submitProfileTiming("full_step")
    panda.step()
    # print(panda.jointinfo0())
    limit = panda.jointinfo0()
    # print(limit)
    if float(limit) > 1.001 or float(limit) < -1.001:
        # panda.legochoice = [2, 1, 0]
        panda.humanintervention([2, 0, 1])
    p.stepSimulation()
    time.sleep(timeStep)
    panda.bullet_client.submitProfileTiming()

    # 1080 * 720
    # print("hello", len(img_arr[3]))

    # render_image = render_image + 1
# panda.bullet_client.submitProfileTiming()
# panda.bullet_client.stopStateLogging(logId)


# import pybullet as p
# import pybullet_data as pd
# import os
# import math
# import time
# import numpy as np
# import motionplanningpanda as panda_sim

# p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)
# # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
# p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=38,
#                              cameraPitch=-22, cameraTargetPosition=[0.35, 0.5, 0])
# # p.setAdditionalSearchPath(pd.getDataPath())

# urdfRootPath = pd.getDataPath()

# pandaNumDofs = 7
# ll = [-7]*pandaNumDofs
# # upper limits for null space (todo: set them to proper range)
# ul = [7]*pandaNumDofs
# # joint ranges for null space (todo: set them to proper range)
# jr = [7]*pandaNumDofs
# # restposes for null space
# jointPositions = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
# rp = jointPositions
# # load Franka Emika Panda robotic arm
# panda = p.loadURDF(os.path.join(
#     urdfRootPath, "franka_panda/panda.urdf"), [0, 0, 0], [-0.707107, 0.0, 0.0, 0.707107], useFixedBase=True)

# table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),
#                    [0.6, -0.6, 0], [-0.5, -0.5, -0.5, 0.5])


# lego = p.loadURDF(os.path.join(urdfRootPath, "lego/lego.urdf"),
#                   [0.5, 0.3, -0.5], globalScaling=3)


# timeStep = 1./240.  # 240.
# # p.setTimeStep(timeStep)
# p.setRealTimeSimulation(1)
# p.setGravity(0, -9.8, 0)

# while True:

#     orn = p.getQuaternionFromEuler(
#     [math.pi/2., 0., 0.])
# pos, o = p.getBasePositionAndOrientation(
#     lego)
# pos = [pos[0], 0.034, pos[2]]
# jointPoses = p.calculateInverseKinematics(panda, 11, pos, orn, ll, ul,
#                                           jr, rp, maxNumIterations=20)
# # p.submitProfileTiming()
# print(jointPoses)
# for i in range(pandaNumDofs):
#     p.setJointMotorControl2(
#         panda, i, p.POSITION_CONTROL, jointPoses[i], force=5 * 240.)

# for i in [9, 10]:
#     p.setJointMotorControl2(
#         panda, i, p.POSITION_CONTROL, 0.04, force=10)
# p.submitProfileTiming()

# print(p.getBasePositionAndOrientation(lego))
# # p.stepSimulation()
