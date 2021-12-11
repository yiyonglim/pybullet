import time
import numpy as np
import math
import sys

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11  # 8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
# upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
# joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
# restposes for null space
jointPositions = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions


class PandaSim(object):
    def __init__(self, bullet_client, offset, legos):
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)
        # print("offset=",offset)
        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.legochoice = legos
        self.legos = []
        self.tray = []
        self.temp = 0
        self.temptemp = 0

        # self.planeId = self.bullet_client.loadURDF("plane.urdf")
        self.bullet_client.loadURDF(
            "table/table.urdf", [0+offset[0], -0.6+offset[1], -0.9+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
        self.tray.append(bullet_client.loadURDF(
            "tray/traybox.urdf", [0.4+offset[0], 0.025+offset[1], -0.25+offset[2]], [-0.5, -0.5, -0.5, 0.5], globalScaling=0.25))
        self.tray.append(bullet_client.loadURDF(
            "tray/traybox.urdf", [0.4+offset[0], 0.025+offset[1], -0.45+offset[2]], [-0.5, -0.5, -0.5, 0.5], globalScaling=0.25))
        self.tray.append(bullet_client.loadURDF(
            "tray/traybox.urdf", [0.4+offset[0], 0.025+offset[1], -0.65+offset[2]], [-0.5, -0.5, -0.5, 0.5], globalScaling=0.25))
        self.bullet_client.changeVisualShape(
            self.tray[0], -1, rgbaColor=[1, 0, 0, 1])
        self.bullet_client.changeVisualShape(
            self.tray[1], -1, rgbaColor=[0, 1, 0, 1])
        self.bullet_client.changeVisualShape(
            self.tray[2], -1, rgbaColor=[0, 0, 1, 1])
        # dummy lego (4 colour segmentation)
        dummy_lego = self.bullet_client.loadURDF(
            "lego/lego.urdf", np.array([-0.1, 0.3, -1])+self.offset, globalScaling=1.5)
        self.bullet_client.changeVisualShape(
            dummy_lego, -1, rgbaColor=[1, 1, 1, 1])
        self.legos.append(self.bullet_client.loadURDF(
            "lego/lego.urdf", np.array([-0.1, 0.3, -0.5])+self.offset, globalScaling=1.5))
        self.legos.append(self.bullet_client.loadURDF(
            "lego/lego.urdf", np.array([-0.4, 0.3, -0.5])+self.offset, globalScaling=1.5))
        self.legos.append(self.bullet_client.loadURDF(
            "lego/lego.urdf", np.array([0.1, 0.3, -0.7])+self.offset, globalScaling=1.5))
        self.bullet_client.changeVisualShape(
            self.legos[0], -1, rgbaColor=[1, 0, 0, 1])
        self.bullet_client.changeVisualShape(
            self.legos[1], -1, rgbaColor=[0, 1, 0, 1])
        self.bullet_client.changeVisualShape(
            self.legos[2], -1, rgbaColor=[0, 0, 1, 1])
        # self.sphereId = self.bullet_client.loadURDF(
        #     "sphere_small.urdf", np.array([0, 0.3, -0.6])+self.offset, flags=flags)
        # self.bullet_client.loadURDF("sphere_small.urdf", np.array(
        #     [0, 0.3, -0.5])+self.offset, flags=flags)
        # self.bullet_client.loadURDF("sphere_small.urdf", np.array(
        #     [0, 0.3, -0.7])+self.offset, flags=flags)
        # p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
        orn = [-0.707107, 0.0, 0.0, 0.707107]
        eul = self.bullet_client.getEulerFromQuaternion(
            [-0.5, -0.5, -0.5, 0.5])
        # print("HELLO", eul)
        self.panda = self.bullet_client.loadURDF(
            "franka_panda/panda.urdf", np.array([0, 0, 0])+self.offset, orn, useFixedBase=True, flags=flags)
        index = 0
        self.state = 0
        self.control_dt = 1./120.
        self.finger_target = 0
        self.gripper_height = 0.2
        # create a constraint to keep the fingers centered
        c = self.bullet_client.createConstraint(self.panda,
                                                9,
                                                self.panda,
                                                10,
                                                jointType=self.bullet_client.JOINT_GEAR,
                                                jointAxis=[1, 0, 0],
                                                parentFramePosition=[
                                                    0, 0, 0],
                                                childFramePosition=[0, 0, 0])
        self.bullet_client.changeConstraint(
            c, gearRatio=-1, erp=0.1, maxForce=50)

        for j in range(self.bullet_client.getNumJoints(self.panda)):
            self.bullet_client.changeDynamics(
                self.panda, j, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.panda, j)
            # print(info)
            # file_path = r'C:\Users\yylim\OneDrive\Desktop\YEAR 3\COMP3003 - Individual Dissertation\resources\FYP simulation\jointinfo.txt'
            # sys.stdout = open(file_path, "w")
            # print("This text will be added to the file")
            # print("\n info = ", info)
            jointName = info[1]
            jointType = info[2]

            if (jointType == self.bullet_client.JOINT_PRISMATIC):

                self.bullet_client.resetJointState(
                    self.panda, j, jointPositions[index])
                index = index+1
            if (jointType == self.bullet_client.JOINT_REVOLUTE):
                self.bullet_client.resetJointState(
                    self.panda, j, jointPositions[index])
                index = index+1
        self.t = 0.

    def reset(self):
        pass

    def update_state(self):
        keys = self.bullet_client.getKeyboardEvents()
        if len(keys) > 0:
            for k, v in keys.items():
                if v & self.bullet_client.KEY_WAS_TRIGGERED:
                    if (k == ord('1')):
                        self.state = 1
                    if (k == ord('2')):
                        self.state = 2
                    if (k == ord('3')):
                        self.state = 3
                    if (k == ord('4')):
                        self.state = 4
                    if (k == ord('5')):
                        self.state = 5
                    if (k == ord('6')):
                        self.state = 6
                if v & self.bullet_client.KEY_WAS_RELEASED:
                    self.state = 0

    def step(self):
        if self.state == 6:
            self.finger_target = 0.01
        if self.state == 5:
            self.finger_target = 0.04
        self.bullet_client.submitProfileTiming("step")
        self.update_state()
        # print("self.state=",self.state)
        # print("self.finger_target=",self.finger_target)
        alpha = 0.9  # 0.99
        if self.state == 1 or self.state == 2 or self.state == 3 or self.state == 4 or self.state == 7:
            # gripper_height = 0.034
            self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.03
            if self.state == 2 or self.state == 3 or self.state == 7:
                self.gripper_height = alpha * \
                    self.gripper_height + (1.-alpha)*0.2

            t = self.t
            self.t += self.control_dt
            pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1] +
                   self.gripper_height, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
            if self.state == 3 or self.state == 4:
                # which colour should pick RGB
                pos, o = self.bullet_client.getBasePositionAndOrientation(
                    self.legos[self.legochoice[0]])
                pos = [pos[0], self.gripper_height+0.015, pos[2]]
                self.prev_pos = pos

            if self.state == 7:
                # pos = self.prev_pos
                # diffX = pos[0] - self.offset[0]
                # diffZ = pos[2] - (self.offset[2]-0.6)
                # self.prev_pos = [self.prev_pos[0] - diffX*0.1,
                #                  self.prev_pos[1], self.prev_pos[2]-diffZ*0.1]
                # # if self.temp
                # print(self.legos[self.legochoice[self.temp % 3]])
                if self.temp == 0:
                    self.temptemp = self.legochoice[0]
                    if len(self.legochoice) >= 0:
                        self.legochoice.remove(self.legochoice[0])
                        self.temp = 1

                if self.temptemp == 0:
                    pos, o = self.bullet_client.getBasePositionAndOrientation(
                        self.tray[0])
                if self.temptemp == 1:
                    pos, o = self.bullet_client.getBasePositionAndOrientation(
                        self.tray[1])
                if self.temptemp == 2:
                    pos, o = self.bullet_client.getBasePositionAndOrientation(
                        self.tray[2])

                # pos, o = self.bullet_client.getBasePositionAndOrientation(
                #     self.tray[0])
                pos = [pos[0], self.gripper_height, pos[2]]
                self.prev_pos = pos

            orn = self.bullet_client.getQuaternionFromEuler(
                [math.pi/2., 0., 0.])
            self.bullet_client.submitProfileTiming("IK")
            jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul,
                                                                       jr, rp, maxNumIterations=20)
            self.bullet_client.submitProfileTiming()
            for i in range(pandaNumDofs):
                self.bullet_client.setJointMotorControl2(
                    self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240., maxVelocity=1)
            # target for fingers
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(
                self.panda, i, self.bullet_client.POSITION_CONTROL, self.finger_target, force=10)
        self.bullet_client.submitProfileTiming()

    def jointinfo0(self):
        jointState = self.bullet_client.getJointState(self.panda, 0)
        # print(self.bullet_client.getJointState(self.panda, 0))
        velocity = "{:.8f}".format(jointState[1])
        return velocity


class PandaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset, choice):
        PandaSim.__init__(self, bullet_client, offset, choice)
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 3, 5, 4, 6, 3, 7, 5, 6]
        self.state_durations = [1, 1, 1, 1, 1, 1, 1, 1, 1]

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            self.state == 0
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
            if self.state == 0:
                self.temp = 0

    def humanintervention(self, choice):
        self.state_t = 0
        self.cur_state = 0
        self.legochoice = choice
