#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematicsnew
from constants import *


class Parameters:
    def __init__(
        self, z=-60,
    ):
        self.z= z
        self.legAngles = LEG_ANGLES
        self.initLeg = []
        self.initLeg.append([0.170,0])
        self.initLeg.append([0.170,0]) 
        self.initLeg.append([0.170,0]) 
        self.initLeg.append([0.170,0]) 
        self.initLeg.append([0.170,0]) 
        self.initLeg.append([0.170,0]) 

        self.legs =  {}
        self.legs[1] = ["j_c1_rf","j_thigh_rf","j_tibia_rf"]
        self.legs[6] = ["j_c1_rm","j_thigh_rm","j_tibia_rm"]
        self.legs[5] = ["j_c1_rr","j_thigh_rr","j_tibia_rr"]
        self.legs[2] = ["j_c1_lf","j_thigh_lf","j_tibia_lf"]
        self.legs[3] = ["j_c1_lm","j_thigh_lm","j_tibia_lm"]
        self.legs[4] = ["j_c1_lr","j_thigh_lr","j_tibia_lr"]


# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat

# Updates the values of the dictionnary targets to set 3 angles to given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]

# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4


if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "direct":
    for name in sim.getJoints():
        print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    alphas = kinematicsnew.computeDK(0, 0, 0, use_rads=True)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])


while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematicsnew.computeDKDetailed(
            targets["j_c1_rf"], targets["j_thigh_rf"], targets["j_tibia_rf"]
        )
        i = -1
        for T in points:
            # Drawing each step of the DK calculation
            i += 1
            T = kinematicsnew.rotaton_2D(T[0], T[1], T[2], leg_angle)
            T[0] += leg_center_pos[0]
            T[1] += leg_center_pos[1]
            T[2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T, to_pybullet_quaternion(0, 0, leg_angle)
            )
        # Temp
        sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
        # sim.setRobotPose(
        #     leg_center_pos, to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        alphas = kinematicsnew.computeIK(x, y, z, verbose=True)

        # print(
        #     "Asked IK for x:{}, y:{}, z{}, got theta1:{}, theta2:{}, theta3:{}".format(
        #         x, y, z, alphas[0], alphas[1], alphas[2]
        #     )
        # )
        dk0 = kinematicsnew.computeDK(0, 0, 0, use_rads=True)
        print("dk0 = {}".format(dk0))
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematicsnew.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )

    elif args.mode == "robot-ik" :
        params = Parameters()
        for leg_id in range(1,7):
            alphas = kinematicsnew.computeIKOriented(0,0,0,leg_id,params)
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)
    sim.tick()

