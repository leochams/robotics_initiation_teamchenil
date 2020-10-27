#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics

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


# m_friction

controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(90, 0, 0, degrees = True))

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4


for name in sim.getJoints():
    #print(name)
    if "c1" in name or "thigh" in name or "tibia" in name:
        controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

while(True): 
    targets = {}  
    time.sleep(0.001)
    sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(10*time.time(), 0, 0, degrees = True))
    print(p.readUserDebugParameter(0))
    for name in controls.keys():
        targets[name] = p.readUserDebugParameter(controls[name])
    
    state = sim.setJoints(targets)
    print(targets)
    sim.tick()
    



# while True:
#     targets = {}
#     for name in sim.getJoints():
#         if "c1" in name or "thigh" in name or "tibia" in name:
#             targets[name] = 0
    
#     for name in controls.keys():
#         targets[name] = p.readUserDebugParameter(controls[name])

#         sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
       
#         state = sim.setJoints(targets)

#     sim.tick()

