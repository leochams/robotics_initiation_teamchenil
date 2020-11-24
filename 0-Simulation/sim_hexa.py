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
# from squaternion import Quaternion
from scipy.spatial.transform import Rotation

class Parameters:
    def __init__(
        self, z=-0.12,
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

#Init robot arms position
def initRobot(params,extra_theta):
    targets = {}
    for leg_id in range(1,7):
        alphas =  kinematicsnew.computeIKOriented(0,0,0,leg_id,params,extra_theta)
        set_leg_angles(alphas, leg_id, targets, params)
    state = sim.setJoints(targets)
    sim.tick()

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

params = Parameters()



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
elif args.mode == "robot-ik":
    controls["target_x"] = p.addUserDebugParameter("target_x",0,0.05)
    controls["target_y"] = p.addUserDebugParameter("target_y",0,0.05)
    controls["target_z"] = p.addUserDebugParameter("target_z",0,0.05)
    controls["direction"] = p.addUserDebugParameter("direction",0,2*math.pi,0)

elif args.mode == "walking":
    #controls["target_x"] = p.addUserDebugParameter("target_x",0,0,0)
    controls["target_z"] = p.addUserDebugParameter("target_z",-0.1,0.1,0)
    controls["target_h"] = p.addUserDebugParameter("target_h",0,0.2,0.005)
    controls["target_w"] = p.addUserDebugParameter("target_w",0.1,0.5,0.1)
    controls["target_p"] = p.addUserDebugParameter("target_p",0.1,10,1)
    controls["direction"] = p.addUserDebugParameter("direction",0,2*math.pi,0)

elif args.mode == "walkingcircle":
    #controls["target_x"] = p.addUserDebugParameter("target_x",0,0,0)
    controls["target_z"] = p.addUserDebugParameter("target_z",-0.1,0,-0.005)
    controls["target_r"] = p.addUserDebugParameter("target_r",0.01,0.5,0.03)
    # controls["target_w"] = p.addUserDebugParameter("target_w",0.1,0.5,0.1)
    controls["target_duration"] = p.addUserDebugParameter("target_duration",0.5,1,0.595)
    controls["direction"] = p.addUserDebugParameter("direction",0,2*math.pi,1.6)

elif args.mode == "walkingcircle_oriented":
    #controls["target_x"] = p.addUserDebugParameter("target_x",0,0,0)
    controls["target_z"] = p.addUserDebugParameter("target_z",-0.1,0,-0.045)
    controls["target_r1"] = p.addUserDebugParameter("target_r1",-5,0.5,0)
    #controls["target_r2"] = p.addUserDebugParameter("target_r2",-0.5,0.5,0)
    # controls["target_w"] = p.addUserDebugParameter("target_w",0.1,0.5,0.1)
    controls["target_duration"] = p.addUserDebugParameter("target_duration",0.5,1,0.595)
    controls["direction"] = p.addUserDebugParameter("direction",-math.pi,math.pi,math.pi/2)

elif args.mode == "rotatecircle":
    #controls["target_x"] = p.addUserDebugParameter("target_x",0,0,0)
    controls["target_z"] = p.addUserDebugParameter("target_z",-0.1,0,0.0)
    controls["target_r"] = p.addUserDebugParameter("target_r",0.01,0.5,0.023)
    # controls["target_w"] = p.addUserDebugParameter("target_w",0.1,0.5,0.1)
    controls["target_duration"] = p.addUserDebugParameter("target_duration",0.5,5,1)
    #controls["direction"] = p.addUserDebugParameter("direction",0,2*math.pi,0)

initRobot(params,0)


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
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        extra_theta = p.readUserDebugParameter(controls["direction"])
        for leg_id in range(1,7):
            alphas = kinematicsnew.computeIKOriented(x * math.sin(2*math.pi*0.5*time.time()),
                                                    y * math.sin(2*math.pi*0.5*time.time()),
                                                    z * math.sin(2*math.pi*0.5*time.time()),
                                                    leg_id,
                                                    params,
                                                    extra_theta)
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "walking" :
        #x = p.readUserDebugParameter(controls["target_x"])
        x=0
        z = p.readUserDebugParameter(controls["target_z"])
        h = p.readUserDebugParameter(controls["target_h"])
        w = p.readUserDebugParameter(controls["target_w"])
        period = p.readUserDebugParameter(controls["target_p"])
        extra_theta = p.readUserDebugParameter(controls["direction"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
                alphas = kinematicsnew.triangle(x,z,h,w,sim.t,period,leg_id,params,extra_theta)

                set_leg_angles(alphas, leg_id, targets, params)
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematicsnew.triangle(x,z,h,w,sim.t + 0.5*period ,period,leg_id,params,extra_theta)

                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)

    elif args.mode == "walkingcircle_oriented" :
        #x = p.readUserDebugParameter(controls["target_x"])
        x=0
        z = p.readUserDebugParameter(controls["target_z"])
        r1 = p.readUserDebugParameter(controls["target_r1"])
        #r2 = p.readUserDebugParameter(controls["target_r2"])
        # w = p.readUserDebugParameter(controls["target_w"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        extra_theta = p.readUserDebugParameter(controls["direction"])
        # if (direction <2) and (direction > -2):
        #     extra_theta = 0
        #     for leg_id in range (1,7):
        #         if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
        #             alphas = kinematicsnew.demicircle(x,z,r, sim.t,duration,leg_id,params,extra_theta)

        #             set_leg_angles(alphas, leg_id, targets, params)
        #         elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
        #             alphas = kinematicsnew.demicircle(x,z,r,sim.t + 0.5*duration ,duration,leg_id,params,extra_theta)

        #             set_leg_angles(alphas, leg_id, targets, params)
        #         state = sim.setJoints(targets)
        # elif (direction!= math.pi/2):
        #     extra_theta = math.pi/2
        #     for leg_id in range (1,7):
        #         if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
        #             alphas = kinematicsnew.demicircleOTG(x,z,r, sim.t,duration,leg_id,params)

        #             set_leg_angles(alphas, leg_id, targets, params)
        #         elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
        #             alphas = kinematicsnew.demicircleOTG(x,z,r,sim.t + 0.5*duration ,duration,leg_id,params)

        #             set_leg_angles(alphas, leg_id, targets, params)
        #         state = sim.setJoints(targets)

        for leg_id in range (1,7):
            r2 = 0.05
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
                alphas1 = kinematicsnew.demicircleOTGITA(x,z,r1, sim.t,duration,leg_id,params)
                alphas2 = kinematicsnew.segmentcircle(x,z,r2, sim.t,duration,leg_id,params,extra_theta)
                A0 = (alphas1[0] + alphas2[0])/2
                A1 = (alphas1[1] + alphas2[1])/2
                A2 = (alphas1[2] + alphas2[2])/2
                ultralphas = [A0,A1,A2]
                set_leg_angles(ultralphas, leg_id, targets, params)

            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas1 = kinematicsnew.demicircleOTGITA(x,z,r1,sim.t + 0.5*duration ,duration,leg_id,params)
                alphas2 = kinematicsnew.segmentcircle(x,z,r2,sim.t + 0.5*duration ,duration,leg_id,params,extra_theta)
                A0 = (alphas1[0] + alphas2[0])/2
                A1 = (alphas1[1] + alphas2[1])/2
                A2 = (alphas1[2] + alphas2[2])/2
                ultralphas = [A0,A1,A2]                
                #ultralphas = alphas1 + alphas2
                set_leg_angles(ultralphas, leg_id, targets, params)

            state = sim.setJoints(targets)
            #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    elif args.mode == "walkingcircle" :
        #x = p.readUserDebugParameter(controls["target_x"])
        x=0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        # w = p.readUserDebugParameter(controls["target_w"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        extra_theta = p.readUserDebugParameter(controls["direction"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
                alphas = kinematicsnew.segmentcircle(x,z,r, sim.t,duration,leg_id,params,extra_theta)

                set_leg_angles(alphas, leg_id, targets, params)
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematicsnew.segmentcircle(x,z,r,sim.t + 0.5*duration ,duration,leg_id,params,extra_theta)

                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    
    elif args.mode == "rotatecircle" :
        #x = p.readUserDebugParameter(controls["target_x"])
        x=0
        z = p.readUserDebugParameter(controls["target_z"])
        r = p.readUserDebugParameter(controls["target_r"])
        # w = p.readUserDebugParameter(controls["target_w"])
        duration = p.readUserDebugParameter(controls["target_duration"])
        #extra_theta = p.readUserDebugParameter(controls["direction"])
        for leg_id in range (1,7):
            if (leg_id == 1) or (leg_id == 3) or (leg_id == 5):
                alphas = kinematicsnew.demicircleOTGITA(x,z,r, sim.t,duration,leg_id,params)

                set_leg_angles(alphas, leg_id, targets, params)
            elif (leg_id == 2) or (leg_id == 4) or (leg_id == 6):
                alphas = kinematicsnew.demicircleOTGITA(x,z,r,sim.t + 0.5*duration ,duration,leg_id,params)

                set_leg_angles(alphas, leg_id, targets, params)
            state = sim.setJoints(targets)
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    
            
    robot_pose = (sim.getRobotPose()) # (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
    yaw = robot_pose[1][2]
    sim.lookAt(robot_pose[0]) 
    pos = [1*math.cos(time.time()),0,0]  
    state = sim.setJoints(targets)      #state contient 3 tab position/vitesse/force

    # Debug visuel
    for leg_id in range (1,7):
        pos = kinematicsnew.computeDK(state[params.legs[leg_id][0]][0],
                                    state[params.legs[leg_id][1]][0] ,
                                    state[params.legs[leg_id][2]][0] )
        yaw = robot_pose[1][2]
        pos = kinematicsnew.rotaton_2D(pos[0],pos[1],pos[2],- LEG_ANGLES[leg_id - 1] + yaw)  
        leg_c_pos = kinematicsnew.rotaton_2D(LEG_CENTER_POS[leg_id-1][0],LEG_CENTER_POS[leg_id-1][1],LEG_CENTER_POS[leg_id-1][2],yaw)
        pos[0] += leg_c_pos[0] + robot_pose[0][0]
        pos[1] += leg_c_pos[1] + robot_pose[0][1]
        pos[2] += leg_c_pos[2] + robot_pose[0][2]

        
        print("DK pour {} = {}". format(leg_id,pos))
        sim.addDebugPosition(pos, duration=3)

    sim.tick()

