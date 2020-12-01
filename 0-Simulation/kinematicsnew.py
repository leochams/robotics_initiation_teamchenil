import math
from constants import *
from scipy.optimize import minimize
import numpy as np

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=-1,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )

    return result


# Computes the inverse kinematics of a leg in a frame colinear to the robot's frame (x points in front of the robot, y points to its left, z towards the sky)
# but whose (0,0) point is leg dependent, ie will match the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes

#extrat theta help for adding angle in dplcmt
def computeIKOriented(x, y, z, legID, params, extra_theta = 0, verbose=False):

    x,y,z = rotaton_2D(x ,y ,z ,LEG_ANGLES[legID-1])
    alphas = computeIK(x + params.initLeg[legID-1][0],
                        y + params.initLeg[legID-1][1],
                        z+ params.z)
    return alphas 

def computeIKOrientedExtraAngle(x, y, z, legID, params, extra_theta , verbose=False):

    x,y,z = rotaton_2D(x ,y ,z ,LEG_ANGLES[legID-1] + extra_theta)
    alphas = computeIK(x + params.initLeg[legID-1][0],
                        y + params.initLeg[legID-1][1],
                        z+ params.z)
    return alphas 


# Computes the inverse kinematics of a leg in a frame colinear to the leg's frame (x points in front of the leg, y points to its left, z towards the sky)
# but whose (0,0) point matches the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKNotOriented(x, y, z, legID, params, verbose=False):
    return computeIK(
        x + params.initLeg[legID - 1][0],
        y + params.initLeg[legID - 1][1],
        z + params.z
    )

def computeIK_RobotCentered(x, y, z, legID, verbose=False):

    x -= LEG_CENTER_POS[legID-1][0]
    y -= LEG_CENTER_POS[legID-1][1]
    z -= LEG_CENTER_POS[legID-1][2]

    new_pos = rotaton_2D(x ,y ,z ,-LEG_ANGLES[legID-1])

    result = computeIK(new_pos[0],new_pos[1],new_pos[2],verbose=verbose,use_rads= True)
    return result
    
def rotation_new(x,y,z,duration=1):
    max_angle = math.pi/8
    angle = max_angle* math.sin (2*math.pi*time.time()*0.5)
    for leg_id in range (1,7):
        r = 0.3

        x = r * math.cos(angle)

        y = r * math.sin(angle)
        result = computeIK_RobotCentered(x,y,z,legID,verbose = False)
    return result

def rotaton_2D(x, y, z, theta):
    # Applying a rotation around the Z axis
    new_x = math.cos(theta)*x - math.sin(theta)* y 
    new_y = math.sin(theta)*x + math.cos(theta) * y
    return [new_x, new_y, z]


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


def trianglePoints(y, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    P1 = [0,y,z+h]
    P2 = [w/2,y,z]
    P3 = [-w/2,y,z]
    return [P1,P2,P3]

def segdist(P1,P2):
    seg = math.sqrt(math.pow(P2[0]-P1[0],2)+math.pow(P2[1]-P1[1],2)+math.pow(P2[2]-P1[2],2))
    return seg

def triangle(x, z, h, w, t, period,legID,params,extra_theta):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    alphas = [0,0,0]
    points = trianglePoints(x,z,h,w)
    d1 = segdist(points[0],points[1])
    d2 = segdist(points[1],points[2])
    d3 = segdist(points[2],points[0])
    peri1 = (d1/(d1+d2+d3))*period
    peri2 = (d2/(d1+d2+d3))*period
    peri3 = (d3/(d1+d2+d3))*period
    t = math.fmod(t,period)
    
    if  (t < peri1) : 
        alphas = segment_1way_ExtraAngle(points[0][0],points[0][1],points[0][2],points[1][0],points[1][1],points[1][2],t,peri1,legID,params,extra_theta)
    elif  (t < (peri1+peri2)) :
        alphas = segment_1way_ExtraAngle(points[1][0],points[1][1],points[1][2],points[2][0],points[2][1],points[2][2],t - peri1,peri2,legID,params,extra_theta)
    else :
        alphas = segment_1way_ExtraAngle(points[2][0],points[2][1],points[2][2],points[0][0],points[0][1],points[0][2],t - peri1 - peri2,peri3,legID,params,extra_theta)
    return alphas


def triangletimed(x, z, h, w, t, period,legID,params):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    alphas = [0,0,0]
    points = trianglePoints(x,z,h,w)
    d1 = segdist(points[0],points[1])
    d2 = segdist(points[1],points[2])
    d3 = segdist(points[2],points[0])
    peri1 = (d1/(d1+d2+d3))*period
    peri2 = (d2/(d1+d2+d3))*period
    peri3 = (d3/(d1+d2+d3))*period
    t = math.fmod(t,period)
    
    if  (t < peri1) : 
        alphas = segment_1way(points[0][0],points[0][1],points[0][2],points[1][0],points[1][1],points[1][2],t,peri1,legID,params)
    elif  (t < (peri1+peri2)) :
        alphas = segment_1way(points[1][0],points[1][1],points[1][2],points[2][0],points[2][1],points[2][2],t - peri1,peri2,legID,params)
    else :
        alphas = segment_1way(points[2][0],points[2][1],points[2][2],points[0][0],points[0][1],points[0][2],t - peri1 - peri2,peri3,legID,params)
    return alphas

def circlePoints(x, z, r, N=16):
    """
    Takes the geometric parameters of the cercle and returns N points approximating the circle. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], etc]
    """
    None


"""Autre façon de faire le cercle (à présenter)
            # y_circle = r * math.cos(2 * math.pi * (1 / duration) * sim.t)
            # z_circle = r * math.sin(2 * math.pi * (1 / duration) * sim.t)
            # alphas = kinematics.computeIK(x, y_circle, z_circle + z)
"""


def circle(x, z, r, t, duration,leg_id,params) :
    """
    Takes the geometric parameters of the circle and the current time, gives the joint angles to draw the circle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    y_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    z_circle =+ r * math.sin(2 * math.pi * (1 / duration) * t)
    p1 = [x,y_circle+r,z_circle ]
    p2 = [x,y_circle,z ]
    if z_circle< 0 :
        alphas = segment_1way(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],t,duration,leg_id,params)
    else :
        alphas = computeIK(x, y_circle, z_circle + z)
    return(alphas)

def segment_1way(segment_x1, segment_y1, segment_z1,segment_x2, segment_y2, segment_z2,t , duration,legID,params):
    nt = math.fmod(t,duration)
    x = (nt/duration) * (segment_x2 - segment_x1)+ segment_x1
    y = (nt/duration) * (segment_y2 - segment_y1)+ segment_y1
    z = (nt/duration) * (segment_z2 - segment_z1)+ segment_z1
    theta1, theta2, theta3 = computeIKOriented(x,y,z,legID,params)
    return(theta1,theta2,theta3) 

def segment_1wayNO(segment_x1, segment_y1, segment_z1,segment_x2, segment_y2, segment_z2,t , duration,legID,params):
    nt = math.fmod(t,duration)
    x = (nt/duration) * (segment_x2 - segment_x1)+ segment_x1
    y = (nt/duration) * (segment_y2 - segment_y1)+ segment_y1
    z = (nt/duration) * (segment_z2 - segment_z1)+ segment_z1
    theta1, theta2, theta3 = computeIKNotOriented(x,y,z,legID,params)
    return(theta1,theta2,theta3) 

def segment_1way_ExtraAngle(segment_x1, segment_y1, segment_z1,segment_x2, segment_y2, segment_z2,t , duration,legID,params,extra_theta):
    nt = math.fmod(t,duration)
    x = (nt/duration) * (segment_x2 - segment_x1)+ segment_x1
    y = (nt/duration) * (segment_y2 - segment_y1)+ segment_y1
    z = (nt/duration) * (segment_z2 - segment_z1)+ segment_z1
    theta1, theta2, theta3 = computeIKOrientedExtraAngle(x,y,z,legID,params,extra_theta)
    return(theta1,theta2,theta3) 

def segment(segment_x1, segment_y1, segment_z1,segment_x2, segment_y2, segment_z2,t , duration):
    nt = math.cos(2*math.pi*(t/duration) +1) * duration /2
    x = (nt/duration) * (segment_x2 - segment_x1)+ segment_x1
    y = (nt/duration) * (segment_y2 - segment_y1)+ segment_y1
    z = (nt/duration) * (segment_z2 - segment_z1)+ segment_z1
    theta1, theta2, theta3 = computeIK(x,y,z)
    return(theta1,theta2,theta3)


def segmentcircle(x,z,r,t,duration,legID,params,extra_theta):

    y_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    z_circle = + r * math.sin(2 * math.pi * (1 / duration) * t)
    p1 = [x,y_circle+r,z ]
    p2 = [x,y_circle-r,z ]
    #per1 = (d1/(d1+d2))*duration
    per1 = duration /2 

    if t<per1:
            alphas = segment_1way_ExtraAngle(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2],t,per1,legID,params,extra_theta)
    else :
        alphas = computeIKOrientedExtraAngle(x, y_circle, z_circle + z, legID, params, extra_theta , verbose=False)
    return alphas

def demicircleITA(x,z,r,t,duration,legID,params,extra_theta):       # demicercle en l'air 
    t = math.fmod(t,duration)
    y_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    z_circle = + r * math.sin(2 * math.pi * (1 / duration) * t)
    #per1 = (d1/(d1+d2))*duration
    per1 = duration /2 

    if t<per1:
        alphas = computeIKNotOriented(x, -y_circle, z_circle + z, legID, params, verbose=False)
        
    return alphas


def demicircleOTGITA(x,z,r,t,duration,legID,params):        # demicercle au sol puis demi cercle en l'air
    t = math.fmod(t,duration)
    per1 = duration/2

    if t<per1:
        alphas = demicircleOTG(x,z,r,t,duration,legID,params,extra_theta = False)
    else :
        alphas = demicircleITA(x,z,r,t - per1,duration,legID,params,extra_theta = False)
        
    return alphas

def demicircleOTG(x,z,r,t,duration,legID,params,extra_theta):       # que demicercle au sol pas de segment
    t = math.fmod(t,duration)
    y_circle = r * math.cos(2 * math.pi * (1 / duration) * t)
    x_circle =+ r * math.sin(2 * math.pi * (1 / duration) * t)
    #per1 = (d1/(d1+d2))*duration
    per1 = duration /2 

    if t<per1:
            alphas = computeIKNotOriented(x_circle + x, y_circle,z, legID, params, verbose=False)   
            #alphas = rotaton_2D(alphas[0],alphas[1],alphas[2], -math.pi/4)
    return alphas

def main():
    print(
        "0, -90, -90 --> ", computeDK(0, -90, -90, l1=constL1, l2=constL2, l3=constL3)
    )
    print("0, 0, 0 --> ", computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print("90, 0, 0 --> ", computeDK(90, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print(
        "180, -30.501, -67.819 --> ",
        computeDK(180, -30.501, -67.819, l1=constL1, l2=constL2, l3=constL3),
    )
    print(
        "0, -30.645, 38.501 --> ",
        computeDK(0, -30.645, 38.501, l1=constL1, l2=constL2, l3=constL3),
    )


if __name__ == "__main__":
    main()