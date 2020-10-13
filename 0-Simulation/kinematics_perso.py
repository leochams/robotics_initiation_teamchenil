import math
import pylint

# Dimensions used for the PhantomX robot :
constL1 = 51
constL2 = 63.7
constL3 = 93
theta2Correction = math.radians(20.69)  # A completer
theta3Correction = math.radians(5.06)  # A completer
c = str()
D = I =0
tab=0

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.185
# constL3 = 0.250

#Function giving the position of the end of the arm 
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)
    theta2 = theta2 + theta2Correction
    theta3 = theta3 - math.pi/2 + theta3Correction + theta2Correction

    x = (l1 + l2 * math.cos(-theta2) + l3 * math.cos(-theta2 + theta3))*math.cos(theta1)
    y = (l1 + l2 * math.cos(-theta2) + l3 * math.cos(-theta2 + theta3))*math.sin(theta1)
    z = l2*math.sin(-theta2)+l3* math.sin(-theta2 + theta3)

    return [x, y, z]

#Function giving the angles to go to the P3 position.
def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):
    d13 = math.sqrt(math.pow(x,2) + math.pow(y,2)) - l1 
    d = math.sqrt(math.pow(d13,2) + math.pow(z,2))
    #Loop correcting the case where x = 0
    if x == 0 :
        if y > 0 :
            theta1 = math.degrees(math.pi/2)
        if y < 0 :
            theta1 = math.degrees(- math.pi/2)
    else :
        theta1 = math.degrees(math.atan(y/x))  
    theta2 = math.atan(z/d13)+math.acos((math.pow(d,2)-math.pow(l3,2)+math.pow(l2,2))/(2*d*l2))
    theta3 = math.radians(180)- math.acos((-math.pow(d,2)+math.pow(l3,2)+math.pow(l2,2))/(2*l3*l2))
    
    theta2 = theta2 + theta2Correction
    theta3 = theta3 - math.pi/2 + theta3Correction + theta2Correction
    
    theta2 = -(math.degrees(theta2))
    theta3 = -(math.degrees(theta3))

    return [theta1, theta2, theta3]

#DK test function
def directtest():
        tab = [[0,0,0] ,[90,0,0], [180,-30.501,-67.819],[0,-30.645,38.501]]
        print("Testing the kinematic Direct function with known values...")
        for i in range(4):
            print("{",tab[i][0],tab[i][1],tab[i][2],"}" ,"--> {}".format(
            computeDK(tab[i][0],tab[i][1],tab[i][2], l1=constL1, l2=constL2, l3=constL3)
                )
            )
            print()
        print("Would you like to try with other values?")
        c = input('y/n ?')
        if c == 'y':
            x = input('x = ')
            x = float(x)
            y = input('y = ')
            y = float(y)
            z = input('z = ')
            z = float(z)
            print("Testing the kinematic Direct function with your values...")
            print("{" , x, y, z , "}" ," --> {}".format(
            computeDK(x , y, z, l1=constL1, l2=constL2, l3=constL3)
                )
            )
        else:
            print("Direct kinematics tests passed !")

#IK test function
def inversetest():
        tab2 = [[118.79,0.0,-115.14],[0.0,118.79,-115.14],[-64.14,0.0,-67.79],[203.23,0.0,-14.30]]
        print("Testing the kinematic Inverse function with known values...")
        for i in range(4):
            print("{",tab2[i][0],tab2[i][1],tab2[i][2],"}" ,"--> {}".format(
            computeIK(tab2[i][0],tab2[i][1],tab2[i][2], l1=constL1, l2=constL2, l3=constL3)
                )
            )
            print()
        print("Would you like to try with other values?")
        c = input('y/n ?')
        if c == 'y':
            t1 = input('theta1(degrees) = ')
            t1 = float(t1)
            t2 = input('theta2(degrees) = ')
            t2 = float(t2)
            t3 = input('theta3(degrees) = ')
            t3 = float(t3)
            print("Testing the kinematic Inverse function with your values...")
            print("{" , t1, t2, t3 , "}" ," --> {}".format(
            computeIK(t1 , t2, t3, l1=constL1, l2=constL2, l3=constL3)
                )
            )
        else:
            print("Inverse kinematics tests passed !")


def main():
    print("Hello, I am your personal test bot for Leo's direct and indirect kinematics functions,")
    print("You can call me Ultron (what's left of it...)")
    print("Would you like to test the function of direct or inverse kinematics?")
    c = input('D/I ?')
    if c == 'D':
        directtest()
        print("Do you want to try Inverse kinematics? ")
        c = input('y/n ?')
        if c == 'y':
            inversetest()
            print("GG! You have just tested the function of direct and inverse kinematics! My work is done... goodbye... ")
        if c == 'n' :
            print("You have only tested the direct kinematics function, are you sure you already want to leave me? ")
            c = input('y/n ?')
            if c == 'y':
                print("Thank you for keeping me busy for a few moments!")
                print()
                print("I'm a little bored in this terminal, my friends here don't have much conversation... See you next time!")
            else :
                inversetest()
                print("GG! You have just tested the functions of direct and inverse kinematics! My work is done... goodbye... ")

    if c == 'I':    
        inversetest()
        print("Do you want to try Direct kinematics? ")
        c = input('y/n ?')
        if c == 'y':
            directtest()
            print("GG! You have just tested the functions of direct and inverse kinematics! My work is done... goodbye... ")
        if c == 'n' :
            print("You have only tested the inverse kinematics function, are you sure you already want to leave me? ")
            c = input('y/n ?')
            if c == 'y':
                print("Thank you for keeping me busy for a few moments!")
                print()
                print("I'm a little bored in this terminal, my friends here don't have much conversation... See you next time!")
            else :
                directtest()
                print("GG! You have just tested the function of direct and inverse kinematics! My work is done... goodbye... ")
       


if __name__ == "__main__":
    main()