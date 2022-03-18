from cProfile import label
from tkinter import FIRST
from traceback import print_tb
from turtle import distance
import matplotlib.pyplot as plt
import numpy as np
import math


from enum import Enum

TAU = 2*math.pi

	# public static double normalizeAngleRR(double radians) {
	# 	double modifiedAngle = radians % TAU;
	# 	modifiedAngle = (modifiedAngle + TAU) % TAU;
	# 	return modifiedAngle;
	# }

def normalizeAngleRR(radians):
    modifiedAngle = radians % TAU
    modifiedAngle = (modifiedAngle + TAU) % TAU
    return modifiedAngle


	# /**
	#  * sus thing that sometimes fixes imu data
	#  * @param radians read angle in radians
	#  * @return sus angle
	#  */
	# public static double normalizeAngle(double radians) {
	# 	return AngleWrap(-normalizeAngleRR(radians));
	# }

def normalizeAngle(radians):
    return angleWrap(-normalizeAngleRR(radians))
	# /**
	#  * calculates the normalized heading error from roadrunner odometry
	#  * @param referenceRadians the angle we would like to be at
	#  * @param stateRadians the angle where we are
	#  * @return the normalized heading error in radians
	#  */
	# public static double normalizedHeadingError(double referenceRadians, double stateRadians) {

	# 	return normalizeAngle(referenceRadians - stateRadians);
	# }
def normalizeHeadingError(referenceRadians, stateRadians):
    return normalizeAngle(referenceRadians - stateRadians)

class PIDController:
    def __init__(self,Kp,Ki,Kd,dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.lastError = 0

    def update(self,error):
        self.integral = self.integral + error*self.dt
        derivative = (error - self.lastError)/self.dt
        self.lastError = error
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

def angleWrap(x):
    return math.atan2(math.sin(x),math.cos(x))


Kv = 0.5 # Velocity given input 
Ka = 0.5 # Acceleration given input
W = 0.2 # Wheelbase

# 5 states: x,y,theta,v_l,v_r
x = np.zeros(5)

def getU(voltage_l,voltage_r):
    u = np.array([voltage_l,voltage_r])
    return u

# function to get the u vector from the input
def getB():
    u = np.zeros((5,2))
    u[3][0] = 1/Ka
    u[4][1] = 1/Ka

    return u 

def getA(theta):
    A = np.zeros((5,5))

    # # rotation portion of the A matrix
    A[0][0] = math.cos(theta)
    A[0][1] = -math.sin(theta)
    A[1][0] = math.sin(theta)
    A[1][1] = math.cos(theta)

    # velocity portion of the A matrix
    A[2][2] = 1
    A[3][3] = -(Kv/Ka)
    A[4][4] = -(Kv/Ka)


    return A



def getC():
    return np.identity(5)

def getD():
    return np.zeros((5,2))

def calculateXVector(vl,vr):
    x = np.zeros(5)
    x[0] = (vl + vr)/2
    x[1] = 0
    x[2] = (vr-vl) / W
    x[3] = vl
    x[4] = vr
    return x

def simulate(x,voltage_l,voltage_r,dt):
    A = getA(x[2])
    B = getB()
    u = getU(voltage_l=voltage_l,voltage_r=voltage_r)
    # print("A")
    # print(A)
    # print("B")
    # print(B)
    # print("u")
    # print(u)
    # print("x")
    # print(x)
    vl = x[3]
    vr = x[4]




    xdot = A @ calculateXVector(vl, vr) + B @ u
    x = x + xdot*dt
    x[2] = angleWrap(x[2])


    return x


def sinc(x):
    if x == 0:
        return 1
    else:
        return math.sin(x)/x

dt = 0.1
t = np.arange(0,250 * 10,dt)

positionsX = []
positionsY = []

thetaError = []

referenceX = 10
referenceY = 10

firstPosition = (-10, 40)
secondPosition = (10, 40)
thirdPosition = (20, -20)

drivePid = PIDController(Kp=0.01,Ki=0.0,Kd=0.0,dt=dt)
turnPid = PIDController(Kp=0.09,Ki=0.0,Kd=0.0,dt=dt)
headings = []
refences = []
distances = []
leftVelocities = []
rightVelocities = []


class FollowingStates(Enum):
    FIRST = 0
    SECOND = 1 
    THIRD = 2

state = FollowingStates.FIRST

for i in range(len(t)):


    print("state: " + str(state))

    # print refences
    print("reference: " + str(referenceX) + "," + str(referenceY))


    x_pos = x[1]
    y_pos = x[0]

    print("x: " + str(x[0]))
    print("y: " + str(x[1]))
    print("theta: " + str(x[2]))
    print("vl: " + str(x[3]))
    print("vr: " + str(x[4]))
    leftVelocities.append(x[3])
    rightVelocities.append(x[4])

    # calculate the error
    distance = math.sqrt((referenceX - x_pos)**2 + (referenceY - y_pos)**2)
    reference = math.atan2(referenceX - x_pos,referenceY - y_pos)
    headingError = angleWrap(reference- x[2])
    thetaError.append(math.degrees(headingError))

    distances.append(distance)
    # print the heading, reference heading, and error in degrees
    if state == FollowingStates.FIRST:
        referenceX = firstPosition[0]
        referenceY = firstPosition[1]
        distance = math.sqrt((referenceX - x_pos)**2 + (referenceY - y_pos)**2)
        if (distance < 0.1):
            print("Reached first position")
            state = FollowingStates.SECOND
    elif state == FollowingStates.SECOND:
        referenceX = secondPosition[0]
        referenceY = secondPosition[1]
        distance = math.sqrt((referenceX - x_pos)**2 + (referenceY - y_pos)**2)

        if (distance < 0.1):
            print("Reached second position")
            state = FollowingStates.THIRD
    elif state == FollowingStates.THIRD:
        referenceX = thirdPosition[0]
        referenceY = thirdPosition[1]
            
    headings.append(math.degrees(x[2]))
    refences.append(math.degrees(reference))

    forwardPower = drivePid.update(error=distance)
    turnPower = turnPid.update(error=headingError)

    voltage_l = forwardPower - turnPower
    voltage_r = forwardPower + turnPower

    x = simulate(x,voltage_l,voltage_r,dt)
    # print("x")
    # print(x)
    positionsX.append(x[0])
    positionsY.append(x[1]) 




plt.plot(positionsY,positionsX,label="Position")
plt.scatter([firstPosition[0]],[firstPosition[1]],c='g',label="First Pose")
plt.scatter([secondPosition[0]],[secondPosition[1]],c='b',label="Second Pose")
plt.scatter([thirdPosition[0]],[thirdPosition[1]],c='y',label="Third Pose")
plt.legend()
plt.show()

plt.plot(t,thetaError,label="Theta Error")
plt.plot(t,headings,label="Heading")
plt.plot(t,refences,label="Reference Heading")
plt.legend()
plt.show()


plt.plot(t,distances,label="Distance")
plt.legend()
plt.show()

plt.plot(t,leftVelocities,label="Left Velocity")
plt.plot(t,rightVelocities,label="Right Velocity")
plt.legend()
plt.show()
