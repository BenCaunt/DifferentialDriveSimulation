from cProfile import label
from turtle import distance
import matplotlib.pyplot as plt
import numpy as np
import math



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

    # rotation portion of the A matrix
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
    print("A")
    print(A)
    print("B")
    print(B)
    print("u")
    print(u)
    print("x")
    print(x)
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

# simulate the system for 10 seconds with a dt of 0.1 seconds
dt = 0.1
t = np.arange(0,250,dt)

positionsX = []
positionsY = []

thetaError = []

referenceX = -10
referenceY = 40

drivePid = PIDController(Kp=0.21,Ki=0.0,Kd=0.0,dt=dt)
turnPid = PIDController(Kp=0.06,Ki=0.0,Kd=0.019,dt=dt)

for i in range(len(t)):

    # calculate the error
    distance = math.sqrt((referenceX - x[0])**2 + (referenceY - x[1])**2)
    headingError = math.atan2(referenceY - x[1],referenceX - x[0]) - x[2]
    thetaError.append(headingError + math.pi)
    headingError = angleWrap(headingError)

    forwardPower = drivePid.update(error=-distance)
    turnPower = turnPid.update(error=headingError)

    voltage_l = forwardPower * sinc(headingError) + turnPower
    voltage_r = forwardPower * sinc(headingError) - turnPower

    
    x = simulate(x,voltage_l,voltage_r,dt)
    print("x")
    print(x)
    positionsX.append(x[0])
    positionsY.append(x[1]) 

plt.plot(positionsX,positionsY,label="Position")
plt.scatter([referenceX],[referenceY],c='r',label="Reference Pose")
plt.legend()
plt.show()

plt.cla()
plt.plot(t,thetaError,label="Theta Error")
plt.legend()
plt.show()
