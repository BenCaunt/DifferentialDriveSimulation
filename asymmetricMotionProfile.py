import matplotlib.pyplot as plt 
from math import fabs
from math import sqrt
import numpy as np

max_accel = 12
max_decel = -4
max_vel = 10
targetPosition = 30


def signum(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

class motionState:
    def __init__(self, x, v, a):
        '''
        position, velocity, and acceleration at a given time step
        '''
        self.x = x
        self.v = v
        self.a = a
    def __str__(self) -> str:
        return "x: " + str(self.x) + " v: " + str(self.v) + " a: " + str(self.a)

class asymmetricalProfile:
    def __init__(self, max_accel, max_decel, max_vel, targetPosition):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_vel = max_vel
        self.targetPosition = targetPosition
        self.motionState = motionState(0, 0, 0)
        self.dt1 = None 
        self.dt2 = None
        self.dt3 = None
        self.profileDuration = None
        self.calculate()
    def calculate(self):
        '''
        Calculate the motion profile for the robot
        '''
        self.dt1 = fabs(self.max_vel) / fabs(self.max_accel)
        self.dt3 = fabs(self.max_vel) / fabs(self.max_decel)
        averageDt = (self.dt1 + self.dt3) / 2
        self.dt2 = fabs(self.targetPosition)/fabs(self.max_vel) - averageDt
        if (self.dt2 < 0):
            self.dt2 = 0
            
            if (fabs(self.max_accel) > fabs(self.max_decel)):
                self.max_accel = fabs(self.max_decel)
            else:
                self.max_decel = fabs(self.max_accel)

            self.dt1 = sqrt(fabs(self.targetPosition)/fabs(self.max_accel))
            self.dt3 = sqrt(fabs(self.targetPosition)/fabs(self.max_decel))

        self.profileDuration = self.dt1 + self.dt2 + self.dt3
    def getState(self, seconds):
        '''
        Get the state of the robot at a given time
        '''
        accel = 0
        velocity = 0
        position = 0

        if seconds <= self.dt1:
            # accel 
            accel = fabs(self.max_accel)
            velocity = seconds * accel  
            position = 0.5 * accel * seconds**2
        elif seconds <= self.dt1 + self.dt2:
            # cruise 
            accel = 0
            velocity = self.getState(self.dt1).v
            position = self.getState(self.dt1).x + self.max_vel * (seconds - self.dt1)
        elif seconds <= self.dt1 + self.dt2 + self.dt3:
            # decel 
            accel = fabs(self.max_decel)
            coastVelocity = self.getState(self.dt1).v
            velocity = coastVelocity - (seconds - self.dt1 - self.dt2) * accel
            endOfdt2 = self.dt1 + self.dt2
            endofdt2Pos = self.getState(endOfdt2).x
            position = endofdt2Pos + coastVelocity * (seconds - endOfdt2) - 0.5 * accel * (seconds - endOfdt2)**2
            accel = -accel
        else:
            # done 
            accel = 0
            velocity = 0
            position = self.targetPosition
        return motionState(position, velocity, accel)


profile = asymmetricalProfile(max_accel, max_decel, max_vel, targetPosition)

# graph the profile 
time = np.arange(0, profile.profileDuration, 0.01)
states = np.array([profile.getState(t) for t in time])

positions = [state.x for state in states]
velocities = [state.v for state in states]
accelerations = [state.a for state in states]


print("dt1: " + str(profile.dt1))
print("dt2: " + str(profile.dt2))
print("dt3: " + str(profile.dt3))
print("profile duration (s): " + str(profile.profileDuration))
plt.plot(time, positions, label="position")
plt.plot(time, velocities, label="velocity")
plt.plot(time, accelerations, label="acceleration")
plt.legend()
plt.show()
