#!/usr/bin/env python

import numpy as np
import rospy
import os
import math
import time
import roslib
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from quadruped_driver.msg import LegState

    # ..... Geometrical .....
r1              = 20.0               # length of Cranks
r2              = 40.0               # length of Coupler Links
L               = 5.0                # Distance between the pivot points
base_l          = 60.0               # Base length
base_w          = 35.0               # Base width
d               = 0.0                # Offset distance between side edge and foot center
xc1 =  L/2.0; yc1 = 0.0              # Joint-1 coordinates
xc2 = -L/2.0; yc2 = 0.0              # Joint-2 coordinates
c1 = np.array([xc1, yc1]).reshape(1,2)
c2 = np.array([xc2, yc2]).reshape(1,2)

NUM_MOTORS                  = 8
NUM_LEGS                    = 4
MOTOR_AXIS                  = np.ones(8, dtype=int)
MOTOR_OFFSETS               = np.array([0 for i in range(NUM_MOTORS)],dtype=int)

sub = 0
pos_a = np.zeros((800,2))
num = 0

def forward_kinematics(position):
    rcos = r1*np.cos(position)
    rsin = r1*np.sin(position)
    a = c1 + np.hstack([rcos[:,0].reshape(800,1),rsin[:,0].reshape(800,1)])
    b = c2 + np.hstack([-1*rcos[:,1].reshape(800,1), -rsin[:,1].reshape(800,1)])
    m = (a+b)/2
    d = np.linalg.norm(a-b, axis=1, keepdims=True)
    sq = 0.5*np.sqrt((4*(r2**2)/d**2)-1)
    dfy = b[:,1]-a[:,1]
    dfx = a[:,0]-b[:,0]
    df = np.hstack((dfy.reshape(800,1), dfx.reshape(800,1)))
    toe_cord = m - sq*df
    return toe_cord

def callback(data):
    global num, pos_a
    if num<800:
        pos_a[num, :] = np.array(data.position).reshape(1,2)
        num = num+1
        print(num)
    if num==800:
        num = num+1
        print(pos_a)

def init_node():
    global sub
    rospy.init_node('single_leg_listener', anonymous=True)
    sub = rospy.Subscriber("single_leg/joint_states", JointState, callback)

if __name__ == '__main__':

    try:
        init_node()
        print(1)
        t = rospy.Duration(6.0)
        print(1)
        while(num<800):
            pass
        print(1)
        #pos_a = obj.pos_a
        print(1)
        toe_cord = forward_kinematics(pos_a)
        print(1)
        print(toe_cord.shape)
        np.savetxt("actraj.csv", toe_cord, delimiter=",")

    except rospy.ROSInterruptException:
        pass
