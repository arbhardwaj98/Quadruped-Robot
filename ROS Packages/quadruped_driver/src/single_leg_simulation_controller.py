#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import os
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from quadruped_driver.msg import LegState

'''



                 m1 |-------------------| m1
          Leg1      |     F R O N T     |      Leg3              /|\ x
                 m2 |                   | m2                      |
                    |                   |                         |
                    |                   |                  <----- o
                  LEFT                RIGHT                y
                    |                   |
                    |                   |
                 m1 |                   | m1
          Leg2      |      B A C K      |      Leg4
                 m2 |-------------------| m2



'''

    # ..... Geometrical .....
r1              = 20.0               # length of Cranks
r2              = 40.0               # length of Coupler Links
L               = 5.0                # Distance between the pivot points
base_l          = 60.0               # Base length
base_w          = 35.0               # Base width
d               = 0.0                # Offset distance between side edge and foot center
xc1 =  L/2.0; yc1 = 0.0              # Joint-1 coordinates
xc2 = -L/2.0; yc2 = 0.0              # Joint-2 coordinates

    # ..... Trajectory .....
stroke_h        = 36.0               # Robot Height
ht              = 12.0                # Step height

    # ..... Temporal .....
def_period      = 4.0                # default period
dc              = 0.8                # Duty factor
dt              = 0.03               # Extension time
phi1            = +0.5               # Phase diff. b/w Contralateral legs (Leg 1 and Leg 4, Leg 2 and Leg 3)
phi2            = +0.75              # Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3, Leg 2 and Leg 4)
step_size       = 0.002
n               = 0
transn_idx      = 0

leg_phase       = np.array([0, 0.5, 0.25, 0.75], dtype=float)

NUM_MOTORS                  = 8
NUM_LEGS                    = 4
MOTOR_AXIS                  = np.ones(8, dtype=int)
MOTOR_OFFSETS               = np.array([0 for i in range(NUM_MOTORS)],dtype=int)
JOINT_LIMITS                = np.array([[0,4095],[0,4095]])

prev_vel        = Twist()
curr_vel        = Twist()
reso_ang        = 0.01
reso_lin        = 0.001
isChanged       = False
isReceived      = 4
motor_angles    = np.zeros(8)
leg_offset      = np.array((leg_phase/step_size),dtype=int)

joint_list = ["motor_1","motor_2"]

class HandlerClass:

    pub_list = []
    leg_state_pub = None
    def __init__(self):
        for joint in joint_list:
            topic = "single_leg/" + joint + "_position_controller/command"
            pub = rospy.Publisher(topic, Float64, queue_size=10)
            self.pub_list.append(pub)
        self.leg_state_pub = rospy.Publisher("/quadruped/leg_contact", LegState, queue_size=10)

motor_handler   = HandlerClass()

def write_motor_angles():
    goal_pos = get_goal_pos()
    execute_goal_pos(goal_pos)

def write_leg_state(idx_t):
    i = (idx_t+leg_offset[:])%n
    lim_max = dc*n/2
    lim_min = n - dc*n/2
    k = (i > lim_min) | (i < lim_max)
    motor_handler.leg_state_pub.publish(k)

def get_goal_pos():
    goal_pos = motor_angles*MOTOR_AXIS + MOTOR_OFFSETS
    return goal_pos

def move_motors():
    goal_pos_steps = get_goal_steps()
    for goal_pos in goal_pos_steps:
        execute_goal_pos(goal_pos)
        time.sleep(0.01)

def get_goal_steps():
    num_steps = 100
    goal_pos_steps  = []

    current_pos = get_current_pose()
    goal_pos = motor_angles*MOTOR_AXIS + MOTOR_OFFSETS
    diff_step = (goal_pos-current_pos)/num_steps

    for i in range(num_steps):
        goal_pos_steps.append(current_pos + (i+1)*diff_step)
    return goal_pos_steps

def get_current_pose():
    joint_state = rospy.wait_for_message('single_leg/joint_states', JointState, timeout=None)
    pos = np.array(joint_state.position,dtype=float)
    return pos

def calculate_speeds(goal, current, time):
    diff = np.absolute((goal - current), dtype=float)
    speeds = diff/time
    speeds = np.array(speeds,dtype=int)
    return speeds

def execute_goal_pos(goal):
    for i in range(2):
        pub = motor_handler.pub_list[i]
        pub.publish(goal[i])

def getParams():
    global n
    global transn_idx

    lin_vel = 100*curr_vel.linear.x
    ang_vel = curr_vel.angular.z

    #params = [stroke, ht, period]
    l_params = np.zeros(3)
    r_params = np.zeros(3)

    vl = lin_vel - base_w*ang_vel/2.0
    vr = lin_vel + base_w*ang_vel/2.0

    if abs(vl) > reso_lin:
        l_params[0] = dc*def_period*vl
        l_params[1] = (2*abs(l_params[0])/8 + 1)
    if abs(vr) > reso_lin:
        r_params[0] = dc*def_period*vr
        r_params[1] = (2*abs(r_params[0])/8 + 1)

    l_params[2] = def_period
    r_params[2] = def_period

    n = int(def_period/step_size)
    transn_idx = (n-leg_offset)%n

    return [l_params, r_params]

def getTrajectory(paramTraj):
    stroke, ht, period = paramTraj

    C_x = np.array([              - (stroke/2) - (dt*stroke)/(dc*period),
                                                    - stroke/(dc*period),
                      (3*stroke)/(dc*pow((2*dt - period + dc*period),2)),
                     (2*stroke)/(dc*pow((2*dt - period + dc*period),3))],
                                                             dtype=float)

    C_y = np.array([                               -stroke_h,
                                                           0,
                            (16*ht)/pow((period*(dc - 1)),2),
                            (32*ht)/pow((period*(dc - 1)),3),
                           (16*ht)/pow((period*(dc - 1)),4)],
                                                 dtype=float)


    Time = np.arange(0,period,step_size)
    X = np.zeros(n); Y = np.zeros(n)

    #.......... X vs Time ..........#
    Phase_x = np.array([((0.5*dc*period+dt)/step_size), ((period-(dt+0.5*dc*period))/step_size)], dtype=int)
    X[0:Phase_x[0]] = -stroke*Time[0:Phase_x[0]]/(dc*period)
    t = Time[Phase_x[0]:Phase_x[1]] - (0.5*dc*period+dt)
    X[Phase_x[0]:Phase_x[1]] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3))).T),C_x)
    X[Phase_x[1]:] = stroke*(period - Time[Phase_x[1]:])/(dc*period)

    #.......... Y vs Time ..........#
    Phase_y = np.array([(0.5*dc*period)/step_size, ((1 - 0.5*dc)*period)/step_size], dtype=int)
    Y[0:Phase_y[0]] = -1*stroke_h*np.ones(Phase_y[0])
    t = Time[Phase_y[0]:Phase_y[1]] - (0.5*dc*period)
    Y[Phase_y[0]:Phase_y[1]] = np.dot((np.vstack((np.ones(t.size), t, np.power(t,2), np.power(t,3), np.power(t,4))).T),C_y)
    Y[Phase_y[1]:] = -1*stroke_h*np.ones(n-Phase_y[1])

    #return np.column_stack((X, Y, Time))
    return [X, Y, Time]

def subplotTraj(x,y,idx,name):
    plt.subplot(idx)
    plt.plot(x, y);
    plt.title(name);

def plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r):
    subplotTraj(Time_l, X_l, 231, 'Left X vs Time')
    subplotTraj(Time_l, Y_l, 232, 'Left Y vs Time')
    subplotTraj(X_l, Y_l, 233, 'Left Trajectory')

    subplotTraj(Time_r, X_r, 234, 'Right X vs Time')
    subplotTraj(Time_r, Y_r, 235, 'Right Y vs Time')
    subplotTraj(X_r, Y_r, 236, 'Right Trajectory')

    plt.show()

def solveIK(x,y):
    a_sq = np.power((x - xc1),2) + np.power((y - yc1),2);
    b_sq = np.power((x - xc2),2) + np.power((y - yc2),2);
    a = np.sqrt(a_sq)
    b = np.sqrt(b_sq)

    o1 = np.arccos((-(r2*r2) + (r1*r1) + b_sq)/(2*r1*b));
    o2 = np.arccos((-(r2*r2) + (r1*r1) + a_sq)/(2*r1*a));
    g1 = np.arccos(-(a_sq - (L*L) - b_sq)/(2*L*b));
    g2 = np.arccos(-(b_sq - (L*L) - a_sq)/(2*L*a));
    Q2 = +np.pi-(g1+o1);
    Q1 = -np.pi+(g2+o2);

    return [Q1, Q2]

def saveData(pos, var):
    np.savetxt(pos, var, delimiter=",")

def cmd_vel_listener():
    global curr_vel
    rospy.init_node('base_controller', anonymous=True)
    curr_vel = rospy.wait_for_message("cmd_vel", Twist, timeout=None)
    rospy.Subscriber("cmd_vel", Twist, callback)

def callback(data):
    global prev_vel
    global curr_vel
    global isChanged
    curr_vel = data
    if abs(curr_vel.linear.x - prev_vel.linear.x) > reso_lin or abs(curr_vel.angular.z - prev_vel.angular.z) > reso_ang:
        prev_vel = curr_vel
        isChanged = True

def solver():
    l_params, r_params = getParams()
    [X_l, Y_l, Time_l] = getTrajectory(l_params)
    [X_r, Y_r, Time_r] = getTrajectory(r_params)
    ttraj = np.concatenate((X_l.reshape(-1,1),Y_l.reshape(-1,1)), axis=1)
    #np.savetxt("traj.csv", ttraj, delimiter=",")
    print(l_params)
    print(r_params)
    #plotTraj(X_l, Y_l, Time_l, X_r, Y_r, Time_r)
    [Q1_l, Q2_l] = solveIK(X_l,Y_l)
    [Q1_r, Q2_r] = solveIK(X_r,Y_r)

    return [Q1_l, Q2_l, Q1_r, Q2_r]

def change_motion_plan(idx_t, motion_plan, new_motion_plan):
    global isReceived
    isReceived = isReceived + 1
    if idx_t == transn_idx[0]: motion_plan[0] = new_motion_plan[0]
    elif idx_t == transn_idx[1]: motion_plan[1] = new_motion_plan[1]
    elif idx_t == transn_idx[2]: motion_plan[2] = new_motion_plan[2]
    elif idx_t == transn_idx[3]: motion_plan[3] = new_motion_plan[3]
    else: isReceived = isReceived - 1

def get_motor_angles(motion_plan, idx_t):
    global motor_angles
    motor_angles[0] = motion_plan[0][(idx_t+leg_offset[0])%n]
    motor_angles[1] = motion_plan[1][(idx_t+leg_offset[0])%n]
    motor_angles[2] = motion_plan[0][(idx_t+leg_offset[1])%n]
    motor_angles[3] = motion_plan[1][(idx_t+leg_offset[1])%n]
    motor_angles[4] = motion_plan[2][(idx_t+leg_offset[2])%n]
    motor_angles[5] = motion_plan[3][(idx_t+leg_offset[2])%n]
    motor_angles[6] = motion_plan[2][(idx_t+leg_offset[3])%n]
    motor_angles[7] = motion_plan[3][(idx_t+leg_offset[3])%n]


if __name__ == '__main__':

    cmd_vel_listener()
    motion_plan = solver()
    idx_t = int(0.625*n)
    get_motor_angles(motion_plan, idx_t)
    print(motor_angles)
    #move_motors()
    time.sleep(0.1)
    step_last_time = time.time()

    while not rospy.is_shutdown():

        if isChanged:
            new_motion_plan = solver()
            isChanged = False
            isReceived = 0

        if isReceived != 4:
            change_motion_plan(idx_t, motion_plan, new_motion_plan)

        get_motor_angles(motion_plan, idx_t)
        time.sleep(max(0,step_size-step_last_time))
        write_motor_angles()
        step_last_time = time.time()
        #write_leg_state(idx_t)

        idx_t = (idx_t+1)%n
