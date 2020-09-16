#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import os
import numpy as np
import sys, tty, termios
import math
import time
from geometry_msgs.msg import Twist
from dynamixel_sdk import *

'''



                 m1 |-------------------| m1
          Leg1      |     F R O N T     |      Leg3              /|\ x
                 m2 |                   | m2                      |
                    |                   |                         |
                    |                   |                  <----- o
                  RIGHT                LEFT                y
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
base_l          = 35.0               # Base length
base_w          = 60.0               # Base width
d               = 0.0                # Offset distance between side edge and foot center
xc1 =  L/2.0; yc1 = 0.0              # Joint-1 coordinates
xc2 = -L/2.0; yc2 = 0.0              # Joint-2 coordinates

    # ..... Trajectory .....
stroke_h        = 18.0               # Robot Height
ht              = 6.0                # Step height

    # ..... Temporal .....
def_period      = 1.0                # default period
dc              = 0.8                # Duty factor
dt              = 0.03               # Extension time
phi1            = +0.5               # Phase diff. b/w Contralateral legs (Leg 1 and Leg 4, Leg 2 and Leg 3)
phi2            = +0.75              # Phase diff. b/w Ipsilateral legs  (Leg 1 and Leg 3, Leg 2 and Leg 4)
step_size       = 0.002
n               = 0
transn_idx      = 0

leg_phase       = np.array([0, 0.5, 0.25, 0.75], dtype=float)

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_MOVING_SPEED        = 32
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_SPEED       = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Data Byte Length
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_POSITION     = 2
LEN_MX_MOVING_SPEED         = 2
LEN_MX_PRESENT_SPEED        = 2

# Default setting
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'      # Check which port is being used on your controller


TORQUE_ENABLE               = 1               # Value for enabling the torque
TORQUE_DISABLE              = 0               # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value
DXL_MOVING_STATUS_THRESHOLD = 2               # Dynamixel moving status threshold

NUM_MOTORS                  = 8
NUM_LEGS                    = 4
MOTOR_AXIS                  = np.ones(8, dtype=int)
MOTOR_OFFSETS               = np.array([2048 for i in range(NUM_MOTORS)],dtype=int)
JOINT_LIMITS                = np.array([[0,4095],[0,4095]])

prev_vel        = Twist()
curr_vel        = Twist()
reso_ang        = 0.01
reso_lin        = 0.001
isChanged       = False
isReceived      = 4
motor_angles    = np.zeros(8)
leg_offset      = np.array((leg_phase/step_size),dtype=int)


#motor_handler   = HandlerClass()


class HandlerClass:

    portHandler = None
    packetHandler = None

    def __init__(self):
        self.get_port_handler()
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

    def get_port_handler(self):
        self.portHandler = PortHandler(DEVICENAME)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def close_port(self):
        self.portHandler.closePort()


def setup_motors():
    check_connection()
    write_torque(TORQUE_ENABLE)

def check_connection():
    isConnected = True
    for i in range(NUM_MOTORS):
        id = i+1
        status = check_motor(id)
        if not status:
            isConnected = False
            print("Cannot communicate with motor id: %d"% (id))
    if not isConnected:
        print("Cannot communicate with all the motors")
        print("Cannot start base controller")
        exit(1)

def check_motor(id):
    val = True
    dxl_model_number, dxl_comm_result, dxl_error = motor_handler.packetHandler.ping(motor_handler.portHandler, id)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % motor_handler.packetHandler.getTxRxResult(dxl_comm_result))
        val = False
    elif dxl_error != 0:
        print("%s" % motor_handler.packetHandler.getRxPacketError(dxl_error))
        val = False
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, dxl_model_number))

    return val

def write_torque(character):
    for i in range(NUM_MOTORS):
        id = i+1
        dxl_comm_result, dxl_error = motor_handler.packetHandler.write1ByteTxRx(motor_handler.portHandler, id, ADDR_MX_TORQUE_ENABLE, character)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % motor_handler.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % motor_handler.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)

def write_motor_angles():
    goal_pos = get_goal_pos()
    execute_goal_pos(goal_pos)

def get_goal_pos():
    goal_pos = 2048*motor_angles/np.pi
    goal_pos = goal_pos*MOTOR_AXIS + MOTOR_OFFSETS
    goal_pos = np.array(goal_pos, dtype=int)

    return goal_pos

def move_motors():
    goal_pos_steps = get_goal_steps()
    for goal_pos in goal_pos_steps:
        execute_goal_pos(goal_pos)
        time.sleep(0.1)

def get_goal_steps():
    num_steps = 10
    goal_pos_steps  = []

    current_pos = get_current_pose()
    goal_pos = 2048*motor_angles/np.pi
    goal_pos = goal_pos*MOTOR_AXIS + MOTOR_OFFSETS
    diff_step = (goal_pos-current_pos)/num_steps
    diff_step.astype(int)

    for i in range(num_steps):
        goal_pos_steps.append(current_pos + (i+1)*diff_step)
    return goal_pos_steps

def get_current_pose():
    pos = np.zeros((NUM_MOTORS),dtype=int)
    for i in range(NUM_MOTORS):
        id = i+1
        dxl_present_position, dxl_comm_result, dxl_error = motor_handler.packetHandler.read2ByteTxRx(motor_handler.portHandler, id, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % motor_handler.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % motor_handler.packetHandler.getRxPacketError(dxl_error))

        pos[i] = dxl_present_position
        #print("[ID:%03d] PresPos:%d" % (id, dxl_present_position))

    return pos

def calculate_speeds(goal, current, time):
    diff = np.absolute((goal - current), dtype=float)
    speeds = diff/time
    speeds = np.array(speeds,dtype=int)
    return speeds

def set_moving_speeds(speeds): #yahan se
    groupSyncWrite = GroupSyncWrite(motor_handler.portHandler, motor_handler.packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED)
    param_moving_speeds_i = [0, 0]
    # Allocate moving speed value into byte array
    for i in range(NUM_MOTORS):
        param_moving_speeds_i = [DXL_LOBYTE(DXL_LOWORD(speeds[i])), DXL_HIBYTE(DXL_LOWORD(speeds[i]))]
        id = i+1
        dxl_addparam_result = groupSyncWrite.addParam(id, param_moving_speeds_i)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % motor_handler.packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("Moving speeds have been successfully written")

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    return True

def execute_goal_pos(goal):
    groupSyncWrite = GroupSyncWrite(motor_handler.portHandler, motor_handler.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
    param_goal_positions = [0, 0]
    # Add Dynamixel moving speed value to the Syncwrite parameter storage
    for i in range(NUM_MOTORS):
        id = i+1
        param_goal_positions_i = [DXL_LOBYTE(DXL_LOWORD(goal[i])), DXL_HIBYTE(DXL_LOWORD(goal[i]))]
        #print("Dynamixel#%d goal pos: %d" % (id, pos[id-1]))
        dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_positions_i)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))

        #print("Goal positions have been successfully written")

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    return True

def write_address(address, val, id):
    dxl_comm_result, dxl_error = motor_handler.packetHandler.write2ByteTxRx(motor_handler.portHandler, id, address, val)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % motor_handler.packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % motor_handler.packetHandler.getRxPacketError(dxl_error))

    return (dxl_comm_result == COMM_SUCCESS)


def getParams():
    global n
    global transn_idx

    lin_vel = 100*curr_vel.linear.x
    ang_vel = curr_vel.angular.z

    #params = [stroke, ht, period]
    l_params = np.zeros(3)
    r_params = np.zeros(3)

    vl = lin_vel - base_w*ang_vel/2
    vr = lin_vel + base_w*ang_vel/2

    if abs(vl) > reso_lin:
        l_params[0] = dc*def_period*vl
        l_params[1] = (2.5*abs(l_params[0])/8 + 1)
    if abs(vr) > reso_lin:
        r_params[0] = dc*def_period*vr
        r_params[1] = (2.5*abs(r_params[0])/8 + 1)

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
    Q2 = -(g1+o1);
    Q1 = -(np.pi-(g2+o2));

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
    #check_connection()                          #Uncomment motor_handler line 102
    motion_plan = solver()
    idx_t = int(0.625*n)
    get_motor_angles(motion_plan, idx_t)
    #move_motors()

    while not rospy.is_shutdown():
        #start_time = time.time()
        if isChanged:
            new_motion_plan = solver()
            isChanged = False
            isReceived = 0

        if isReceived != 4:
            change_motion_plan(idx_t, motion_plan, new_motion_plan)


        get_motor_angles(motion_plan, idx_t)
        #write_motor_angles()
        #time.sleep(step_size)
        idx_t = (idx_t+1)%n

        #print("--- %s seconds ---" % (time.time() - start_time))

    handler.close_port()
