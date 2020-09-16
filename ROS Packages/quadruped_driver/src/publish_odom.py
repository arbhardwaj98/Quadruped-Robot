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

x, y, z, th = 0,0,0,0
t = rospy.Time()
prev = [x, y, z, th, t]
leg_states = np.ones((4,1))
p_toe_cord = np.zeros([4,2])

br = tf.TransformBroadcaster()
odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

def forward_kinematics(data):
    position = np.array(data.position).reshape(4,2)
    rcos = r1*np.cos(position)
    rsin = r1*np.sin(position)
    a = c1 + np.hstack([rcos[:,0].reshape(4,1),rsin[:,0].reshape(4,1)])
    b = c2 + np.hstack([-1*rcos[:,1].reshape(4,1),rsin[:,1].reshape(4,1)])
    m = (a+b)/2
    d = np.linalg.norm(a-b, axis=1, keepdims=True)
    sq = 0.5*np.sqrt((4*(r2**2)/d**2)-1)
    dfy = b[:,1]-a[:,1]
    dfx = a[:,0]-b[:,0]
    df = np.hstack((dfy.reshape(4,1), dfx.reshape(4,1)))
    toe_cord = m - sq*df
    return toe_cord

def get_odom(data):
    global p_toe_cord, prev
    toe_cord = forward_kinematics(data)
    ms = -1*leg_states*(toe_cord-p_toe_cord)
    p_toe_cord = toe_cord
    ml = sum(ms[[0,1],0])/sum(leg_states[[0,1]])
    print(".....")
    print(ml)
    mr = sum(ms[[2,3],0])/sum(leg_states[[2,3]])
    print(mr)
    lin_ms = (ml+mr)/200.0
    print(lin_ms)
    ang_ms = 100*(mr-ml)/base_w
    print(ang_ms)
    x = np.mean(prev[0] + lin_ms*np.cos(prev[3]+ang_ms/2))
    y = np.mean(prev[1] + lin_ms*np.sin(prev[3]+ang_ms/2))
    z = -1*np.mean(leg_states*toe_cord[:,1].reshape(4,1))/100.0
    th = prev[3] + np.mean(ang_ms)
    t = data.header.stamp
    #print(x, y, z, th, t)
    publish_transform(x, y, z, th, t)
    publish_odometry(x, y, z, th, t, prev)
    return [x, y, z, th, t]

def init_node():
    rospy.init_node('odom_publisher', anonymous=True)
    rospy.Subscriber("quadruped/joint_states", JointState, callback)
    rospy.Subscriber("quadruped/leg_contact", LegState, callback_2)
    rospy.spin()

def callback(data):
    global prev
    prev = get_odom(data)

def callback_2(data):
    global leg_states
    leg_states = np.asarray(data.data,dtype=int).reshape(4,1)

def publish_transform(x, y, z, th, t):
    global br
    br.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, 0, th), t, "base_link", "odom")

def publish_odometry(x, y, z, th, t, prev):
    global odom_pub
    odom = Odometry()
    time_diff = (t.to_sec()-prev[4].to_sec())

    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.header.stamp = t

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z
    q = tf.transformations.quaternion_from_euler(0, 0, th)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    odom.twist.twist.linear.x = (x-prev[0])/time_diff
    odom.twist.twist.linear.y = (y-prev[1])/time_diff
    odom.twist.twist.linear.z = (z-prev[2])/time_diff
    odom.twist.twist.angular.z = (th-prev[3])/time_diff
    odom_pub.publish(odom)


if __name__ == '__main__':

    try:
        init_node()
    except rospy.ROSInterruptException:
        pass
