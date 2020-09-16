#!/usr/bin/env python

import rospy
import os
import time
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
test_name = "rotate"

dir = os.path.join("/home/arjun/quadruped_ws/src/quadruped_driver/results_csvs", test_name)
if os.path.isdir(dir):
    print ("Directory Exists")
    exit()
os.mkdir(dir)
gtf_n = os.path.join(dir, "ground_truth.csv")
gtf = open(gtf_n, "w")

def init_node():
    rospy.init_node('ground_pose_listener', anonymous=True)
    twist = rospy.wait_for_message('cmd_vel', Twist, timeout=None)
    write_twist(twist)
    tstart = get_sim_time()
    rospy.Subscriber("/quadruped/base_pose_ground_truth", Odometry, write_odom)
    return tstart

def get_sim_time():
    t = rospy.wait_for_message('/clock', Time, timeout=None)
    t_sec = t.clock.to_sec()
    return t_sec

def write_twist(twist):
    v = twist.linear.x
    ome = twist.angular.z
    outname = os.path.join(dir, "twist.csv")
    f = open(outname, "w")
    f.write(str(v) + "\n")
    f.write(str(ome))
    f.close()

def write_sim_time(start, end):
    outname = os.path.join(dir, "sim_time.csv")
    f = open(outname, "w")
    f.write(str(start) + "\n")
    f.write(str(end))
    f.close()

def write_odom(data):
    t = str(data.header.stamp.to_sec())
    x = str(data.pose.pose.position.x)
    y = str(data.pose.pose.position.y)
    z = str(data.pose.pose.position.z)
    qx = str(data.pose.pose.orientation.x)
    qy = str(data.pose.pose.orientation.y)
    qz = str(data.pose.pose.orientation.z)
    qw = str(data.pose.pose.orientation.w)

    strf = t+","+x+","+y+","+z+","+qx+","+qy+","+qz+","+qw
    gtf.write(strf + "\n")

if __name__ == '__main__':

    tstart = init_node()
    ts = time.time()
    while(time.time()-ts<500):
        b = input()
        if b == 0:
            break
    tend = get_sim_time()
    write_sim_time(tstart, tend)
    print([tstart, tend])
    gtf.close()
