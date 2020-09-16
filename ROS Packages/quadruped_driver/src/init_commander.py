#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_list = ["leg1_motor_1","leg1_motor_2","leg2_motor_1","leg2_motor_2","leg3_motor_1","leg3_motor_2","leg4_motor_1", "leg4_motor_2"]

def get_current_state():
    rospy.init_node('command_publisher', anonymous=True)
    joint_state = rospy.wait_for_message('/quadruped/joint_states', JointState, timeout=None)
    return joint_state

def pub_init_pos(joint_state):
    pub_list = []
    steps = 10
    rate = rospy.Rate(5)
    for joint in joint_list:
        topic = "quadruped/" + joint + "_position_controller/command"
        pub = rospy.Publisher(topic, Float64, queue_size=10)
        pub_list.append(pub)

    for j in range(steps):
        for i in range(len(joint_list)):
            pub = pub_list[i]
            pub.publish(joint_state.position[i]*(steps - j - 1)/steps)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state = get_current_state()
        pub_init_pos(joint_state)
    except rospy.ROSInterruptException:
        pass
