#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':
    rospy.init_node('joint_controller')

    pub_body = rospy.Publisher("/fish/ventral_body_position_controller/command", Float64, queue_size=1)
    pub_fin = rospy.Publisher("/fish/caudal_fin_position_controller/command", Float64, queue_size=1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        t = rospy.get_time()
        pub_body.publish(Float64(0.5 * np.sin(t)))
        pub_fin.publish(Float64(0.5 * np.cos(t)))
        rate.sleep()
