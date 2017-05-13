#!/usr/bin/env python

# import sys
import rospy
from std_msgs.msg import Float32MultiArray 
import math
import tf

class move:

    def __init__(self):

        self.boat_pub = rospy.Publisher('/boat/set_pose', Float32MultiArray, queue_size=1)

        self.cmd = Float32MultiArray()
        self.cmd.data = [0.0, 0.0, 0.0, 0.0]

        self.update_rate = 100.0
        self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update)

        self.start_time = rospy.get_time()
        self.omega = 1.0/5.0

    def update(self, event):
    	self.now_time = rospy.get_time()
        # self.cmd.data[2] += 0.01
        time = self.now_time - self.start_time
        self.cmd.data[0] = 1.0*math.sin(time*self.omega)

        self.boat_pub.publish(self.cmd)

if __name__ == "__main__":
    
    rospy.init_node('boat_cmd')

    boat = move()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()