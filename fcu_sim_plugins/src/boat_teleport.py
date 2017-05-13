#!/usr/bin/env python

# import sys
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float32MultiArray 
import tf

class teleport:

    def __init__(self):
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=1)
        except rospy.ROSException:
            print "no teleport service available"

        self._vehicle_state_sub = rospy.Subscriber('/boat/set_pose', Float32MultiArray, self.cmd_callback)

        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # self.tele = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, persistent=True)
        self.tele = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        self.state = ModelState()
        self.state.model_name = 'boat'

        state_init = self.get_state(self.state.model_name,'')

        # print 'state inited'

        self.state.pose.position.x = state_init.pose.position.x
        self.state.pose.position.y = state_init.pose.position.y
        self.state.pose.position.z = state_init.pose.position.z
        self.state.pose.orientation.x = state_init.pose.orientation.x
        self.state.pose.orientation.y = state_init.pose.orientation.y
        self.state.pose.orientation.z = state_init.pose.orientation.z
        self.state.pose.orientation.w = state_init.pose.orientation.w

        self.update_rate = 10.0
        self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.move)

    def move(self, event):
        # self.state.pose.position.z += 1
        resp = self.tele.publish(self.state)
        # print resp.success

    def cmd_callback(self, msg):
        # roll, pitch, yaw, forward position
        roll = msg.data[0]
        pitch = msg.data[1]
        yaw = msg.data[2]
        # print roll

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.state.pose.orientation.x = quaternion[0]
        self.state.pose.orientation.y = quaternion[1]
        self.state.pose.orientation.z = quaternion[2]
        self.state.pose.orientation.w = quaternion[3]

        self.position = msg.data[3]

if __name__ == "__main__":
    
    rospy.init_node('boat_teleport')

    tele = teleport()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()