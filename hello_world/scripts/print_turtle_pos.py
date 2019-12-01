#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

def print_turtle_pos():
	rospy.init_node('print_turtle_pos', anonymous=False)
	rospy.Subscriber("/turtle1/pose", Pose, callback)
	rospy.spin()

def callback(data):
	rospy.loginfo("Turtle Pose X : %s, Y : %s", data.x, data.y)

if __name__ == '__main__':
	print_turtle_pos()