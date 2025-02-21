#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ButlerRobot:
    def __init__(self):
        rospy.init_node('butler_robot')
        self.cmd_vel_pub = rospy.Publisher('/butler_robot/cmd_vel', Twist, queue_size=10)
        self.task_sub = rospy.Subscriber('/task_command', String, self.task_callback)

    def move_forward(self, speed=0.2, duration=2):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(duration)
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def task_callback(self, msg):
        rospy.loginfo(f"Received order for {msg.data}")
        rospy.loginfo("Moving to Kitchen")
        self.move_forward()
        rospy.loginfo("Moving to Table")
        self.move_forward()
        rospy.loginfo("Returning to Home")
        self.move_forward()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    robot = ButlerRobot()
    robot.run()
