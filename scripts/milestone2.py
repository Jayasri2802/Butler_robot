#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.sleep(1)  # Give time for publisher to connect
        rospy.loginfo("Butler Robot Controller Initialized")

    def move_forward(self, speed=0.2, duration=3):
        """ Moves the robot forward for a specified duration. """
        rospy.loginfo("Moving forward...")
        twist = Twist()
        twist.linear.x = speed

        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)  # 10 Hz

        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop()

    def stop(self):
        """ Stops the robot. """
        rospy.loginfo("Stopping the robot")
        self.cmd_vel_pub.publish(Twist())  # Publish zero velocity

    def wait_for_confirmation(self, location, timeout=5):
        """ Waits at a location for confirmation or timeout. """
        rospy.loginfo(f"Waiting for confirmation at {location} for {timeout} seconds...")
        rospy.sleep(timeout)
        rospy.loginfo(f"Timeout at {location}, moving on")

    def execute_task(self):
        """ Task sequence: Move, wait, move, wait, move """
        rospy.loginfo("Starting task sequence...")

        self.move_forward()
        self.wait_for_confirmation("Kitchen")

        self.move_forward()
        self.wait_for_confirmation("Table")

        self.move_forward()

        rospy.loginfo("Task completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task interrupted")

