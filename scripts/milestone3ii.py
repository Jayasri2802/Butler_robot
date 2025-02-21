#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.kitchen_command = None  # Stores the command from the kitchen
        self.rate = rospy.Rate(1)  # 1 Hz

        # Subscribe to kitchen and table commands
        rospy.Subscriber("/kitchen_command", String, self.kitchen_callback)
        rospy.Subscriber("/table_command", String, self.table_callback)

    def kitchen_callback(self, msg):
        """Callback to receive kitchen commands."""
        self.kitchen_command = msg.data

    def table_callback(self, msg):
        """Callback to receive table commands (if needed)."""
        pass  # Can be used for future features

    def move_to(self, location):
        """Simulates movement of the robot."""
        rospy.loginfo(f"Moving to {location}...")
        rospy.sleep(3)  # Simulate movement delay
        rospy.loginfo(f" Reached {location}")

    def wait_for_command(self, timeout=10):
        """Waits for a command from the kitchen within a timeout."""
        rospy.loginfo(" Waiting for command from the kitchen...")

        for _ in range(timeout):
            if self.kitchen_command:  # Received a command
                rospy.loginfo(f"Received kitchen command: {self.kitchen_command}")
                return self.kitchen_command
            rospy.sleep(1)  # Wait 1 second

        rospy.loginfo(" No command received from the kitchen, returning home.")
        return None

    def execute_task(self):
        """Main function to execute the robot task."""
        # Step 1: Move to Kitchen
        self.move_to("Kitchen")

        # Step 2: Wait for "food ready" command
        command = self.wait_for_command()

        if command == "food ready":
            rospy.loginfo("Collecting food and moving to table...")
            self.move_to("Table")

            # Step 3: Wait at Table for someone to take food
            rospy.loginfo(" Waiting at Table for someone to attend...")
            rospy.sleep(10)  # Simulate waiting

            # Step 4: No one attends → Return food to Kitchen
            rospy.loginfo("No one attended, returning food to Kitchen...")
            self.move_to("Kitchen")

        # Step 5: Finally, return Home
        self.move_to("Home")
        rospy.loginfo("Butler Robot Task Completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()
    except rospy.ROSInterruptException:
        pass

