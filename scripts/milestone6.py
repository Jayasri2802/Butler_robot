#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.tables = []  # List of tables for delivery
        self.confirmed_tables = set()  # Stores confirmed tables
        self.kitchen_command_received = False  # Flag to check if we got the kitchen command

        # Subscribe to kitchen commands
        rospy.Subscriber("/kitchen_command", String, self.kitchen_callback)

    def kitchen_callback(self, msg):
        """Receives kitchen command with table numbers."""
        if "food ready:" in msg.data:
            self.tables = msg.data.replace("food ready:", "").split(',')
            self.tables = [table.strip() for table in self.tables]  # Remove spaces
            self.kitchen_command_received = True
            rospy.loginfo(f" Orders received for tables: {self.tables}")

    def move_to(self, location):
        """Simulates movement of the robot."""
        rospy.loginfo(f" Moving to {location}...")
        rospy.sleep(3)  # Simulate movement delay
        rospy.loginfo(f" Reached {location}")

    def wait_for_confirmation(self, location, timeout=10):
        """
        Simulates a customer confirming at exactly two tables.
        """
        rospy.loginfo(f" Waiting for confirmation at {location} (Timeout: {timeout}s)")

        # Pick exactly two random tables at the start of the process (only once)
        if not self.confirmed_tables:  
            self.confirmed_tables = set(random.sample(self.tables, min(2, len(self.tables))))  # Pick 2 or less

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if location in self.confirmed_tables:  # If table is one of the confirmed ones
                rospy.loginfo(f"Order confirmed and delivered at {location}!")
                return True
            rospy.sleep(1)

        rospy.loginfo(f" No confirmation at {location}, skipping...")
        return False

    def execute_task(self):
        """Main function to execute the robot task."""

        rospy.loginfo(" Waiting for 'food ready' command from kitchen...")
        while not self.kitchen_command_received:  # Keep waiting until command is received
            rospy.sleep(1)

        if not self.tables:
            rospy.loginfo(" No tables received. Returning home.")
            self.move_to("Home")
            return

        # Step 1: Move to Kitchen
        self.move_to("Kitchen")

        # Step 2: Deliver to each table
        for table in self.tables:
            self.move_to(table)
            if not self.wait_for_confirmation(table):
                continue  # Skip to next table if no one attends

        # Step 3: After last table → Return to Kitchen
        rospy.loginfo(" Delivery completed! Returning to Kitchen...")
        self.move_to("Kitchen")

        # Step 4: Return Home
        rospy.loginfo("Returning to Home...")
        self.move_to("Home")

        rospy.loginfo(" Butler Robot Task Completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()  # This will now run correctly
        rospy.spin()  # Keeps node alive for future commands if needed
    except rospy.ROSInterruptException:
        pass

