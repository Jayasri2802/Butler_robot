#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.tables = []  # List of tables for delivery
        self.canceled_table = "table2"  # Table to skip due to cancellation
        self.kitchen_command_received = False  # Flag to check if kitchen command is received

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

        # Step 2: Deliver to each table, skipping the canceled one
        for table in self.tables:
            if table == self.canceled_table:
                rospy.loginfo(f" {table} order canceled. Skipping.")
                continue  # Skip the canceled table
            
            self.move_to(table)
            rospy.loginfo(f" Delivered order to {table}")

        # Step 3: After last table → Return to Kitchen
        rospy.loginfo(" Delivery completed! Returning to Kitchen...")
        self.move_to("Kitchen")

        # Step 4: Return Home
        rospy.loginfo(" Returning to Home...")
        self.move_to("Home")

        rospy.loginfo(" Butler Robot Task Completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()  # Run the task
        rospy.spin()  # Keeps the node alive
    except rospy.ROSInterruptException:
        pass

