#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.kitchen_command = None  # Stores "food ready" command
        self.table_number = None  # Stores the table number
        self.order_cancelled = False  # Tracks if order is canceled

        # Possible tables
        self.tables = ["table1", "table2", "table3"]

    def move_to(self, location):
        """Simulates movement of the robot."""
        rospy.loginfo(f" Moving to {location}...")
        rospy.sleep(3)  # Simulate movement delay
        rospy.loginfo(f" Reached {location}")

    def execute_task(self):
        """Main function to execute multiple orders automatically."""
        
        cancel_order_num = random.randint(1, 3)  # Choose a random order to cancel

        for i in range(1, 4):  # Run for 3 automatic orders
            rospy.loginfo(f"\n Processing Order {i}...\n")

            # Step 1: Get a random table order
            self.table_number = random.choice(self.tables)
            rospy.loginfo(f" New Order Received for {self.table_number}!")

            # Step 2: Move to Kitchen
            self.move_to("Kitchen")

            # Step 3: Simulate "food ready" command (automatic)
            rospy.sleep(2)  # Simulate kitchen delay
            rospy.loginfo(f" Food is ready for {self.table_number}!")

            # Step 4: Move to the table (Check for cancellation)
            rospy.loginfo(f" Delivering food to {self.table_number}...")

            for j in range(3):  # Simulating movement in small steps
                if i == cancel_order_num and j == 1:  # Cancel one random order mid-way
                    self.order_cancelled = True
                    rospy.logwarn(f"Order {i} CANCELLED! Returning food to Kitchen...")
                    self.move_to("Kitchen")  # Return food to kitchen
                    self.move_to("Home")  # Move to home
                    rospy.loginfo(" Butler Robot Task Completed!\n")
                    self.order_cancelled = False  # Reset for next orders
                    break
                rospy.sleep(1)  # Simulating movement delay

            if self.order_cancelled:
                continue  # Skip to the next order

            self.move_to(self.table_number)  # Successfully delivered food

            # Step 5: Move Home
            self.move_to("Home")
            rospy.loginfo(" Butler Robot Task Completed!\n")

        rospy.loginfo(" All orders completed. Robot is now idle.")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()
    except rospy.ROSInterruptException:
        pass

