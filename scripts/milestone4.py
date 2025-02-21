#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)
        self.kitchen_command = None  # Stores kitchen command ("food ready")
        self.table_number = None  # Stores the table number
        self.order_cancelled = False  # Tracks if order is canceled

        rospy.Subscriber("/kitchen_command", String, self.kitchen_callback)
        rospy.Subscriber("/table_command", String, self.table_callback)
        rospy.Subscriber("/order_cancelled", String, self.cancel_callback)

    def kitchen_callback(self, msg):
        """Receives 'food ready' command from kitchen."""
        self.kitchen_command = msg.data

    def table_callback(self, msg):
        """Receives table number (e.g., 'table3') at the start."""
        self.table_number = msg.data

    def cancel_callback(self, msg):
        """Receives 'order cancelled' command from the table."""
        if msg.data == "order cancelled":
            self.order_cancelled = True

    def move_to(self, location):
        """Simulates movement of the robot."""
        rospy.loginfo(f" Moving to {location}...")
        rospy.sleep(3)  # Simulate movement delay
        rospy.loginfo(f" Reached {location}")

    def wait_for_command(self, attr_name, timeout=10):
        """Waits for a specific command (kitchen or table) within a timeout."""
        rospy.loginfo(f" Waiting for {attr_name} command...")

        for _ in range(timeout):
            if getattr(self, attr_name):  # If the attribute is set (command received)
                rospy.loginfo(f"Received {attr_name}: {getattr(self, attr_name)}")
                return getattr(self, attr_name)
            rospy.sleep(1)  # Wait 1 second

        rospy.loginfo(f" No {attr_name} received, returning home.")
        return None

    def execute_task(self):
        """Main function to execute the robot task."""

        # Step 1: Wait for a table number (10s timeout)
        self.table_number = self.wait_for_command("table_number")
        if not self.table_number:
            self.move_to("Home")
            return

        # Step 2: Move to Kitchen
        self.move_to("Kitchen")

        # Step 3: Wait for "food ready" command (10s timeout)
        command = self.wait_for_command("kitchen_command")
        if command != "food ready":
            self.move_to("Home")
            return

        # Step 4: Move to the respective table, checking for cancellation mid-way
        rospy.loginfo(f" Collecting food and moving to {self.table_number}...")

        for _ in range(10):  # Simulate movement in small steps (10 loops = 10s)
            if self.order_cancelled:  # If cancellation happens mid-way
               rospy.logwarn("Order Cancelled! Stopping and returning food to Kitchen...")
               self.move_to("Kitchen")  # Return food to kitchen
               self.move_to("Home")  # Move to home
               rospy.loginfo("Butler Robot Task Completed!")
               return  # End task immediately

            rospy.sleep(1)  # Simulating movement delay per step

        # If no cancellation, the robot reaches the table
        rospy.loginfo(f"Reached {self.table_number}")


        # Step 5: Move Home
        self.move_to("Home")
        rospy.loginfo(" Butler Robot Task Completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()
    except rospy.ROSInterruptException:
        pass

