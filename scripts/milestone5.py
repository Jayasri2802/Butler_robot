#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ButlerRobot:
    def __init__(self):
        rospy.init_node("butler_robot_controller", anonymous=True)

        self.table_orders = []  # Stores multiple table numbers
        self.kitchen_command_received = False  # Flag for "food ready" command

        self.rate = rospy.Rate(1)  # 1 Hz

        # Subscriber for receiving table numbers from the kitchen
        rospy.Subscriber("/kitchen_command", String, self.kitchen_callback)

    def kitchen_callback(self, msg):
        """Callback to receive kitchen commands (table numbers + 'food ready')."""
        data = msg.data.split(":")
        if len(data) == 2 and data[0] == "food ready":
            self.table_orders = data[1].split(',')  # Extract table numbers
            self.kitchen_command_received = True
            rospy.loginfo(f" Food Ready! Orders received for: {', '.join(self.table_orders)}")

    def move_to(self, location):
        """Simulates movement of the robot."""
        rospy.loginfo(f" Moving to {location}...")
        rospy.sleep(3)  # Simulate movement delay
        rospy.loginfo(f"Reached {location}")

    def wait_for_kitchen_command(self, timeout=10):
        """Waits for kitchen to send the table numbers with 'food ready'."""
        rospy.loginfo(" Waiting for kitchen to confirm 'food ready' and table orders...")

        for _ in range(timeout):
            if self.kitchen_command_received:  # Kitchen sent the order details
                return True
            rospy.sleep(1)  # Wait 1 second

        rospy.loginfo(" No kitchen response, returning home.")
        return False

    def execute_task(self):
        """Main function to execute multiple deliveries."""
        # Step 1: Move to Kitchen
        self.move_to("Kitchen")

        # Step 2: Wait for kitchen to send "food ready" and table numbers
        if not self.wait_for_kitchen_command():
            self.move_to("Home")
            return

        # Step 3: Deliver food to each table
        for table in self.table_orders:
            rospy.loginfo(f"Delivering food to {table}...")
            self.move_to(table)
            rospy.loginfo(f"Food delivered to {table}!")

        # Step 4: Return Home
        self.move_to("Home")
        rospy.loginfo("Butler Robot Task Completed!")

if __name__ == "__main__":
    try:
        robot = ButlerRobot()
        robot.execute_task()
    except rospy.ROSInterruptException:
        pass

