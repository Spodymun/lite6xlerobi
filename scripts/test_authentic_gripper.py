#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

class AuthenticGripperTest(Node):
    def __init__(self):
        super().__init__('authentic_gripper_test')
        
        # Action client for the authentic gripper controller
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/authentic_lite6_gripper_controller/gripper_cmd'
        )
        
        self.get_logger().info("Waiting for gripper action server...")
        if self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Gripper action server found!")
            self.test_gripper()
        else:
            self.get_logger().error("Gripper action server not available!")
    
    def send_gripper_command(self, position, description):
        """Send a gripper command and wait for completion"""
        self.get_logger().info(f"{description} (position: {position})")
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().accepted:
            self.get_logger().info(f"Command accepted: {description}")
        else:
            self.get_logger().warn(f"Command rejected: {description}")
        
        time.sleep(2)  # Wait for movement to complete
    
    def test_gripper(self):
        """Test the gripper functionality with different positions"""
        
        # Test sequence: open -> close -> half open
        test_positions = [
            (0.0, "Opening gripper"),
            (0.015, "Closing gripper"), 
            (0.0075, "Half opening gripper")
        ]
        
        for position, description in test_positions:
            self.send_gripper_command(position, description)
        
        self.get_logger().info("Gripper test completed successfully!")

def main():
    rclpy.init()
    
    try:
        node = AuthenticGripperTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()