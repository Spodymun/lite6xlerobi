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
        goal.command.max_effort = 15.0  # Gleiche max_effort wie real hardware
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info(f"Command accepted: {description}")
            
            # Warte auf Completion
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            
            if result.result.reached_goal:
                self.get_logger().info(f"Goal reached: {description}")
            else:
                self.get_logger().warn(f"Goal not reached: {description}")
        else:
            self.get_logger().warn(f"Command rejected: {description}")
        
        time.sleep(3)  # Mehr Zeit für realistische Bewegung
    
    def test_gripper(self):
        """Test the gripper functionality with different positions"""
        
        # Erweiterte Test-Sequenz für realistisches Verhalten
        test_positions = [
            (0.0, "Gripper vollständig öffnen"),
            (0.005, "Gripper leicht schließen"),
            (0.010, "Gripper weiter schließen"), 
            (0.015, "Gripper vollständig schließen"),
            (0.0075, "Gripper zur Hälfte öffnen"),
            (0.0, "Gripper vollständig öffnen")
        ]
        
        for position, description in test_positions:
            self.send_gripper_command(position, description)
        
        self.get_logger().info("Realistische Gripper-Test erfolgreich abgeschlossen!")

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