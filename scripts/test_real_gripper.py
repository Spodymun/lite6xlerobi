#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class RealGripperTest(Node):
    def __init__(self):
        super().__init__('real_gripper_test')
        
        # Publisher für den Gripper Controller
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/lite6_gripper_controller/commands', 
            10
        )
        
        self.get_logger().info("Real Gripper Test gestartet!")
        self.test_gripper()
    
    def send_gripper_command(self, position, description):
        """Sende einen Gripper-Befehl"""
        self.get_logger().info(f"{description} (position: {position})")
        
        msg = Float64MultiArray()
        msg.data = [position]  # Position für lite6_left_finger_joint
        
        self.gripper_pub.publish(msg)
        time.sleep(3)  # Warte auf Bewegung
    
    def test_gripper(self):
        """Teste den echten Gripper mit verschiedenen Positionen"""
        
        # Erweiterte Test-Sequenz für den echten Gripper
        test_positions = [
            (0.0, "Gripper vollständig öffnen"),
            (0.003, "Gripper leicht schließen"),
            (0.006, "Gripper weiter schließen"), 
            (0.010, "Gripper fast geschlossen"),
            (0.015, "Gripper vollständig schließen"),
            (0.007, "Gripper leicht öffnen"),
            (0.0, "Gripper vollständig öffnen")
        ]
        
        for position, description in test_positions:
            self.send_gripper_command(position, description)
        
        self.get_logger().info("Echter Gripper-Test erfolgreich abgeschlossen!")

def main():
    rclpy.init()
    
    try:
        node = RealGripperTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()