#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

URDF = "/tmp/lite6_fake_hw.urdf"

def main():
    rclpy.init()
    n = Node('robot_description_publisher')
    q = QoSProfile(depth=1)
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    q.reliability = QoSReliabilityPolicy.RELIABLE
    pub = n.create_publisher(String, '/robot_description', q)
    with open(URDF) as f:
        data = f.read()
    msg = String(data=data)
    for _ in range(5):
        pub.publish(msg)
        time.sleep(0.1)
    n.get_logger().info(f"republished /robot_description from {URDF}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
