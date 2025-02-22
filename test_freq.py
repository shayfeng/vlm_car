#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')
        self.last_time = time.time()
        self.count = 0
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.callback,
            10)
            
    def callback(self, msg):
        self.count += 1
        current_time = time.time()
        if current_time - self.last_time >= 1.0:
            self.get_logger().info(f'Messages per second: {self.count}')
            self.count = 0
            self.last_time = current_time

def main():
    rclpy.init()
    tester = PerformanceTester()
    rclpy.spin(tester)
    
if __name__ == '__main__':
    main()
