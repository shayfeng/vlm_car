#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            JointState, 
            'joint_command',
            10
        )
        
        # 获取用户输入的舵机ID列表
        self.servo_ids = self.get_servo_ids()
        self.get_logger().info(f'Testing servos with IDs: {self.servo_ids}')
        
        # 初始化参数 - 使用弧度单位
        self.center_position = 0  # 中心位置 (0 弧度 = 2048 刻度)
        self.amplitude = math.pi/4  # 摆动幅度 (±π/4 弧度，约±45度)
        self.velocity_amplitude = 10 * math.pi  # 速度模式下的幅度 (±π rad/s)
        self.start_time = time.time()
        
        # 获取控制模式并创建关节名称列表
        self.control_mode = self.get_control_mode()
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        # 创建定时器，50Hz
        self.timer = self.create_timer(1/50.0, self.timer_callback)

    def get_servo_ids(self):
        """获取用户输入的舵机ID列表"""
        while True:
            try:
                input_str = input("舵机ID(逗号分隔,如 1,2,3): ")
                ids = [int(x.strip()) for x in input_str.split(',')]
                if not ids:
                    print("错误：至少需要一个舵机ID")
                    continue
                if not all(1 <= x <= 253 for x in ids):
                    print("错误：舵机ID必须在1-253范围内")
                    continue
                return ids
            except ValueError:
                print("错误：请输入有效的数字，用逗号分隔")

    def get_control_mode(self):
        """获取控制模式选择"""
        while True:
            mode = input("控制模式(p=位置/v=速度): ").lower()
            if mode == 'p':
                self.get_logger().info("位置控制模式")
                return "position"
            elif mode == 'v':
                self.get_logger().info("速度控制模式")
                return "velocity"
            print("错误：请输入 'p' 或 'v'")

    def timer_callback(self):
        current_time = time.time() - self.start_time
        # 使用正弦函数生成平滑的往复运动，周期为4秒
        angle = math.sin(2 * math.pi * current_time / 4.0)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        if self.control_mode == "position":
            position = self.center_position + self.amplitude * angle
            msg.position = [float(position)] * len(self.servo_ids)
            msg.velocity = []
            msg.effort = []
            self.get_logger().info(f'Position: {position:.2f} rad')
        else:  # velocity mode
            velocity = self.velocity_amplitude * angle
            msg.position = []
            msg.velocity = [float(velocity)] * len(self.servo_ids)
            msg.effort = []
            self.get_logger().info(f'Velocity: {velocity:.2f} rad/s')
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    tester = ServoTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n停止测试...")
    finally:
        # 停止时将舵机恢复到中心位置或停止
        msg = JointState()
        msg.header.stamp = tester.get_clock().now().to_msg()
        msg.name = tester.joint_names
        
        if tester.control_mode == "position":
            msg.position = [math.pi] * len(tester.servo_ids)  # 中心位置 (π 弧度)
            msg.velocity = []
            msg.effort = []
        else:  # velocity mode
            msg.position = []
            msg.velocity = [0.0] * len(tester.servo_ids)
            msg.effort = []
        
        tester.publisher.publish(msg)
        time.sleep(0.5)  # 等待最后一条消息发送
        
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
