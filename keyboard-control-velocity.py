#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import sys
import tty
import termios
import threading

def get_key():
    """获取键盘输入"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # 创建发布者
        self.publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )
        
        # 配置参数
        self.max_speed = 10  # 最大速度（可以根据需要调整）
        self.is_running = True
        
        # 获取舵机ID
        self.servo_ids = self.get_servo_ids()
        self.get_logger().info(f'Controlling servos with IDs: {self.servo_ids}')
        
        # 创建关节名称列表
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        # 初始化速度
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        # 创建键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.start()
        
        # 创建定时器，50Hz发布控制命令
        self.timer = self.create_timer(1/50.0, self.timer_callback)

    def get_servo_ids(self):
        """获取用户输入的舵机ID列表"""
        while True:
            try:
                input_str = input("舵机ID(逗号分隔,如 1,2,3,4 分别对应左前,右前,左后,右后): ")
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

    def keyboard_loop(self):
        """键盘监听循环"""
        self.get_logger().info("""
控制键:
    w: 前进
    s: 后退
    a: 左转
    d: 右转
    空格: 停止
    q: 退出
""")
        
        while self.is_running:
            key = get_key()
            
            # 更新速度值
            if key == 'a':  # 左转
                self.left_speed = self.max_speed/4
                self.right_speed = self.max_speed/4
            elif key == 'd':  # 右转
                self.left_speed = -self.max_speed/4
                self.right_speed = -self.max_speed/4
            elif key == 'w':  # 前进
                self.left_speed = -self.max_speed/2
                self.right_speed = self.max_speed/2
            elif key == 's':  # 后退
                self.left_speed = self.max_speed/2
                self.right_speed = -self.max_speed/2
            elif key == ' ':  # 停止
                self.left_speed = 0.0
                self.right_speed = 0.0
            elif key == 'q':  # 退出程序
                self.left_speed = 0.0
                self.right_speed = 0.0
                self.is_running = False
                break

    def timer_callback(self):
        """定时器回调函数，发布速度命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 如果是4个舵机，按左前、右前、左后、右后的顺序
        if len(self.servo_ids) == 4:
            msg.velocity = [
                float(self.left_speed),   # 左前
                float(self.right_speed),  # 右前
                float(self.left_speed),   # 左后
                float(self.right_speed)   # 右后
            ]
        # 如果是2个舵机，按左右顺序
        elif len(self.servo_ids) == 2:
            msg.velocity = [
                float(self.left_speed),   # 左轮
                float(self.right_speed)   # 右轮
            ]
        else:
            # 其他配置，所有舵机使用相同速度
            msg.velocity = [float(self.left_speed)] * len(self.servo_ids)
        
        msg.position = []
        msg.effort = []
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 发送停止命令
        msg = JointState()
        msg.header.stamp = controller.get_clock().now().to_msg()
        msg.name = controller.joint_names
        msg.position = []
        msg.velocity = [0.0] * len(controller.servo_ids)
        msg.effort = []
        
        controller.publisher.publish(msg)
        time.sleep(0.5)  # 等待最后一条消息发送
        
        controller.is_running = False
        controller.keyboard_thread.join()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()