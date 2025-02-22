#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import os
import base64
import re
import math
import time
import sys
import threading

class VisionControlledRobot(Node):
    def __init__(self):
        super().__init__('vision_controlled_robot')
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 创建OpenAI客户端
        self.client = OpenAI(
            api_key=os.getenv('DASHSCOPE_API_KEY'),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        # 创建发布者和订阅者
        self.publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )
        
        # 订阅相机话题
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # 根据实际相机话题修改
            self.camera_callback,
            10
        )
        
        # 配置参数
        self.max_speed = 10  # 最大速度
        self.is_running = True
        self.latest_frame = None
        
        # 获取舵机ID并设置joint names
        self.servo_ids = self.get_servo_ids()
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        # 初始化速度
        self.left_speed = 0.0
        self.right_speed = 0.0
        
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

    def camera_callback(self, msg):
        """处理相机图像回调"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def encode_image(self, image):
        """将OpenCV图像编码为base64"""
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode('utf-8')

    def get_vision_command(self, target_object):
        """调用视觉模型API获取导航命令"""
        if self.latest_frame is None:
            self.get_logger().warning('No camera frame available')
            return None

        try:
            # 编码图像
            base64_image = self.encode_image(self.latest_frame)
            
            # 调用API
            completion = self.client.chat.completions.create(
                model="qwen-vl-max-2025-01-25",
                messages=[
                    {
                        "role": "system",
                        "content": [{"type":"text","text": "你是移动机器人的导航助手。\
现在，给定摄像头返回的图像和人类指令，当提供图像和导航指令时，请严格按照以下格式输出：\
你可以判断一下是先转弯还是先移动，然后输出最终指令。\
move: <距离，单位为厘米> turn: <角度，单位为度，左转为负> or turn:<角度，单位为度，左转为负> move:<距离，单位为厘米>"}]
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                            },
                            {"type": "text", "text": f"前往{target_object}"},
                        ],
                    }
                ],
                top_p=0.1,
            )
            
            return completion.choices[0].message.content
            
        except Exception as e:
            self.get_logger().error(f'Error calling vision API: {str(e)}')
            return None

    def parse_model_response(self, response):
        """解析大模型的响应"""
        if not response:
            return 0, 0
            
        # 使用正则表达式提取move和turn的值
        move_match = re.search(r'move:\s*(-?\d+)', response)
        turn_match = re.search(r'turn:\s*(-?\d+)', response)
        
        move_value = int(move_match.group(1)) if move_match else 0
        turn_value = int(turn_match.group(1)) if turn_match else 0
        
        return move_value, turn_value

    def execute_command(self, move_value, turn_value):
        """执行移动和转向命令"""
        if turn_value != 0:
            # 执行转向
            # 转向速度设置为最大速度的1/4
            turn_speed = self.max_speed/4
            if turn_value < 0:  # 左转
                self.left_speed = turn_speed
                self.right_speed = turn_speed
            else:  # 右转
                self.left_speed = -turn_speed
                self.right_speed = -turn_speed
            
            # 根据角度计算转向时间（这需要根据实际机器人调整）
            turn_time = abs(turn_value) / 90.0  # 假设90度需要1秒
            time.sleep(turn_time)
            
            # 停止转向
            self.left_speed = 0.0
            self.right_speed = 0.0

        if move_value > 0:
            # 执行直线移动
            # 设置为最大速度的1/2进行直线移动
            self.left_speed = -self.max_speed/2
            self.right_speed = self.max_speed/2
            
            # 根据距离计算移动时间（这需要根据实际机器人调整）
            move_time = move_value / 50.0  # 假设50cm需要1秒
            time.sleep(move_time)
            
            # 停止移动
            self.left_speed = 0.0
            self.right_speed = 0.0

    def timer_callback(self):
        """定时器回调函数，发布速度命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 根据舵机数量设置速度
        if len(self.servo_ids) == 4:
            msg.velocity = [
                float(self.left_speed),   # 左前
                float(self.right_speed),  # 右前
                float(self.left_speed),   # 左后
                float(self.right_speed)   # 右后
            ]
        elif len(self.servo_ids) == 2:
            msg.velocity = [
                float(self.left_speed),   # 左轮
                float(self.right_speed)   # 右轮
            ]
        else:
            msg.velocity = [float(self.left_speed)] * len(self.servo_ids)
        
        msg.position = []
        msg.effort = []
        
        self.publisher.publish(msg)

def main():
    rclpy.init()
    robot = VisionControlledRobot()
    
    try:
        while True:
            # 获取用户输入的目标物体
            target_object = input("请输入目标物体(输入'q'退出): ")
            if target_object.lower() == 'q':
                break
                
            # 获取视觉命令
            model_response = robot.get_vision_command(target_object)
            if model_response:
                print(f"模型响应: {model_response}")
                # 解析并执行命令
                move_value, turn_value = robot.parse_model_response(model_response)
                print(f"解析结果 - 移动: {move_value}cm, 转向: {turn_value}度")
                robot.execute_command(move_value, turn_value)
            else:
                print("未能获取有效的导航命令")
        
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        # 发送停止命令
        msg = JointState()
        msg.header.stamp = robot.get_clock().now().to_msg()
        msg.name = robot.joint_names
        msg.position = []
        msg.velocity = [0.0] * len(robot.servo_ids)
        msg.effort = []
        
        robot.publisher.publish(msg)
        time.sleep(0.5)  # 等待最后一条消息发送
        
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()