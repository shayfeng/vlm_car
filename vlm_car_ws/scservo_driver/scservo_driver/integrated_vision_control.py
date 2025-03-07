import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import os
import base64
import re
import math
import time
import threading

class IntegratedVisionControl(Node):
    def __init__(self):
        super().__init__('integrated_vision_control')
        
        # 相机参数
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        
        # 获取相机参数
        self.device_id = self.get_parameter('device_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 初始化相机
        self.init_camera()
        
        # 创建OpenAI客户端
        self.client = OpenAI(
            api_key=os.getenv('DASHSCOPE_API_KEY'),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        # 创建发布者
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )
        
        # 配置参数
        self.max_speed = 10  # 根据舵机的最大速度值修改
        self.is_running = True
        self.latest_frame = None
        
        # 舵机配置
        self.servo_ids = [1, 2, 3, 4]  # 使用的舵机ID
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        # 初始化速度
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        # 移动和转向的速度比例
        self.move_speed_ratio = 1  # 移动时使用最大速度的50%
        self.turn_speed_ratio = 0.25  # 转向时使用最大速度的25%
        
        # 创建定时器，50Hz发布控制命令
        self.timer = self.create_timer(1/50.0, self.timer_callback)
        
        # 创建相机定时器
        self.camera_timer = self.create_timer(1.0/self.fps, self.camera_callback)
        
        # 状态标志
        self.is_executing_command = False
        
        self.get_logger().info(
            f'Initialized with camera device {self.device_id} at {self.frame_width}x{self.frame_height} @ {self.fps}fps'
        )

    def init_camera(self):
        """初始化相机设备"""
        self.get_logger().info(f'Attempting to open camera device {self.device_id}')
        self.cap = cv2.VideoCapture(self.device_id)
        
        # 添加重试逻辑
        retry_count = 0
        max_retries = 3
        while not self.cap.isOpened() and retry_count < max_retries:
            self.get_logger().warn(f'Failed to open camera, retrying... ({retry_count + 1}/{max_retries})')
            time.sleep(1)
            self.cap = cv2.VideoCapture(self.device_id)
            retry_count += 1
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_id}')
            raise RuntimeError(f'Failed to open camera device {self.device_id}')
            
        # 设置分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        # 设置帧率
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 验证设置是否生效
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(
            f'Camera initialized with resolution: {actual_width}x{actual_height} @ {actual_fps}fps'
        )

    def camera_callback(self):
        """相机定时器回调函数，读取图像"""
        if not self.cap.isOpened():
            self.get_logger().error('Camera is not opened')
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
            
        if frame is None:
            self.get_logger().error('Captured frame is None')
            return
            
        self.latest_frame = frame
        # 打印图像的形状以确认数据正确
        # self.get_logger().info(f'Captured frame with shape: {frame.shape}')

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
            base64_image = self.encode_image(self.latest_frame)
            
            completion = self.client.chat.completions.create(
                model="qwen-vl-max-2025-01-25",
                messages=[
                    {
                        "role": "system",
                        "content": [{"type":"text","text": "你是移动机器人的导航助手。\
现在，给定摄像头返回的图像和人类指令，当提供图像和导航指令时，请严格按照以下格式输出：\
你可以判断一下是先转弯还是先移动，然后输出最终指令。\
move: <距离，单位为厘米> turn: <角度，单位为度，左转为负> or turn:<角度，单位为度，左转为负> move:<距离，单位为厘米> \
\ 当检测不到目标物体时,请输出move: 0 turn: 0\
示例：\
\
    向前移动 50 厘米，左转 90 度:move: 50 turn: -90\
    直行 30 厘米，右转 45 度:move: 30 turn: 45\
    左转 45 度，向前移动 50 厘米:turn: -45 move: 50\
\
请先仔细分析场景，以及到达目标距离和角度，然后输出最终指令。注意：仅输出最终结果。\
\
图像场景描述：\
机器人正在一张(宽1m2,长3m)木质桌面上导航\
\
提示：\
\
    如果目标位于图像中心，则转向角度为 0 度。\
    如果目标位于图像最左侧，则转向角度为 -x 度（最大为 -90 度）。\
    如果目标位于图像最右侧，则转向角度为 x 度（最大为 90 度）。\
      角度一般与目标物和图像中心像素点相对位置有关。\
    距离一般与目标物在图像中的像素点分布有关。\
    位于中心和边缘之间的目标通常为 ±45 度。\
    在确定移动和转向角度时，请注意桌面上的障碍物。"}]
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
            print(completion.choices[0].message.content)
            return completion.choices[0].message.content
            
        except Exception as e:
            self.get_logger().error(f'Error calling vision API: {str(e)}')
            return None

    def execute_command(self, move_value, turn_value):
        """执行移动和转向命令"""
        self.is_executing_command = True
        
        try:
            if turn_value != 0:
                # 执行转向
                turn_speed = int(self.max_speed * self.turn_speed_ratio)
                if turn_value < 0:  # 左转
                    self.publish_velocity(turn_speed, turn_speed)
                else:  # 右转
                    self.publish_velocity(-turn_speed, -turn_speed)
                
                # 根据角度计算转向时间
                turn_time = abs(turn_value) / 100 # 假设90度需要1秒
                time.sleep(turn_time)
                print("turn time:")
                print(turn_time)
                self.publish_velocity(0, 0)  # 停止转向

            if move_value > 0:
                # 执行直线移动
                move_speed = int(self.max_speed * self.move_speed_ratio)
                self.publish_velocity(-move_speed, move_speed)  # 左轮负值，右轮正值表示前进
                
                # 根据距离计算移动时间
                move_time = move_value / 20  # 假设50cm需要1秒
                time.sleep(move_time)
                print("move_time")
                print(move_time)
                self.publish_velocity(0, 0)  # 停止移动
        
        finally:
            self.is_executing_command = False

    def publish_velocity(self, left_speed, right_speed):
        """发布速度命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []  # 空列表表示不控制位置
        # 设置四个舵机的速度：舵机1和3是左侧，舵机2和4是右侧
        msg.velocity = [float(left_speed), float(right_speed), float(left_speed), float(right_speed)]
        msg.effort = []
        self.joint_publisher.publish(msg)

    def timer_callback(self):
        """定时器回调函数"""
        # 只在没有执行命令时发布0速度
        if not self.is_executing_command:
            self.publish_velocity(0, 0)

    def on_shutdown(self):
        """关闭时释放相机资源"""
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = IntegratedVisionControl()
    
    # 创建一个线程来处理ROS2的回调
    def spin_thread():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    try:
        while True:
            # 获取用户输入的目标物体
            target_object = input("请输入目标物体(输入'q'退出): ")
            if target_object.lower() == 'q':
                break
            
            if node.is_executing_command:
                print("正在执行上一个命令，请稍后...")
                continue
                
            # 等待获取有效的图像帧
            retry_count = 0
            max_retries = 10  # 最多等待10次
            while node.latest_frame is None and retry_count < max_retries:
                print(f"等待相机图像... ({retry_count + 1}/{max_retries})")
                time.sleep(0.5)
                retry_count += 1
                
            if node.latest_frame is None:
                print("无法获取相机图像，请检查相机连接")
                continue
                
            # 获取视觉命令
            model_response = node.get_vision_command(target_object)
            if model_response:
                print(f"模型响应: {model_response}")
                # 解析并执行命令
                move_match = re.search(r'move:\s*(-?\d+)', model_response)
                turn_match = re.search(r'turn:\s*(-?\d+)', model_response)
                
                move_value = int(move_match.group(1)) if move_match else 0
                turn_value = int(turn_match.group(1)) if turn_match else 0
                
                print(f"解析结果 - 移动: {move_value}cm, 转向: {turn_value}度")
                node.execute_command(move_value, turn_value)
            else:
                print("未能获取有效的导航命令")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保停止运动
        node.publish_velocity(0, 0)
        time.sleep(0.1)  # 等待最后一条消息发送
        
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()