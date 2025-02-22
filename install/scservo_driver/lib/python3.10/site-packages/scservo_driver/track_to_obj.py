import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import os, sys, time, threading, select, termios, tty, re, math, base64

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
        self.current_target = None
        self.switch_target_flag = False
        self.stop_tracking = False
        # 舵机配置
        self.servo_ids = [1, 2, 3, 4]  # 使用的舵机ID
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        # 初始化速度
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        # 移动和转向的速度比例
        self.move_speed_ratio = 0.5 # 移动时使用最大速度的50%
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
机器人正在一张(宽1m2,长3m)木质桌面上导航，桌面为深棕色纹理表面，类似木纹。桌面上摆放了各种物品，包括一个塑料水瓶、笔记本电脑、卷曲的充电线等。\
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
            # 记录是否在连续移动模式
            self.in_continuous_mode = move_value >= 100
            
            if move_value < 100:
                self.is_executing_command = True
            
            try:
                if turn_value != 0:
                    # 执行转向
                    turn_speed = int(self.max_speed * self.turn_speed_ratio)
                    if turn_value < 0:  # 左转
                        self.publish_velocity(turn_speed, turn_speed)
                    else:  # 右转
                        self.publish_velocity(-turn_speed, -turn_speed)
                    
                    turn_time = abs(turn_value) / 100 
                    time.sleep(turn_time)
                    print(f"转向时间: {turn_time}秒")
                    self.publish_velocity(0, 0)  # 停止转向

                if move_value > 0:
                    # 如果距离小于等于20cm，认为已到达目标
                    if move_value <= 20:
                        print("已到达目标位置!")
                        self.in_continuous_mode = False
                        return True  # 返回True表示已到达目标
                    
                    # 设置移动速度
                    move_speed = int(self.max_speed * self.move_speed_ratio)
                    
                    if move_value >= 100:
                        # 进入连续移动模式
                        print("目标距离大于等于100cm，进入连续移动模式")
                        self.publish_velocity(-move_speed, move_speed)
                        # 不等待，直接返回以便继续获取新的命令
                        return False
                    elif move_value < 50:
                        # 恢复正常移动逻辑
                        print("目标距离小于50cm，使用精确移动模式")
                        self.publish_velocity(-move_speed, move_speed)
                        move_time = move_value / 40
                        time.sleep(move_time)
                        print(f"移动时间: {move_time}秒")
                        self.publish_velocity(0, 0)
                    else:
                        # 50-100cm之间的过渡区域，使用较短的移动时间
                        print("目标距离在过渡区域，使用中等移动距离")
                        self.publish_velocity(-move_speed, move_speed)
                        move_time = move_value / 60  # 使用更快的移动速度
                        time.sleep(move_time)
                        print(f"移动时间: {move_time}秒")
                        self.publish_velocity(0, 0)
                
                return False  # 返回False表示尚未到达目标
        
            finally:
                # 只有在非连续移动模式时才设置is_executing_command为False
                if move_value < 100:
                    self.is_executing_command = False
                    self.in_continuous_mode = False
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
    
    def rotate_search(self):
        """执行原地旋转搜索"""
        # 使用较低的转向速度
        turn_speed = int(self.max_speed * 0.8)  # 使用15%的最大速度
        # 顺时针旋转（左轮正转，右轮反转）
        self.publish_velocity(-turn_speed, -turn_speed)
        time.sleep(0.2)  # 等待0.1秒
        self.publish_velocity(0, 0)  # 停止旋转

    def timer_callback(self):
        """定时器回调函数"""
        # 只在没有执行命令时发布0速度
        if not self.is_executing_command and not hasattr(self, 'in_continuous_mode'):
            self.publish_velocity(0, 0)

    def on_shutdown(self):
        """关闭时释放相机资源"""
        if hasattr(self, 'cap'):
            self.cap.release()

def get_key(timeout=0.1):
    """非阻塞方式获取键盘输入"""
    # 保存终端设置
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # 设置终端为raw模式
        tty.setraw(sys.stdin.fileno())
        # 使用select检查是否有输入，设置超时时间
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key
def main():
    rclpy.init()
    node = IntegratedVisionControl()
    
    def spin_thread():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    try:
        while not node.stop_tracking:
            if node.switch_target_flag or node.current_target is None:
                # 获取新的目标
                node.current_target = input("\n请输入要追踪的目标物体(输入'q'退出): ")
                if node.current_target.lower() == 'q':
                    break
                    
                print(f"\n开始追踪新目标: {node.current_target}")
                node.switch_target_flag = False
            
            if node.is_executing_command:
                time.sleep(0.01)
                continue
            
            # 检查键盘输入
            key = get_key()
            if key == 'n':
                print("\n检测到'n'键，准备切换目标...")
                node.switch_target_flag = True
                continue
            elif key == 'q':
                print("\n检测到'q'键，准备退出程序...")
                break
            
            # 等待获取有效的图像帧
            retry_count = 0
            max_retries = 10
            while node.latest_frame is None and retry_count < max_retries:
                print(f"等待相机图像... ({retry_count + 1}/{max_retries})")
                time.sleep(0.2)
                retry_count += 1
            
            if node.latest_frame is None:
                print("无法获取相机图像，请检查相机连接")
                continue
            
            # 获取视觉命令
            model_response = node.get_vision_command(node.current_target)
            if model_response:
                print(f"模型响应: {model_response}")
                move_match = re.search(r'move:\s*(-?\d+)', model_response)
                turn_match = re.search(r'turn:\s*(-?\d+)', model_response)
                
                move_value = int(move_match.group(1)) if move_match else 0
                turn_value = int(turn_match.group(1)) if turn_match else 0
                
                # 如果视觉模型检测不到目标（返回move: 0 turn: 0），继续尝试
                if move_value == 0 and turn_value == 0:
                    print("未检测到目标，继续搜索...")
                    node.rotate_search() 
                    
                    continue
                
                print(f"解析结果 - 移动: {move_value}cm, 转向: {turn_value}度")
                reached_target = node.execute_command(move_value, turn_value)
                
                if reached_target:
                    print(f"已到达目标: {node.current_target}")
                    print("按'n'键切换新目标，或按'q'键退出")
                
 # 在连续移动模式下使用更短的等待时间
                if hasattr(node, 'in_continuous_mode') and node.in_continuous_mode:
                    time.sleep(0.05)
                else:
                    time.sleep(0.1)
            else:
                print("未能获取有效的导航命令，重试中...")
                node.rotate_search()
                time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        node.publish_velocity(0, 0)
        time.sleep(0.1)
        
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()