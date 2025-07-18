import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import os, sys, time, threading, re, math, base64
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

class IntegratedVisionControl(Node):
    def __init__(self):
        super().__init__('integrated_vision_control')
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("机器人控制界面")
        
        # 设置窗口大小和位置
        self.root.geometry("1024x700")  # 增加宽度以适应状态面板
        
        # 配置列权重以使布局更灵活
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        
        # 创建主框架
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置主框架的列权重
        self.main_frame.grid_columnconfigure(0, weight=3)  # 相机区域占更多空间
        self.main_frame.grid_columnconfigure(1, weight=1)  # 状态面板占较少空间
        
        # 创建输入区域
        self.create_input_area()
        
        # 创建相机显示区域和状态面板
        self.create_camera_area()
        
        # 创建状态显示区域（日志区域）
        self.create_status_area()
        
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
        self.setup_parameters()
        
        # 创建定时器
        self.setup_timers()
        
        self.get_logger().info(
            f'Initialized with camera device {self.device_id} at {self.frame_width}x{self.frame_height} @ {self.fps}fps'
        )

    def create_input_area(self):
        """创建输入区域"""
        input_frame = ttk.LabelFrame(self.main_frame, text="控制面板", padding="5")
        input_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # 目标输入框
        ttk.Label(input_frame, text="目标物体:").grid(row=0, column=0, padx=5)
        self.target_var = tk.StringVar()
        self.target_entry = ttk.Entry(input_frame, textvariable=self.target_var, width=30)
        self.target_entry.grid(row=0, column=1, padx=5)
        
        # 控制按钮
        self.start_button = ttk.Button(input_frame, text="开始追踪", command=self.start_tracking)
        self.start_button.grid(row=0, column=2, padx=5)
        
        self.stop_button = ttk.Button(input_frame, text="停止追踪", command=self.stop_tracking)
        self.stop_button.grid(row=0, column=3, padx=5)
        self.stop_button.state(['disabled'])
        
        # 绑定回车键到开始追踪
        self.target_entry.bind('<Return>', lambda e: self.start_tracking())

    def create_camera_area(self):
        """创建相机显示区域"""
        # 创建左侧主框架（包含相机）
        left_frame = ttk.Frame(self.main_frame)
        left_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        camera_frame = ttk.LabelFrame(left_frame, text="相机画面", padding="5")
        camera_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.camera_label = ttk.Label(camera_frame)
        self.camera_label.grid(row=0, column=0)
        
        # 创建右侧状态面板
        right_frame = ttk.Frame(self.main_frame)
        right_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=5)
        
        status_panel = ttk.LabelFrame(right_frame, text="实时状态", padding="10")
        status_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 状态指示器
        self.tracking_status = tk.StringVar(value="等待开始")
        self.detection_status = tk.StringVar(value="未开始检测")
        self.distance_status = tk.StringVar(value="小车状态: --")
        self.angle_status = tk.StringVar(value="角度: --")
        
        # 设置字体样式
        label_font = ('Arial', 16, 'bold')  # 使用更大的字体
        value_font = ('Arial', 16)          # 数值的字体
        
        # 使用ttk.Label创建状态显示，应用新的字体样式
        ttk.Label(status_panel, text="追踪状态:", font=label_font).grid(row=0, column=0, sticky=tk.W, pady=10)
        self.tracking_label = ttk.Label(status_panel, textvariable=self.tracking_status, font=value_font)
        self.tracking_label.grid(row=0, column=1, sticky=tk.W, pady=10)
        
        ttk.Label(status_panel, text="检测状态:", font=label_font).grid(row=1, column=0, sticky=tk.W, pady=10)
        self.detection_label = ttk.Label(status_panel, textvariable=self.detection_status, font=value_font)
        self.detection_label.grid(row=1, column=1, sticky=tk.W, pady=10)
        
        ttk.Label(status_panel, text="小车状态：", font=label_font).grid(row=2, column=0, sticky=tk.W, pady=10)
        self.distance_label = ttk.Label(status_panel, textvariable=self.distance_status, font=value_font)
        self.distance_label.grid(row=2, column=1, sticky=tk.W, pady=10)
        
        ttk.Label(status_panel, text="转向角度:", font=label_font).grid(row=3, column=0, sticky=tk.W, pady=10)
        self.angle_label = ttk.Label(status_panel, textvariable=self.angle_status, font=value_font)
        self.angle_label.grid(row=3, column=1, sticky=tk.W, pady=10)

        # 增加列间距
        status_panel.grid_columnconfigure(1, minsize=150)  # 给值的显示留出更多空间

    def create_status_area(self):
        """创建状态显示区域"""
        status_frame = ttk.LabelFrame(self.main_frame, text="运行日志", padding="5")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.status_text = tk.Text(status_frame, height=8, width=60)
        self.status_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        self.status_text.configure(yscrollcommand=scrollbar.set)
        self.status_text.config(state=tk.DISABLED)

    def update_status_display(self, tracking_status=None, detection_status=None, distance=None, angle=None):
        """更新状态显示"""
        if tracking_status is not None:
            self.tracking_status.set(tracking_status)
        if detection_status is not None:
            self.detection_status.set(detection_status)
        if distance is not None:
            self.distance_status.set(f"前进")
        if angle is not None:
            self.angle_status.set(f"角度: {angle}°")

    def setup_parameters(self):
        """设置参数"""
        self.max_speed = 10
        self.is_running = True
        self.latest_frame = None
        self.current_target = None
        self.switch_target_flag = False
        self.stop_tracking = False
        self.tracking_active = False
        
        self.servo_ids = [1, 2, 3, 4]
        self.joint_names = [f'servo_{id}' for id in self.servo_ids]
        
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        self.move_speed_ratio = 0.5
        self.turn_speed_ratio = 0.25
        
        self.is_executing_command = False

    def setup_timers(self):
        """设置定时器"""
        self.timer = self.create_timer(1/50.0, self.timer_callback)
        self.camera_timer = self.create_timer(1.0/self.fps, self.camera_callback)

    def log_status(self, message):
        """向状态区域添加日志"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)

    def start_tracking(self):
        """开始追踪目标"""
        target = self.target_var.get().strip()
        if target:
            self.current_target = target
            self.tracking_active = True
            self.stop_tracking = False
            self.start_button.state(['disabled'])
            self.stop_button.state(['!disabled'])
            
            self.update_status_display(
                tracking_status="正在追踪",
                detection_status="开始检测"
            )
            self.log_status(f"开始追踪目标: {target}")
            
            # 启动追踪线程
            threading.Thread(target=self.tracking_loop, daemon=True).start()
        else:
            self.log_status("请输入目标物体名称")

    def stop_tracking(self):
        """停止追踪"""
        self.tracking_active = False
        self.stop_tracking = True
        self.publish_velocity(0, 0)
        self.start_button.state(['!disabled'])
        self.stop_button.state(['disabled'])
        
        self.update_status_display(
            tracking_status="已停止",
            detection_status="未开始检测",
            distance="--",
            angle="--"
        )
        self.log_status("停止追踪")

    def init_camera(self):
        """初始化相机设备"""
        self.get_logger().info(f'Attempting to open camera device {self.device_id}')
        self.cap = cv2.VideoCapture(self.device_id)
        
        retry_count = 0
        max_retries = 3
        while not self.cap.isOpened() and retry_count < max_retries:
            self.get_logger().warn(f'Failed to open camera, retrying... ({retry_count + 1}/{max_retries})')
            time.sleep(1)
            self.cap = cv2.VideoCapture(self.device_id)
            retry_count += 1
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_id}')
            self.update_status_display(detection_status="相机初始化失败")
            raise RuntimeError(f'Failed to open camera device {self.device_id}')
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def camera_callback(self):
        """相机定时器回调函数，读取和显示图像"""
        if not self.cap.isOpened():
            self.update_status_display(detection_status="相机未连接")
            return
            
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.update_status_display(detection_status="相机读取失败")
            return
            
        self.latest_frame = frame
        
        # 调整图像大小以适应显示区域
        display_width = 640
        display_height = 480
        frame = cv2.resize(frame, (display_width, display_height))
        
        # 转换为PIL图像
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(image=pil_image)
        
        # 更新显示
        self.camera_label.configure(image=photo)
        self.camera_label.image = photo
    def tracking_loop(self):
        """追踪循环"""
        find_next_flag = 0
        while self.tracking_active and not self.stop_tracking:
            if self.is_executing_command:
                time.sleep(0.005)
                continue
                
            if self.latest_frame is None:
                continue
                
            # 获取视觉命令
            model_response = self.get_vision_command(self.current_target)
            if model_response:
                self.log_status(f"模型响应: {model_response}")
                move_match = re.search(r'move:\s*(-?\d+)', model_response)
                turn_match = re.search(r'turn:\s*(-?\d+)', model_response)
                
                move_value = int(move_match.group(1)) if move_match else 0
                turn_value = int(turn_match.group(1)) if turn_match else 0
                
                if move_value != 0 or turn_value != 0:
                    find_next_flag = 0
                    self.update_status_display(
                        detection_status="已检测到目标",
                        distance=str(move_value),
                        angle=str(turn_value)
                    )
                else:
                    self.log_status("未检测到目标，继续搜索...")
                    self.update_status_display(
                        detection_status="未检测到目标",
                        distance="--",
                        angle="--"
                    )
                    self.distance_status.set(f"--")
                    find_next_flag += 1
                    if find_next_flag >= 5:
                        self.log_status("连续5次未检测到目标，执行原地搜索")
                        self.rotate_search()
                    continue
                
                self.log_status(f"解析结果 - 移动: {move_value}cm, 转向: {turn_value}度")
                reached_target = self.execute_command(move_value, turn_value)
                
                if reached_target:
                    self.log_status(f"已到达目标: {self.current_target}")
                    self.update_status_display(detection_status="已到达目标")
                
                if hasattr(self, 'in_continuous_mode') and self.in_continuous_mode:
                    time.sleep(0.002)
                else:
                    time.sleep(0.005)
            else:
                self.log_status("未能获取有效的导航命令，重试中...")
                self.update_status_display(detection_status="获取导航命令失败")
                self.rotate_search()
                time.sleep(0.1)

    def execute_command(self, move_value, turn_value):
        """执行移动和转向命令"""
        self.in_continuous_mode = move_value >= 100
        
        if move_value < 100:
            self.is_executing_command = True
        
        try:
            if turn_value != 0:
                turn_speed = int(self.max_speed * self.turn_speed_ratio)
                if turn_value < 0:  # 左转
                    self.publish_velocity(turn_speed, turn_speed)
                else:  # 右转
                    self.publish_velocity(-turn_speed, -turn_speed)
                
                turn_time = abs(turn_value) / 100 
                time.sleep(turn_time)
                self.log_status(f"转向时间: {turn_time}秒")
                self.publish_velocity(0, 0)

            if move_value > 0:
                if move_value <= 20:
                    self.log_status("已到达目标位置!")
                    self.update_status_display(detection_status="已到达目标")
                    self.in_continuous_mode = False
                    return True
                
                move_speed = int(self.max_speed * self.move_speed_ratio)
                
                if move_value >= 100:
                    self.log_status("目标距离大于等于100cm，进入连续移动模式")
                    self.publish_velocity(-move_speed, move_speed)
                    return False
                elif move_value < 50:
                    self.log_status("目标距离小于50cm，使用精确移动模式")
                    self.publish_velocity(-move_speed, move_speed)
                    move_time = move_value / 40
                    time.sleep(move_time)
                    self.log_status(f"移动时间: {move_time}秒")
                    self.publish_velocity(0, 0)
                else:
                    self.log_status("目标距离在过渡区域，使用中等移动距离")
                    self.publish_velocity(-move_speed, move_speed)
                    move_time = move_value / 60
                    time.sleep(move_time)
                    self.log_status(f"移动时间: {move_time}秒")
                    self.publish_velocity(0, 0)
            
            return False

        finally:
            if move_value < 100:
                self.is_executing_command = False
                self.in_continuous_mode = False

    def get_vision_command(self, target_object):
        """调用视觉模型API获取导航命令"""
        if self.latest_frame is None:
            self.update_status_display(detection_status="无相机图像")
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
机器人正在一张(宽1m2,长3m)木质桌面上导航，桌面为深棕色纹理表面，类似木纹。\
\
提示：\
\
    如果目标位于图像中心，则转向角度为 0 度。\
    如果目标位于图像最左侧，则转向角度为 -x 度（最大为 -90 度）。\
    如果目标位于图像最右侧，则转向角度为 x 度（最大为 90 度）。\
      角度一般与目标物和图像中心像素点相对位置有关。\
    距离一般与目标物在图像中的像素点分布有关。\
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
            return completion.choices[0].message.content
            
        except Exception as e:
            self.log_status(f"API调用错误: {str(e)}")
            self.update_status_display(detection_status="API调用失败")
            return None

    def encode_image(self, image):
        """将OpenCV图像编码为base64"""
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode('utf-8')

    def publish_velocity(self, left_speed, right_speed):
        """发布速度命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []
        msg.velocity = [float(left_speed), float(right_speed), float(left_speed), float(right_speed)]
        msg.effort = []
        self.joint_publisher.publish(msg)
    
    def rotate_search(self):
        """执行原地旋转搜索"""
        turn_speed = int(self.max_speed * 0.8)
        self.publish_velocity(-turn_speed, -turn_speed)
        time.sleep(0.25)
        self.publish_velocity(0, 0)

    def timer_callback(self):
        """定时器回调函数"""
        if not self.is_executing_command and not hasattr(self, 'in_continuous_mode'):
            self.publish_velocity(0, 0)

    def on_shutdown(self):
        """关闭时释放资源"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        self.root.quit()

def main():
    rclpy.init()
    node = IntegratedVisionControl()
    
    # 创建ROS自旋线程
    def spin_thread():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    try:
        # 运行tkinter主循环
        node.root.mainloop()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 清理资源
        node.publish_velocity(0, 0)
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()