#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # 声明参数
        self.declare_parameter('device_id', 0)  # 默认使用设备ID 0
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        
        # 获取参数
        self.device_id = self.get_parameter('device_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        
        # 创建发布者
        self.publisher = self.create_publisher(
            Image,
            'camera/image_raw',
            10
        )
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 初始化相机
        self.init_camera()
        
        # 创建定时器
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        
        self.get_logger().info(
            f'Started camera node with device {self.device_id} at {self.frame_width}x{self.frame_height} @ {self.fps}fps'
        )

    def init_camera(self):
        """初始化相机设备"""
        self.cap = cv2.VideoCapture(self.device_id)
        
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

    def timer_callback(self):
        """定时器回调函数，读取和发布图像"""
        if not self.cap.isOpened():
            self.get_logger().error('Camera is not opened')
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
            
        try:
            # 转换图像并发布
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error converting/publishing image: {str(e)}')

    def on_shutdown(self):
        """关闭时释放相机资源"""
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    camera_node = USBCameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.on_shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
