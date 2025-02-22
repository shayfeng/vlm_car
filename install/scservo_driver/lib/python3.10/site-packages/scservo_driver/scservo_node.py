#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .scservo_controller import SCServoController
import math
from rclpy.time import Time

class SCServoNode(Node):
    def __init__(self):
        super().__init__('scservo_driver')
        
        # 添加位置和速度限制常量
        self.MIN_POSITION = 0
        self.MAX_POSITION = 4095
        self.MAX_VELOCITY = 32767
        
        # 声明参数
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('joint_prefix', 'servo_')
        self.declare_parameter('command_timeout', 1.0)  # 命令超时参数(秒)
        
        # 获取参数
        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        update_rate = self.get_parameter('update_rate').value
        self.joint_prefix = self.get_parameter('joint_prefix').value
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # 命令超时相关变量
        self.last_command_time = self.get_clock().now()
        self.torque_enabled = True
        
        # 动态维护的舵机ID集合
        self.active_servo_ids = set()
        self.id_to_joint = {}
        self.joint_to_id = {}
        
        # 状态变量
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5
        
        # 创建发布者和订阅者
        self.joint_state_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        self.status_pub = self.create_publisher(
            String,
            'servo_status',
            10)
        self.command_sub = self.create_subscription(
            JointState,
            'joint_command',
            self.command_callback,
            10)
        
        # 初始化控制器
        try:
            self.controller = SCServoController(
                device=device,
                baudrate=baudrate
            )
            self.get_logger().info('Successfully initialized SCServo controller')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize SCServo controller: {str(e)}')
            raise
            
        # 创建定时器
        self.timer = self.create_timer(1.0/update_rate, self.control_loop)
        self.current_position_command = None
        self.current_velocity_command = None
        
        # 添加关闭回调
        try:
            self.add_on_shutdown_callback(self.on_shutdown)
        except AttributeError:
            # ROS2 Humble 之前的版本使用
            import atexit
            atexit.register(self.on_shutdown)

        # 添加单位转换常量
        self.TICKS_PER_REVOLUTION = 4096  # 每圈的刻度数
        self.RADIANS_PER_TICK = 2 * math.pi / self.TICKS_PER_REVOLUTION
        self.TICKS_PER_RADIAN = self.TICKS_PER_REVOLUTION / (2 * math.pi)

        # 添加通信错误标志
        self.comm_error = False

    def radians_to_ticks(self, radians):
        """将弧度转换为舵机刻度值，以中值2048为0点"""
        # 2048为中值，对应0弧度
        ticks = int(2048 + radians * self.TICKS_PER_RADIAN)
        return min(max(ticks, self.MIN_POSITION), self.MAX_POSITION)

    def ticks_to_radians(self, ticks):
        """将舵机刻度值转换为弧度，以中值2048为0点"""
        # 将刻度值转换为相对于中值的偏移
        return float(ticks - 2048) * self.RADIANS_PER_TICK


    def velocity_to_ticks(self, velocity_rad_per_sec):
        """将弧度/秒转换为舵机速度值"""
        # 使用与位置相同的转换系数
        velocity_ticks = int(velocity_rad_per_sec * self.TICKS_PER_RADIAN)
        # 确保速度在有效范围内
        return min(max(velocity_ticks, -self.MAX_VELOCITY), self.MAX_VELOCITY)

    def ticks_to_velocity(self, velocity_ticks):
        """将舵机速度值转换为弧度/秒"""
        # 使用与位置相同的转换系数
        return float(velocity_ticks) * self.RADIANS_PER_TICK

    def on_shutdown(self):
        """节点关闭时的清理工作"""
        self.get_logger().info("Shutting down, disabling torque...")
        try:
            # 关闭所有舵机的力矩
            self.enable_torque(False)
            # 关闭控制器连接
            self.controller.close()
            self.get_logger().info("Successfully disabled torque and closed connection")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def enable_torque(self, enable):
        """启用或禁用所有舵机的扭矩"""
        for servo_id in self.active_servo_ids:
            try:
                self.controller.packetHandler.write1ByteTxRx(
                    self.controller.portHandler,
                    servo_id,
                    self.controller.ADDR_SCS_TORQUE_ENABLE,
                    1 if enable else 0
                )
            except Exception as e:
                self.get_logger().error(
                    f'Failed to {"enable" if enable else "disable"} torque for servo {servo_id}: {str(e)}'
                )

    def check_command_timeout(self):
        """检查命令是否超时"""
        if not self.torque_enabled:
            return
            
        current_time = self.get_clock().now()
        time_since_last_command = (current_time - self.last_command_time).nanoseconds / 1e9
        
        if time_since_last_command > self.command_timeout:
            self.get_logger().warning(
                f'Command timeout after {time_since_last_command:.2f}s. Disabling torque.'
            )
            self.enable_torque(False)
            self.torque_enabled = False
            self.current_position_command = None
            self.current_velocity_command = None

    def update_servo_mappings(self, servo_id):
        """更新舵机ID和关节名称的映射"""
        if servo_id not in self.active_servo_ids:
            self.active_servo_ids.add(servo_id)
            joint_name = f'{self.joint_prefix}{servo_id}'
            self.id_to_joint[servo_id] = joint_name
            self.joint_to_id[joint_name] = servo_id
            self.get_logger().info(f'Added new servo ID: {servo_id}')

    def command_callback(self, msg: JointState):
        """处理关节命令"""
        try:
            # 更新命令时间戳
            self.last_command_time = self.get_clock().now()
            
            # 只在非通信错误的情况下重新启用力矩
            if not self.torque_enabled and not self.comm_error:
                self.enable_torque(True)
                self.torque_enabled = True
                self.get_logger().info('Re-enabling torque after timeout')
            
            # 检查是位置控制还是速度控制
            has_position = any(not math.isnan(p) for p in msg.position) if msg.position else False
            has_velocity = any(not math.isnan(v) for v in msg.velocity) if msg.velocity else False
            
            if has_position:
                position_dict = {}
                for name, position in zip(msg.name, msg.position):
                    if name.startswith(self.joint_prefix) and not math.isnan(position):
                        try:
                            servo_id = int(name[len(self.joint_prefix):])
                            self.update_servo_mappings(servo_id)
                            # 将弧度转换为刻度值
                            position_ticks = self.radians_to_ticks(position)
                            position_dict[servo_id] = position_ticks
                        except ValueError:
                            continue
                
                if position_dict:
                    self.current_position_command = position_dict
                    self.current_velocity_command = None
                    self.get_logger().debug(f'Received position command: {position_dict}')
            
            elif has_velocity:
                velocity_dict = {}
                for name, velocity in zip(msg.name, msg.velocity):
                    if name.startswith(self.joint_prefix) and not math.isnan(velocity):
                        try:
                            servo_id = int(name[len(self.joint_prefix):])
                            self.update_servo_mappings(servo_id)
                            # 使用相同的转换系数，但直接处理正负值
                            velocity_ticks = int(velocity * self.TICKS_PER_RADIAN)
                            # 限制在速度范围内
                            velocity_ticks = min(max(velocity_ticks, -self.MAX_VELOCITY), self.MAX_VELOCITY)
                            velocity_dict[servo_id] = velocity_ticks
                        except ValueError:
                            continue
                
                if velocity_dict:
                    self.current_velocity_command = velocity_dict
                    self.current_position_command = None
                    self.get_logger().debug(f'Received velocity command: {velocity_dict}')

            
        except Exception as e:
            self.get_logger().error(f'Failed to process command: {str(e)}')

    def reconnect(self):
        """尝试重新连接舵机"""
        try:
            # 1. 关闭现有连接
            self.controller.close()
            
            # 2. 创建新的控制器实例
            self.controller = SCServoController(
                self.get_parameter('device').value,
                self.get_parameter('baudrate').value
            )
            
            # 3. 测试所有舵机的通信状态
            all_servos_ok = True
            failed_servos = []
            
            for servo_id in self.active_servo_ids:
                try:
                    # 尝试读取舵机位置来测试通信
                    result = self.controller.read_servo_position(servo_id)
                    if not result["success"]:
                        all_servos_ok = False
                        failed_servos.append(servo_id)
                except Exception as e:
                    all_servos_ok = False
                    failed_servos.append(servo_id)
                    self.get_logger().error(f"Failed to communicate with servo {servo_id}: {str(e)}")
            
            if not all_servos_ok:
                error_msg = f"Reconnection incomplete - failed servos: {failed_servos}"
                self.get_logger().error(error_msg)
                self.publish_status(error_msg)
                return False
                
            # 4. 所有舵机都正常才算重连成功
            return True
                
        except Exception as e:
            self.get_logger().error(f"Reconnection failed: {str(e)}")
            return False

    def control_loop(self):
        """主控制循环"""
        if not self.active_servo_ids:
            return
            
        try:
            # 检查命令超时
            self.check_command_timeout()
            
            # 只在扭矩启用时发送命令
            if self.torque_enabled:
                if self.current_position_command:
                    result = self.controller.set_servo_positions(self.current_position_command)
                    if result["failed"]:
                        error_msg = f"Communication failed with servos: {result['failed']}, disabling all torque"
                        self.get_logger().warning(error_msg)
                        self.publish_status(error_msg)
                        self.enable_torque(False)
                        self.torque_enabled = False
                        self.comm_error = True
                        return
                    self.current_position_command = None
                
                elif self.current_velocity_command:
                    result = self.controller.set_servo_velocities(self.current_velocity_command)
                    if result["failed"]:
                        error_msg = f"Communication failed with servos: {result['failed']}, disabling all torque"
                        self.get_logger().warning(error_msg)
                        self.publish_status(error_msg)
                        self.enable_torque(False)
                        self.torque_enabled = False
                        self.comm_error = True
                        return
                    self.current_velocity_command = None
            
            # 读取舵机状态
            read_result = self.controller.read_servo_positions()
            positions = read_result["positions"]

            if read_result["errors"]:
                error_msg = f"Communication errors: {read_result['errors']}, disabling all torque"
                self.get_logger().warning(error_msg)
                self.publish_status(error_msg)
                self.enable_torque(False)
                self.torque_enabled = False
                self.comm_error = True
                self.consecutive_errors += 1

                # 检查连续错误是否需要重连
                if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                    self.get_logger().error("Too many consecutive errors, attempting to reconnect...")
                    reconnect_success = self.reconnect()
                    
                    if reconnect_success:
                        # 确认所有舵机都能正常通信
                        test_result = self.controller.read_servo_positions()
                        if not test_result["errors"]:
                            success_msg = "[Reconnection Success] Communication restored with all servos"
                            self.get_logger().info(success_msg)
                            self.publish_status(success_msg)
                            self.comm_error = False
                            self.consecutive_errors = 0
                        else:
                            error_msg = f"[Reconnection Incomplete] Some servos still not responding: {test_result['errors']}"
                            self.get_logger().error(error_msg)
                            self.publish_status(error_msg)
                    else:
                        error_msg = "[Reconnection Failed] Unable to restore communication"
                        self.get_logger().error(error_msg)
                        self.publish_status(error_msg)
                    return
            else:
                # 只有在所有舵机都正常时才重置错误状态
                if self.consecutive_errors > 0:
                    success_msg = "[Recovery Success] Communication restored with all servos"
                    self.get_logger().info(success_msg)
                    self.publish_status(success_msg)
                    self.comm_error = False
                    self.consecutive_errors = 0

            # 发布关节状态
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            for servo_id in sorted(self.active_servo_ids):
                if servo_id in positions:
                    position_ticks, velocity_ticks = positions[servo_id]
                    # 转换单位：刻度值转弧度，速度值转弧度/秒
                    position_rad = self.ticks_to_radians(position_ticks)
                    velocity_rad = self.ticks_to_velocity(velocity_ticks)
                    
                    msg.name.append(self.id_to_joint[servo_id])
                    msg.position.append(position_rad)
                    msg.velocity.append(velocity_rad)
                    msg.effort.append(0.0)

            if msg.name:
                self.joint_state_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')
            # 任何异常都关闭力矩
            error_msg = f"Control loop error: {str(e)}, disabling all torque"
            self.publish_status(error_msg)
            self.enable_torque(False)
            self.torque_enabled = False
            self.comm_error = True  # 设置通信错误标志
            self.consecutive_errors += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = SCServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
