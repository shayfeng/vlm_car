#!/usr/bin/env python3
import os
from .scservo_sdk import *

class SCServoController:
    def __init__(self, device='/dev/ttyUSB0', baudrate=1000000, servo_ids=[]):
        self.device = device
        self.baudrate = baudrate
        self.servo_ids = servo_ids
        
        # Control table addresses
        self.ADDR_SCS_TORQUE_ENABLE = 40
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_SPEED = 46
        self.ADDR_STS_PRESENT_POSITION = 56
        self.ADDR_OPERATING_MODE = 0x21  # 运行模式地址
        
        # 添加模式跟踪字典
        self.servo_modes = {}  # 用于跟踪每个舵机的当前模式
        
        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(self.device)
        self.packetHandler = PacketHandler(0)  # STS/SMS=0
        
        # Initialize connection first
        if not self.portHandler.openPort():
            raise Exception("Failed to open port")
            
        if not self.portHandler.setBaudRate(self.baudrate):
            raise Exception("Failed to set baudrate")
        
        # Initialize GroupSyncWrite instances for both position and velocity
        self.position_sync_write = GroupSyncWrite(
            self.portHandler, 
            self.packetHandler, 
            self.ADDR_STS_GOAL_POSITION, 
            2
        )
        self.velocity_sync_write = GroupSyncWrite(
            self.portHandler, 
            self.packetHandler, 
            self.ADDR_STS_GOAL_SPEED, 
            2
        )
        
        # Initialize GroupSyncRead
        self.groupSyncRead = GroupSyncRead(
            self.portHandler, 
            self.packetHandler, 
            self.ADDR_STS_PRESENT_POSITION, 
            4
        )
        
        # Initialize sync read parameters for existing servo_ids
        if self.servo_ids:
            for servo_id in self.servo_ids:
                if not self.groupSyncRead.addParam(servo_id):
                    raise Exception(f"Failed to add parameter for Servo ID {servo_id}")

    def set_operating_mode(self, servo_id, mode):
        """
        设置舵机运行模式
        mode: 0 = 位置控制模式, 1 = 速度控制模式
        """
        # 先禁用扭矩
        self.packetHandler.write1ByteTxRx(
            self.portHandler,
            servo_id,
            self.ADDR_SCS_TORQUE_ENABLE,
            0
        )
        
        # 设置运行模式
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, 
            servo_id, 
            self.ADDR_OPERATING_MODE, 
            mode
        )
        
        # 重新启用扭矩
        self.packetHandler.write1ByteTxRx(
            self.portHandler,
            servo_id,
            self.ADDR_SCS_TORQUE_ENABLE,
            1
        )
        
        if dxl_comm_result != COMM_SUCCESS:
            return False
        elif dxl_error != 0:
            return False
            
        # 更新模式跟踪
        self.servo_modes[servo_id] = "position" if mode == 0 else "velocity"
        return True

    def ensure_mode(self, servo_id, required_mode):
        """
        确保舵机处于正确的运行模式
        """
        current_mode = self.servo_modes.get(servo_id)
        if current_mode != required_mode:
            mode_value = 0 if required_mode == "position" else 1
            return self.set_operating_mode(servo_id, mode_value)
        return True

    def add_servo(self, servo_id):
        """添加新的舵机ID到控制列表"""
        if servo_id not in self.servo_ids:
            self.servo_ids.append(servo_id)
            if not self.groupSyncRead.addParam(servo_id):
                raise Exception(f"Failed to add parameter for Servo ID {servo_id}")
            # 默认设置为位置模式
            self.set_operating_mode(servo_id, 0)

    def set_servo_positions(self, position_dict):
        """设置舵机位置"""
        if not position_dict:
            return {"success": [], "failed": []}
            
        failed_ids = []
        
        for servo_id, position in position_dict.items():
            if servo_id not in self.servo_ids:
                self.add_servo(servo_id)
            
            # 确保舵机处于位置模式
            if not self.ensure_mode(servo_id, "position"):
                failed_ids.append(servo_id)
                continue
                
            param = [SCS_LOBYTE(position), SCS_HIBYTE(position)]
            if not self.position_sync_write.addParam(servo_id, param):
                failed_ids.append(servo_id)
                continue
        
        if self.position_sync_write.data_dict:
            result = self.position_sync_write.txPacket()
            if result != COMM_SUCCESS:
                self.position_sync_write.clearParam()
                raise Exception(f"Failed to write positions: {self.packetHandler.getTxRxResult(result)}")
        
        self.position_sync_write.clearParam()
        return {
            "success": [id for id in position_dict.keys() if id not in failed_ids],
            "failed": failed_ids
        }

    def set_servo_velocities(self, velocity_dict):
        """设置舵机速度"""
        if not velocity_dict:
            return {"success": [], "failed": []}
            
        failed_ids = []
        
        for servo_id, velocity in velocity_dict.items():
            if servo_id not in self.servo_ids:
                self.add_servo(servo_id)
            
            # 确保舵机处于速度模式
            if not self.ensure_mode(servo_id, "velocity"):
                failed_ids.append(servo_id)
                continue
            
            # 速度方向由最高位决定
            direction_bit = 0x8000 if velocity < 0 else 0
            abs_velocity = abs(int(velocity))
            velocity_value = direction_bit | (abs_velocity & 0x7FFF)
            
            param = [SCS_LOBYTE(velocity_value), SCS_HIBYTE(velocity_value)]
            if not self.velocity_sync_write.addParam(servo_id, param):
                failed_ids.append(servo_id)
                continue
        
        if self.velocity_sync_write.data_dict:
            result = self.velocity_sync_write.txPacket()
            if result != COMM_SUCCESS:
                self.velocity_sync_write.clearParam()
                raise Exception(f"Failed to write velocities: {self.packetHandler.getTxRxResult(result)}")
        
        self.velocity_sync_write.clearParam()
        return {
            "success": [id for id in velocity_dict.keys() if id not in failed_ids],
            "failed": failed_ids
        }

    def test_communication(self, servo_id):
        """
        测试与指定舵机的通信
        返回: (bool, str) - (是否成功, 错误信息)
        """
        try:
            # 尝试读取位置寄存器
            _, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                self.portHandler,
                servo_id,
                self.ADDR_STS_PRESENT_POSITION
            )
            
            if dxl_comm_result != COMM_SUCCESS:
                return False, f"Communication error: {self.packetHandler.getTxRxResult(dxl_comm_result)}"
            elif dxl_error != 0:
                return False, f"Servo error: {self.packetHandler.getRxPacketError(dxl_error)}"
            
            return True, ""
            
        except Exception as e:
            return False, str(e)

    def test_all_servos(self, servo_ids):
        """
        测试与多个舵机的通信
        返回: dict - 包含成功和失败的舵机ID列表
        """
        successful_ids = []
        failed_ids = []
        errors = []

        for servo_id in servo_ids:
            success, error = self.test_communication(servo_id)
            if success:
                successful_ids.append(servo_id)
            else:
                failed_ids.append(servo_id)
                errors.append(f"ID {servo_id}: {error}")

        return {
            "success": successful_ids,
            "failed": failed_ids,
            "errors": errors
        }

    def read_servo_positions(self):
        """读取舵机位置"""
        if not self.servo_ids:
            return {"positions": {}, "errors": []}
            
        result = self.groupSyncRead.txRxPacket()
        positions = {}
        errors = []
        
        for servo_id in self.servo_ids:
            if self.groupSyncRead.isAvailable(servo_id, self.ADDR_STS_PRESENT_POSITION, 4):
                pos_speed = self.groupSyncRead.getData(servo_id, self.ADDR_STS_PRESENT_POSITION, 4)
                position = SCS_LOWORD(pos_speed)
                speed = SCS_TOHOST(SCS_HIWORD(pos_speed), 15)
                positions[servo_id] = (position, speed)
            else:
                errors.append(f"No response from ID:{servo_id}")
        
        if result != COMM_SUCCESS:
            errors.insert(0, f"Communication error: {self.packetHandler.getTxRxResult(result)}")
        
        return {
            "positions": positions,
            "errors": errors
        }
        
    def close(self):
        """关闭串口连接"""
        if self.portHandler.is_open:
            self.portHandler.closePort()
