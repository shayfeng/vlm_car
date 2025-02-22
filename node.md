# SCServo ROS2 驱动节点使用文档

## 简介
SCServoNode 是一个 ROS2 节点，用于控制飞特（FEETECH）系列串口舵机。该节点支持位置控制和速度控制，并提供实时的舵机状态反馈。

## 功能特点
- 支持多个舵机的同时控制
- 自动检测和管理舵机ID
- 支持位置和速度控制模式
- 实时状态反馈
- 自动错误恢复和重连机制
- 命令超时保护

## 参数配置

### 节点参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| device | string | /dev/ttyUSB0 | 串口设备路径 |
| baudrate | int | 1000000 | 串口波特率 |
| update_rate | float | 50.0 | 控制循环更新频率(Hz) |
| joint_prefix | string | servo_ | 关节名称前缀 |
| command_timeout | float | 1.0 | 命令超时时间(秒) |

### 修改参数示例
```bash
ros2 run scservo_driver scservo_node --ros-args -p device:=/dev/ttyUSB1
```

## 话题接口

### 订阅话题
- **joint_command** (sensor_msgs/JointState)
  - 用于发送舵机控制命令
  - 支持位置控制和速度控制
  - 关节名称格式：`{joint_prefix}{servo_id}`

### 发布话题
- **joint_states** (sensor_msgs/JointState)
  - 发布舵机的实时状态信息
  - 包含位置和速度数据
- **servo_status** (std_msgs/String)
  - 发布舵机的状态信息和错误消息

## 使用示例

### 1. 启动节点
```bash
ros2 run scservo_driver scservo_node
```

### 2. 发送位置控制命令
```bash
# 控制ID为1的舵机到1.57弧度（90度）位置
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['servo_1'],
  position: [1.57],
  velocity: [],
  effort: []
}"
```

### 3. 发送速度控制命令
```bash
# 控制ID为1的舵机以2.0弧度/秒的速度运动
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['servo_1'],
  position: [],
  velocity: [2.0],
  effort: []
}"
```

## 错误处理机制
1. **通信错误检测**：
   - 持续监控与舵机的通信状态
   - 发生错误时自动关闭力矩

2. **自动重连**：
   - 连续错误达到阈值时尝试重新连接
   - 只有所有舵机都恢复正常才会重新启用

3. **命令超时保护**：
   - 超过指定时间未收到命令自动关闭力矩
   - 收到新命令后自动重新启用

## 注意事项
1. 确保串口设备具有正确的读写权限
2. 使用前检查波特率设置是否与舵机当前配置匹配
3. 控制命令中的关节名称必须符合格式：`{joint_prefix}{servo_id}`
4. 位置控制范围：-π ~ π rad
5. 速度控制范围：-3.7π ~ 3.7π rad/s @6V

## 故障排除
1. **无法打开串口**
   - 检查设备路径是否正确
   - 检查用户权限：`sudo chmod 666 /dev/ttyUSB0`

2. **通信错误**
   - 检查波特率设置，应为1000000
   - 检查舵机供电
   - 检查串口连接是否可靠

3. **舵机不响应**
   - 检查舵机ID是否正确，有没有冲突
   - 查看 servo_status 话题的错误信息