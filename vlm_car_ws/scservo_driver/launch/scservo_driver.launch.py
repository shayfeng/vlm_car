from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyUSB0',
            description='Serial device for SCServo'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Baudrate for serial communication'
        ),
        DeclareLaunchArgument(
            'joint_prefix',
            default_value='servo_',
            description='Prefix for joint names'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='50.0', # 默认控制频率
            description='Rate at which to publish joint states'
        ),
        DeclareLaunchArgument(
            'command_timeout',
            default_value='1.0',
            description='Timeout for command messages (seconds)'
        ),
        
        Node(
            package='scservo_driver',
            executable='scservo_node',
            name='scservo_driver',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'joint_prefix': LaunchConfiguration('joint_prefix'),
                'update_rate': LaunchConfiguration('update_rate'),
                'command_timeout': LaunchConfiguration('command_timeout'),
            }],
            output='screen',
        )
    ])
