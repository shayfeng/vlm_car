import pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from time import time
import sys

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher = self.create_publisher(JointState, 'joint_command', 10)
        
    def publish_command(self, left_speed, right_speed):
        msg = JointState()
        msg.header.stamp.sec = int(time())
        msg.header.stamp.nanosec = int((time() % 1) * 1e9)
        msg.name = ['servo_1', 'servo_2', 'servo_3', 'servo_4']
        msg.velocity = [float(-left_speed), float(right_speed), float(-left_speed), float(right_speed)]
        msg.position = []
        msg.effort = []
        self.publisher.publish(msg)
        print(f"Publishing velocities: {msg.velocity}")

def main():
    rclpy.init()
    controller = CarController()
    
    pygame.init()
    
    WINDOW_SIZE = (400, 400)
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("四轮差速小车控制")
    
    pygame.font.init()
    font = pygame.font.SysFont(None, 36)
    
    joystick_pos = [WINDOW_SIZE[0]//2, WINDOW_SIZE[1]//2]
    joystick_radius = 100
    stick_radius = 20
    stick_pos = joystick_pos.copy()
    
    clock = pygame.time.Clock()
    dragging = False
    running = True
    left_speed = 0.0
    right_speed = 0.0
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_pos = pygame.mouse.get_pos()
                    if ((mouse_pos[0] - stick_pos[0])**2 + 
                        (mouse_pos[1] - stick_pos[1])**2 <= stick_radius**2):
                        dragging = True
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
                    stick_pos = joystick_pos.copy()
                    controller.publish_command(0.0, 0.0)
                    
        if dragging:
            mouse_pos = pygame.mouse.get_pos()
            dx = mouse_pos[0] - joystick_pos[0]
            dy = mouse_pos[1] - joystick_pos[1]
            dist = (dx**2 + dy**2)**0.5
            if dist > joystick_radius:
                dx = dx * joystick_radius / dist
                dy = dy * joystick_radius / dist
            stick_pos = [joystick_pos[0] + dx, joystick_pos[1] + dy]
            
            max_speed = 11.6
            x_axis = -dx / joystick_radius  # 转向控制
            y_axis = -dy / joystick_radius  # 前进后退控制
            
            # 修改转向逻辑，使其能够实现大半径转弯
            # 当有前进/后退速度时，转向效果会更柔和
            forward_speed = y_axis * max_speed
            turn_speed = x_axis * max_speed * 0.7  # 降低转向灵敏度
            
            # 根据前进速度调整转向效果
            turn_factor = abs(forward_speed / max_speed)  # 速度越快转向越温和
            turn_speed = turn_speed * (1 - turn_factor * 0.5)  # 高速时转向更平缓
            
            # 差速驱动计算
            left_speed = float(forward_speed - turn_speed)
            right_speed = float(forward_speed + turn_speed)
            
            # 限制最大速度
            left_speed = float(max(min(left_speed, max_speed), -max_speed))
            right_speed = float(max(min(right_speed, max_speed), -max_speed))
            
            controller.publish_command(left_speed, right_speed)
            rclpy.spin_once(controller, timeout_sec=0)
        
        # 绘制界面
        screen.fill((255, 255, 255))
        
        # 绘制十字准星来指示前后左右方向
        pygame.draw.line(screen, (200, 200, 200), 
                        (joystick_pos[0], joystick_pos[1] - joystick_radius),
                        (joystick_pos[0], joystick_pos[1] + joystick_radius), 2)
        pygame.draw.line(screen, (200, 200, 200),
                        (joystick_pos[0] - joystick_radius, joystick_pos[1]),
                        (joystick_pos[0] + joystick_radius, joystick_pos[1]), 2)
        
        pygame.draw.circle(screen, (200, 200, 200), joystick_pos, joystick_radius, 2)
        pygame.draw.circle(screen, (100, 100, 255), [int(stick_pos[0]), int(stick_pos[1])], stick_radius)
        
        # 显示速度信息
        speed_text = font.render(f"L: {left_speed:.2f} R: {right_speed:.2f}", True, (0, 0, 0))
        screen.blit(speed_text, (10, 10))
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    controller.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()
