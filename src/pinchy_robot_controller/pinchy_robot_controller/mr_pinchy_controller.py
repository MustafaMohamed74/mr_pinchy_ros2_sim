#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import pygame

class MyNode(Node):

    def __init__(self):
        super().__init__("mr_pinchy_control")
        self.counter_ = 0 
        self.get_logger().info("Hello from mr pinchy controller node ....")
        self.twist_msg_pub = self.create_publisher(Twist , "/cmd_vel" , 10)
        self.joints_msg_pub = self.create_publisher(Float64MultiArray , "/forward_position_controller/commands",10 )
        self.pygame_config()
        self.create_timer(0.1 , self.msg_pub_timer_callback)
    
    def msg_pub_timer_callback(self):
        running = True
        twist_msg = Twist()
        joints_msg = Float64MultiArray()
        joints_msg.data = [-1.5,-1.5]
        

        while running:
            keys = pygame.key.get_pressed()
            if  keys[pygame.K_q]:
                if  joints_msg.data[0] < 0 :
                    joints_msg.data[0] +=0.000005
            if keys[pygame.K_z]:
                if joints_msg.data[0] > -1.58 :
                    joints_msg.data[0] -=0.000005
            if keys[pygame.K_e]:
                if joints_msg.data[1] < 1.58 :
                    joints_msg.data[1] +=0.000005
            if keys[pygame.K_c]:
                if joints_msg.data[1] > -1.58  :
                    joints_msg.data[1] -=0.000005

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    print(f"Key pressed: {pygame.key.name(event.key)}")
                    if event.key == pygame.K_ESCAPE:  
                        running = False
                    if event.key == pygame.K_w:
                        twist_msg.linear.x=1.0
                    if event.key == pygame.K_s:
                        twist_msg.linear.x=-1.0
                    if event.key == pygame.K_d:
                        twist_msg.angular.z=2.0
                    if event.key == pygame.K_a:
                        twist_msg.angular.z=-2.0
                    

                        
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_w or event.key == pygame.K_s: 
                        twist_msg.linear.x=0.0
                    if event.key == pygame.K_d or event.key == pygame.K_a: 
                         twist_msg.angular.z=0.0
                self.twist_msg_pub.publish(twist_msg)
                # print(joints_msg)
                self.joints_msg_pub.publish(joints_msg)
                        
        pygame.quit()

        # /\self.twist_msg_pub.publish(msg)
    def pygame_config(self):
        # Initialize Pygame
        pygame.init()
        # Create a small window (needed for event handling)
        self.screen = pygame.display.set_mode((200, 200))
        pygame.display.set_caption("Keyboard Input Example")



def main(args = None):
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

