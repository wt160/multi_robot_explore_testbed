#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from threading import Thread
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist

class RobotControlInterface(Node):
    def __init__(self, robot_name):
        super().__init__(robot_name + '_control_node')
        
        self.cmd_vel_pub_ = self.create_publisher(Twist, robot_name + '/cmd_vel', 10)
        pass
    
    def debugPrint(self):
        print('RobotControlInterface: debug')

    def sendCmdVel(self, v_x, v_y, v_w):
        t = Twist()
        t.linear.x = v_x
        t.linear.y = v_y
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = v_w
        self.cmd_vel_pub_.publish(t)

    def rotateNCircles(self, n, v_w):
        t_0 = time.time()
        while time.time() - t_0 < n*2*3.14159/v_w:
            self.sendCmdVel(0.0, 0.0, v_w)
            
