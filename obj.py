#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty
import time

class DecObj(Node):

    def __init__(self):
        super().__init__('dec_obj')
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/vision/set_trajectory_demo', 10)
        self.min_distance = float("inf")
        self.key_listener = KeyListener()
        self.got = 0
        self.lfup = [-1.0,-1.0,-1.0]
        self.jup = ['lift_joint','forkone_joint','forktwo_joint']
        self.switch_service_client = self.create_client(SetBool, '/demo/switch_demo')
        while not self.switch_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.switch_service_client1 = self.create_client(SetBool, '/demo/switch_demo')
        self.switch_service_client2 = self.create_client(SetBool, '/demo1/switch_demo')
        while not self.switch_service_client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.timer_ = self.create_timer(0.01, self.callback)
        
    def publish_joint_trajectory_command(self, position , joint):
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.frame_id = 'world'
        joint_trajectory_msg.joint_names = joint

        point = JointTrajectoryPoint()
        point.positions = position
        joint_trajectory_msg.points.append(point)

        self.joint_trajectory_pub.publish(joint_trajectory_msg)


    def laser_callback(self, msg):

        num_readings = len(msg.ranges)

         # Define the angle range to focus on the front of the robot

        front_ranges = msg.ranges

         # Filter out 'inf' values, which may occur if the sensor can't detect an obstacle
        front_ranges_filtered = [r for r in front_ranges if r != float('inf')]

        if front_ranges_filtered:
             self.min_distance = min(front_ranges_filtered)
        else:
             self.min_distance = float('inf')

        print("Minimum distance: ", self.min_distance)
        
    def callback(self):
        twist = Twist()

        if self.min_distance == float('inf') : 
            twist.linear.x=1.00
            twist.angular.z=0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('MOVING')
            print("MOVING")
            
        if  0.6 < self.min_distance <= 20 :
            if(self.min_distance<=5):
                twist.linear.x=0.0
                twist.angular.z=0.35
                self.cmd_vel_pub.publish(twist)
            else:
                twist.linear.x=1.0
                twist.angular.z=0.0
                self.cmd_vel_pub.publish(twist)

            
        # if self.min_distance <= 0.6: 
        #     twist.angular.z = 0.0
        #     twist.linear.x = 0.0
        #     self.get_logger().info('STOP')
        #     print("stop")
        #     self.call_switch_service(True)

        # Check if a key is pressed
        if self.key_listener.key_pressed():
            key = self.key_listener.get_key()
            if key == 'a':
                twist.angular.z = 1.0
                twist.linear.x = 0.0
                self.get_logger().info('KEY PRESSED a - TWIST ANGULAR z anti clock')
                print("Key pressed - a")
            elif key == 'w':
                twist.linear.x = 1.0
                twist.angular.z = 0.0
                self.get_logger().info('KEY PRESSED w - TWIST LINEAR  x front')
                print("Key pressed - w")
            elif key == 's':
                twist.linear.x = -1.0
                twist.angular.z = 0.0
                self.get_logger().info('KEY PRESSED s - TWIST LINEAR  x back')
                print("Key pressed - s")
            elif key == 'd':
                twist.angular.z = -11.0
                twist.linear.x = 0.0
                self.get_logger().info('KEY PRESSED d - TWIST ANGULAR  z clock')
                print("Key pressed - d")
            elif key == 'z':
                self.lfup = [1.0,0.10,0.10]
                self.get_logger().info('KEY PRESSED z - LIFT')
                print("Key pressed - z")
            elif key == 'x':
                self.lfup = [-1.0,-1.0,-1.0]
                self.get_logger().info('KEY PRESSED x - LEAVE')
                print("Key pressed - x")
                self.call_switch_service1(False)
            elif key == 'c':
                self.lfup = [-1.0,0.10,0.10]
                self.get_logger().info('KEY PRESSED c - COMPRESS')
                print("Key pressed - c")
                self.call_switch_service(False)
                self.call_switch_service1(True)

            self.cmd_vel_pub.publish(twist)
            time.sleep(2)
                

        self.publish_joint_trajectory_command(self.lfup,self.jup)
        self.cmd_vel_pub.publish(twist)

    def call_switch_service(self, data):
        request = SetBool.Request()
        request.data = data
        future = self.switch_service_client.call_async(request)

        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     self.get_logger().info('Switch service called successfully: {}'.format(future.result().success))
        # else:
        #     self.get_logger().error('Failed to call switch service')
    def call_switch_service1(self, data):
        request = SetBool.Request()
        request.data = data
        future = self.switch_service_client1.call_async(request)
        future = self.switch_service_client2.call_async(request)

class KeyListener:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def key_pressed(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def get_key(self):
        if self.key_pressed():
            return sys.stdin.read(1)
        else:
            return None     
#------------------------------------------------------------------#  
        
def main(args=None):
    rclpy.init(args=args)
    dec_obj = DecObj()
    try:
        rclpy.spin(dec_obj)
    except KeyboardInterrupt:
        dec_obj.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        dec_obj.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
  main()
