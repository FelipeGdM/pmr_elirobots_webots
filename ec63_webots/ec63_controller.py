from typing import cast

import rclpy
from controller import Motor, Node, Robot
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Ec63Driver:
    def init(self, webots_node, properties):
        self.__robot = cast(Robot, webots_node.robot)

        self.__joints = [cast(Motor, self.__robot.getDevice(f'joint{i+1}')) for i in range(6)]

        rclpy.init(args=None)
        self.__node = rclpy.create_node('ec63_driver')
        self.__node.create_subscription(Float32, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__target = 0.0

    def __cmd_vel_callback(self, msg: Float32):
        self.__target = msg.data

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__joints[0].setPosition(self.__target)

