import sys
import time
from dataclasses import dataclass
from math import pi
from typing import cast

import rclpy
from elite import EC
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


@dataclass
class Command:
    joints_position: dict
    time_interval: float


def build_list(joints_position: dict[str, Command]):
    retval = []
    for joint in JOINT_NAMES:
        if joint not in joints_position.keys():
            retval.append(0.0)
        else:
            retval.append(joints_position[joint])
    return retval

def convert_rad_to_deg(angle: float):
    return angle * 180 / pi


class Ec63Driver(Node):
    def __init__(self):
        super().__init__("ec63_driver")
        self.subscription = self.create_subscription(JointTrajectory, "ec63_driver", self.listener_callback, 1)

        self.declare_parameter("robot_ip", "192.168.1.200")
        self.declare_parameter("dry_run", False)
        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value

        if not self.dry_run:
            self.get_logger().info(f"Connect to robot IP: {robot_ip}")

            self.robot = EC(ip=robot_ip, auto_connect=True)

            if not self.robot.robot_servo_on():
                self.get_logger().error("Servo failed to initialize")
                sys.exit(-1)

    def listener_callback(self, msg: JointTrajectory):
        if type(msg.points) is not list:
            return

        commands = cast(list[Command], [])
        for i in range(len(msg.points)):
            joints_position = {}
            for joint in JOINT_NAMES:
                if joint not in msg.joint_names:
                    continue

                joint_index = cast(list, msg.joint_names).index(joint)

                joints_position[joint] = msg.points[i].positions[joint_index]

            time_interval = (
                (msg.points[i].time_from_start.sec + msg.points[i].time_from_start.nanosec * 1e-9)
                - (msg.points[i - 1].time_from_start.sec + msg.points[i - 1].time_from_start.nanosec * 1e-9)
                if i > 0
                else 0
            )

            commands.append(
                Command(
                    joints_position=joints_position,
                    time_interval=time_interval,
                )
            )

        for cmd in commands:
            joint_cmd_list = build_list(cmd.joints_position)
            speed = 30

            self.get_logger().info(f"Move to {joint_cmd_list} with speed {speed}")

            if not self.dry_run:
                self.robot.move_joint([convert_rad_to_deg(angle) for angle in joint_cmd_list], speed=speed)
                time.sleep(cmd.time_interval)


def main(args=None):
    rclpy.init(args=args)

    ec63_driver = Ec63Driver()

    try:
        rclpy.spin(ec63_driver)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # ec63_driver.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
