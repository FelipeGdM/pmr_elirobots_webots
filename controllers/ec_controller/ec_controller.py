"""dummy_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import sys
from typing import cast

import zmq
from controller import Motor, Robot
from pmr_elirobots_msgs.cmd import JointCommandMsg

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

joint_list = [cast(Motor, robot.getDevice(f"joint{i}")) for i in range(1, 7)]


def receive_callback(raw_msg: list[bytes]):
    payload = raw_msg[1].decode("utf-8")

    msg = JointCommandMsg.from_json(payload)

    for n, angle in enumerate(msg.cmd.as_list):
        if angle is not None:
            joint_list[n].setPosition(angle / 180 * math.pi)


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("tcp://*:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "cmd")  # Subscribe to a topic

poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)  # Monitor for incoming messages

print("Controller ready!")

while robot.step(timestep) != -1:
    events = poller.poll(timeout=10)
    if events:
        recv_msg = socket.recv_multipart()
        receive_callback(recv_msg)
