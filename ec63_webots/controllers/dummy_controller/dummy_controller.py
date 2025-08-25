"""dummy_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
from typing import cast

from controller import Motor, Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

joint1 = cast(Motor, robot.getDevice('joint1'))
pos_sensor = joint1.getPositionSensor()

if pos_sensor is None:
    sys.exit(1)

pos_sensor.enable(timestep)
# Main loop:
# - perform simulation steps until Webots is stopping the controller

angle_cmd = 0
rising = True
while robot.step(timestep) != -1:

    joint1.setPosition(angle_cmd)
    print(f"Position: {pos_sensor.getValue()}")

    if angle_cmd >= 2:
        rising = False
    elif angle_cmd <= 1:
        rising = True

    if rising:
        angle_cmd += 0.01
    else:
        angle_cmd -= 0.01

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
