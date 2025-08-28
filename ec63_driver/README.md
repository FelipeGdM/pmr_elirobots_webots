# EC63 robot ROS 2 driver

ROS 2 wrapper aorund elite robots python SDK

To initialize the driver, run the command:

```bash
ros2 run ec63_driver ec63_driver
```

## Parameters

- `robot_ip` - String with the robot IP address. Defaults to `192.168.1.200`
- `dry_run` - Performs all operations without communicating with the robot. Defaults to `False`

## Communication

The driver listens the topic `/ec63_driver` for `JointTrajectory` messages and relays them to the physical robot
