# Webots simulation for EC63

To run the simulation, execute the command

```bash
webots worlds/table.wbt
```

## Docker

To start the simulation inside a docker container, use the script `run_docker.sh`

A web interface to control the simulation will be available at `localhost:1234/index.html`. The `index.html` part is needed, as no automatic redirect is performed

## Communication

To communicate with the simulation, the `Client` class from `pmr_elirobots_driver` should be used

Usage example:

```python
from pmr_elirobots_driver.client import Client

robot = Client(ip="localhost", port=5555)

count = 0
while True:
    count += 1

    if count % 2:
        print("Send cmd1")
        robot.send_command(joint1=180, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0)
    else:
        print("Send cmd2")
        robot.send_command(joint2=-90) # ommited joints maintain their position

    time.sleep(10)
```

## Copyright notice

EC63 model converted from [Elite-Robots/ROS](https://github.com/Elite-Robots/ROS)

All rights reserved to Elite Robots
