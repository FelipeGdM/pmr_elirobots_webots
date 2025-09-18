FROM cyberbotics/webots:R2025a-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y xvfb screen python3-pip

RUN python3 -m pip install zmq pmr_elirobots_msgs

RUN mkdir /app

WORKDIR /app

ADD . /app

EXPOSE 1234

CMD [ "/bin/bash", "-c", "webots --stream --batch --stdout --stderr --minimize worlds/table.wbt"  ]
# CMD [ "/bin/bash", "-c", "screen -dmS Xsession Xvfb :99 -screen 0 1024x768x16 && webots --stream --batch --stdout --stderr --minimize worlds/table.wbt"  ]
# CMD [ "Xvfb :99 -screen 0 1024x768x16 &", "webots", "--stream", "--batch", "--stdout", "--stderr", "--minimize", "worlds/table.wbt" ]
