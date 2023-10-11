FROM ros:humble

# Install curl, vim, tmux

RUN apt-get update
RUN apt-get install -y curl vim tmux ros-humble-rosbridge-server
RUN apt-get install -y python3-pip build-essential libcap-dev
RUN apt-get update && apt-get install -y ros-humble-cv-bridge
RUN apt-get install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera
RUN apt install ros-humble-image-transport-plugins
RUN pip3 install rpi.gpio
RUN pip3 install smbus
RUN pip3 install getch
RUN pip3 install matplotlib

# Setup camera user
RUN usermod -aG video root
RUN apt-get install raspi-config

# Setup .bashrc
RUN touch /root/.bashrc \
    && cat "source /root/ros2_ws/src/source.sh"
# Create workspace

RUN mkdir -p ~/ros2_ws/src

COPY ./Car.py /root/
COPY ./Ultrasonic.py /root/
