# Robot Suite image for the Unitree Go1 (hand gesture control).
#
# Includes the Go1 driver (unitree_ros, cloned into drivers/ by bootstrap.sh go1),
# the RealSense driver and the robot suite.
#
# Build: docker compose build go1_suite
#   or   docker build -t robot_suite:go1 -f docker/go1/Dockerfile .
FROM ros:jazzy

# System + ROS packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-venv python3-vcstool wget git iputils-ping usbutils \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-realsense2-camera libboost-dev \
    portaudio19-dev alsa-base alsa-utils && \
    rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Python dependencies in their own layer, so code changes don't reinstall them
COPY requirements.txt /tmp/requirements.txt
RUN apt remove -y python3-typing-extensions && \
    pip install --extra-index-url https://download.pytorch.org/whl/cpu torch torchvision && \
    pip install -r /tmp/requirements.txt

# Copy Robot Suite and build it
COPY . /workspace/robot_suite
WORKDIR /workspace/robot_suite
RUN apt update && ./bootstrap.sh go1 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/robot_suite/install/setup.bash" >> ~/.bashrc

# Entrypoint
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/jazzy/setup.bash" >> /entrypoint.sh \
    && echo "source /workspace/robot_suite/install/setup.bash" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
