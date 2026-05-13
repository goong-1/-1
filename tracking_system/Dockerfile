FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    v4l-utils \
    libgl1 \
    libglib2.0-0 \
    libxext6 \
    libsm6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    flask \
    flask-socketio \
    eventlet \
    numpy

WORKDIR /ros_ws

COPY . ./src/tracking_system

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

EXPOSE 5001
EXPOSE 10000

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && ros2 launch tracking_system tracking_tf_launch.py"]
