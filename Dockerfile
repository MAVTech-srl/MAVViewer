FROM arm64v8/ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y python3-pip python3-venv ros-humble-rclpy ros-dev-tools libgl1-mesa-glx

RUN mkdir -p /opt/mavviewer
WORKDIR /opt/mavviewer

# COPY requirements.txt . 
COPY ros_viser_pcd.py .
COPY entrypoint.sh .

# RUN pip install --upgrade pip setuptools
RUN python3 -m venv .venv --system-site-packages --symlinks \
    && . .venv/bin/activate \
    && pip install --upgrade pip setuptools \
    && pip install --verbose numpy open3d scikit-learn viser

ENV SHELL="/bin/bash"

CMD . .venv/bin/activate && exec python3 ros_viser_pcd.py