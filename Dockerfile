# БАЗА по-прежнему: ROS2 Humble + VNC для ARM64
ARG BASE_IMAGE=tiryoh/ros2-desktop-vnc:humble-arm64-20230129T1546

FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,video


ENV ROS_DISTRO=humble

# Починка ROS2 ключей (если ещё не добавлял)
RUN set -e; \
    rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-latest.list; \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg /etc/apt/trusted.gpg.d/ros* || true; \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && update-ca-certificates; \
    mkdir -p /usr/share/keyrings; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'; \
    apt-get update

# Базовые пакеты и зависимости
RUN apt-get install -y --no-install-recommends \
      wget tar xz-utils build-essential cmake pkg-config git nano \
      libusb-1.0-0 libusb-1.0-0-dev udev \
      libgl1 libx11-6 libxext6 libxrender1 libglib2.0-0 \
      python3 python3-pip \
  && rm -rf /var/lib/apt/lists/*

# Python-пакеты (по желанию)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install pyzmq transforms3d

WORKDIR /tmp/ximea
COPY ximea_linux_arm_sp_beta.tgz /tmp/ximea/ximea_arm_sp.tgz

RUN tar -xzf /tmp/ximea/ximea_arm_sp.tgz;
WORKDIR /tmp/ximea/package
RUN ./install -silent -noudev

ENV XIMEA=/opt/XIMEA \
    LD_LIBRARY_PATH=/opt/XIMEA/lib \
    GENICAM_ROOT_V2_4=/opt/XIMEA/GenICam_v2_4 \
    GENICAM_GENTL64_PATH=/opt/XIMEA/lib \
    PATH=/opt/XIMEA/bin:${PATH}

# Установим инструменты сборки ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO:-humble}-velodyne \
  ros-${ROS_DISTRO:-humble}-velodyne-driver \
  ros-${ROS_DISTRO:-humble}-velodyne-pointcloud \
  ros-${ROS_DISTRO:-humble}-pcl-conversions \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-graph-msgs \
  ros-${ROS_DISTRO}-rviz-visual-tools \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  #ros-${ROS_DISTRO}-moveit \
  #ros-${ROS_DISTRO}-moveit-servo \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  #ros-${ROS_DISTRO}-realsense2-camera \
  #ros-${ROS_DISTRO}-realsense2-description \
  ros-${ROS_DISTRO}-*controller* \
  python3-kivy \
  iperf3 \
  nano \
  python3-pip \
  iputils-ping \
  && rm -rf /var/lib/apt/lists/*


# ===== Создание и сборка рабочей области ROS2 =====
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src
# (если у тебя в контексте есть пакет ximea_cam — он скопируется)
COPY ximea_cam /ros2_ws/src/ximea_cam
COPY lidar_package /ros2_ws/src/lidar_package
COPY stream_camera_lidar /ros2_ws/src/stream_camera_lidar
RUN set -eo pipefail; \
    rosdep init 2>/dev/null || true; \
    rosdep update; \
    set +u; source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; set -u; \
    if compgen -G "src/*/package.xml" > /dev/null; then \
        rosdep install --from-paths src --ignore-src -r -y; \
        colcon build --symlink-install; \
    else \
        echo "No ROS2 packages in /ros2_ws/src, skipping colcon build"; \
    fi

# автосорс окружения
RUN echo 'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc


ENTRYPOINT ["/bin/bash"]
CMD ["-l"]
