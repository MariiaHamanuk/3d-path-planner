FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive

# sim, bridges, recording, dev tools, terminal tooling, rendering fallback
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rosbag2-storage-mcap \
        ros-dev-tools \
        tmux tmuxinator \
        mesa-utils libegl1 libgl1 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot SITL (ArduCopter) + MAVProxy console, and the ardupilot_gazebo plugin that connects it to Gazebo
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake ccache gawk wget zip unzip \
        python3-pip python3-dev python3-lxml python3-numpy \
        rapidjson-dev libopencv-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
        empy==3.3.4 pexpect future pymavlink MAVProxy

RUN git clone --depth 1 --branch Copter-4.5.7 --recurse-submodules --shallow-submodules -j8 \
        https://github.com/ArduPilot/ardupilot /opt/ardupilot && \
    cd /opt/ardupilot && ./waf configure --board sitl && ./waf copter -j"$(nproc)"

RUN . /opt/ros/jazzy/setup.sh && \
    git init -q /opt/ardupilot_gazebo && cd /opt/ardupilot_gazebo && \
    git fetch -q --depth 1 https://github.com/ArduPilot/ardupilot_gazebo 80dc9664dda7efd6c5d1a71a79198d1d6b708c0e && \
    git checkout -q FETCH_HEAD && \
    cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j"$(nproc)"

# Environment: UTF-8 text, and GPU graphics/EGL libraries (only used with --gpus all)
ENV LANG=C.UTF-8 \
    NVIDIA_DRIVER_CAPABILITIES=all

# Source ROS and the colcon workspace in every shell (also `docker exec` ones)
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /etc/bash.bashrc

COPY docker/.tmux.conf /root/.tmux.conf
# Where gz would look for resources: first in models/, then in worlds/
ENV GZ_SIM_RESOURCE_PATH=/workspace/simulation/models:/workspace/simulation/worlds
# ArduPilotPlugin (libArduPilotPlugin.so) for the drone model; sim_vehicle.py on PATH
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV PATH=/opt/ardupilot/Tools/autotest:$PATH
WORKDIR /workspace