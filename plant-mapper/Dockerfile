# Base ROS 2 Jazzy desktop image.
# We keep rwthika here because your macOS guide was already based on it.
FROM rwthika/ros2:jazzy-desktop-full-v25.02

# Avoid interactive apt prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# Refresh ROS 2 apt source/key.
# This makes the build more robust in case the base image contains outdated
# ROS repository entries or expired keys.
# -----------------------------------------------------------------------------
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list || true

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor --batch --yes --no-tty -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list

# -----------------------------------------------------------------------------
# Install build tools and ROS packages used for TurtleBot3 simulation,
# Gazebo integration, mapping, navigation, and robot description.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-dev \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-xacro \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Build TurtleBot3 packages into the container's own internal workspace.
# This matches the WSL approach of keeping the prebuilt TB3 stack in
# /opt/turtlebot3_ws.
# -----------------------------------------------------------------------------
WORKDIR /opt/turtlebot3_ws

RUN mkdir -p src && cd src && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Use bash so "source" works inside RUN commands
SHELL ["/bin/bash", "-lc"]

RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y || true && \
    colcon build --event-handlers console_direct+ \
      --packages-select \
        turtlebot3_msgs \
        turtlebot3_description \
        turtlebot3_gazebo \
        turtlebot3_teleop \
        turtlebot3_cartographer \
        turtlebot3_navigation2

# -----------------------------------------------------------------------------
# Auto-source ROS and TurtleBot3 setup for interactive shells.
# -----------------------------------------------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /opt/turtlebot3_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo 'if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> /root/.bashrc

# Start a shell by default
CMD ["/bin/bash"]