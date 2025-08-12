FROM osrf/ros:humble-desktop-full

# Copy the install script for xsarm to the Docker image home directory
COPY xsarm_amd64_install.sh /root/xsarm_amd64_install.sh

# Set the working directory to the home directory
WORKDIR /root

# Make the install script executable
RUN chmod +x xsarm_amd64_install.sh

# Run the install script
RUN ./xsarm_amd64_install.sh -d humble -p /root/interbotix_ws -n

# Source the ROS setup script
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Source the workspace setup script
RUN echo "source /root/interbotix_ws/install/setup.bash" >> /root/.bashrc

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Rebuild the workspace
WORKDIR /root/interbotix_ws
RUN colcon build

# Set environment variables
ENV GAZEBO_MODEL_URI=''
ENV GAZEBO_MODEL_DATABASE_URI=''
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/usr/share/gazebo-11/media:/usr/share/gazebo-11/models:/root/.gazebo/models

# Download gazebo models and move the to #HOME/.gazebo/models
RUN git clone https://github.com/osrf/gazebo_models.git /root/gazebo_models
RUN mkdir -p /root/.gazebo/models
RUN mv /root/gazebo_models/* /root/.gazebo/models/

# Uninstall any existing moveit packages
RUN apt remove ros-$ROS_DISTRO-moveit* -y

# Clone the moveit2 humble branch noveit_py porting
WORKDIR /root/interbotix_ws/src
RUN git clone -b pr-humble-python-bindings https://github.com/CNR-STIIMA-IRAS/moveit2.git

# Install moveit2 from source
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
WORKDIR /root/interbotix_ws

# Install build dependencies for moveit2
RUN apt update && apt install -y build-essential cmake git python3-colcon-common-extensions \
    python3-flake8 python3-rosdep python3-setuptools python3-vcstool wget ros-humble-ament-cmake

RUN apt update
RUN apt dist-upgrade -y
RUN rosdep update

# Rebuild the workspace with the moveit2 fork
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    --parallel-workers 2 \
    --cmake-clean-cache"
# Remark: to build in debug mode, use DCMAKE_BUILD_TYPE=RelWithDebInfo instead of Release

# Place the paths of the interbotix resources before the panda resources
RUN echo 'export AMENT_PREFIX_PATH=/root/interbotix_ws/install:$AMENT_PREFIX_PATH' >> /root/.bashrc
RUN echo 'export CMAKE_PREFIX_PATH=/root/interbotix_ws/install:$CMAKE_PREFIX_PATH' >> /root/.bashrc

# Set default command
CMD ["bash"]

ENTRYPOINT ["/entrypoint.sh"]