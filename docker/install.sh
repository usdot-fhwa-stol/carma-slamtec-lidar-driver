#!/bin/bash

#  Copyright (C) 2023 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Source ros2
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install_ros2/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/ros/foxy/setup.bash
fi

# Don't proceed in Continuous Integration environment
if [[ "$CI" == "true" ]]; then
    exit
fi

# Build wrapper
cd ~
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS2_PACKAGES
else
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to slamtec_lidar_driver_wrapper driver_shutdown_ros2
fi

# # Change to our ROS2 workspace
# cd /home/carma

# # Install all required dependencies for the source code we pulled.
# sudo apt update
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

# # Build everything we need for our drivers
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   --packages-up-to slamtec_lidar_driver_wrapper

# # Add rosbridge
# sudo apt install ros-foxy-rosbridge-suite
