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

FROM usdotfhwastol/c1t-base:carma-system-4.2.0 as setup

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.bash
RUN ~/src/docker/install.sh

FROM usdotfhwastol/c1t-base:carma-system-4.2.0

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-slamtec-lidar-driver"
LABEL org.label-schema.description="carma 1tenth slamted Lidar driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-slamtec-lidar-driver/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=setup /home/carma/install /opt/carma/install_ros2
# Copy dependencies installed
COPY --from=setup /opt/ros/foxy /opt/ros/foxy

CMD [ "wait-for-it.sh", "localhost:11311", "--", "ros2", "launch", "slamtec_lidar_driver_wrapper", "slamtec_lidar_driver_wrapper.launch.py"]

# # Disable the default entrypoint because it breaks our drivers
# ENTRYPOINT []
