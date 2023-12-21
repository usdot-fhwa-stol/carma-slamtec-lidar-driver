#  Copyright (C) 2023 LEIDOS.

#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at

#  http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, launch_configuration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import set_remap
from launch_ros.actions import set_parameter

import os

def generate_launch_description():

    # Declare the launch arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value = 'DEBUG', description = "Log level to print.", choices=["DEBUG","INFO","WARN","ERROR","FATAL"])

    # Args for driver
    frame_id = LaunchConfiguration('frame_id')
    declare_frame_id = DeclareLaunchArgument(name = 'frame_id', default_value = "slamtec", description="The frame id to use for the scan data")

    serial_port = LaunchConfiguration('serial_port')
    declare_serial_port = DeclareLaunchArgument(name = 'serial_port', default_value = '/dev/sensors/rplidar', description="Serial port of the device")

    serial_baudrate = LaunchConfiguration('serial_baudrate')
    declare_serial_baudrate = DeclareLaunchArgument(name = 'serial_baudrate', default_value = '256000', description='Baud rate for serial communication')

    inverted = LaunchConfiguration('inverted')
    declare_inverted = DeclareLaunchArgument(name = 'inverted', default_value = 'False', description="True to flip the scan if the lidar is upside down")

    angle_compensate = LaunchConfiguration('angle_compensate')
    declare_angle_compensate = DeclareLaunchArgument(name = 'angle_compensate', default_value = 'True')

    # Args for Pointcloud
    slamtec_lidar_driver_wrapper_pkg = get_package_share_directory('slamtec_lidar_driver_wrapper')
    sllidar_ros2_pkg = get_package_share_directory('sllidar_ros2')

    #  Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('slamtec_lidar_driver_wrapper'), 'config/parameters.yaml')

    # Define Slamtec ROS2 driver
    slamtec_driver_group = GroupAction(
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', sllidar_ros2_pkg, '/launch', '/sllidar_s1_launch.py']),
                launch_arguments = [
                    {'frame_id' : frame_id},
                    {'serial_port' : serial_port},
                    {'serial_baudrate' : serial_baudrate},
                    {'inverted' : inverted},
                    {'angle_compensate' : angle_compensate}
                ]
            ),
        ]
    )

    # Launch node(s) in a carma container to allow logging to be configured
    slamtec_lidar_wrapper_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='slamtec_lidar_driver_wrapper_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            # Launch the core node(s)
            ComposableNode(
				package='slamtec_lidar_driver_wrapper',
				plugin='slamtec_lidar_driver_wrapper::ComposableNode',
				name='slamtec_lidar_driver_wrapper',
                namespace=GetCurrentNamespace(),
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : log_level }
                ],
                parameters=[ param_file_path ]
            ),
            ComposableNode(
                package='slamtec_lidar_driver_wrapper',
				plugin='slamtec_lidar_driver_wrapper::lidar_scan_to_point_cloud2',
                name='slamtec_convertor_node',
                namespace=GetCurrentNamespace(),
                parameters = [
                    {'frame_id' : frame_id},
                    {'serial_port' : serial_port},
                    {'serial_baudrate' : serial_baudrate},
                    {'inverted' : inverted},
                    {'angle_compensate' : angle_compensate}
                ]
            )
        ]
    )

    return LaunchDescription([
        # Specify Args
        declare_log_level_arg,
        declare_frame_id,
        declare_serial_port,
        declare_serial_baudrate,
        declare_inverted,
        declare_angle_compensate,
        # Specify Nodes
        slamtec_driver_group,
        slamtec_lidar_wrapper_container
    ])