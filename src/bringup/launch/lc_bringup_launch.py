#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_shutdown import OnShutdown
from pycommon import RewrittenYaml, Yaml_launcher, Load_yaml, get_namespace


def generate_launch_description():

    # Define arguments
    bringup_dir = get_package_share_directory('tank_bringup')
    get_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value=get_namespace(),
        description='Namespace of all nodes')
    set_parameter_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'default_param.yaml'),
        description='Path to paramaters YAML file')
    shutdown_cmd = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            LogInfo(msg="ROS Apps is exiting.")]))

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        convert_types=True)

    launch_nodes = Load_yaml(os.path.join(bringup_dir, 'params', 'launch_nodes.yaml'), True)
    remappings = Load_yaml(os.path.join(bringup_dir, 'params', 'remappings.yaml'))
    launch_groups = Load_yaml(os.path.join(bringup_dir, 'params', 'launch_groups.yaml'))

    return Yaml_launcher(
        launch_param=[
            get_namespace_cmd, 
            set_parameter_cmd, 
            shutdown_cmd],
        launch_nodes_yaml=launch_nodes,
        launch_groups_yaml=launch_groups,
        nodes_param=configured_params,
        namespace=namespace,
        remappings=remappings)