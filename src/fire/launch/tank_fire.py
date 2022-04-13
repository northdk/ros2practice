from doctest import OutputChecker
from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def read_yaml(path):
  with open(path, 'r') as f:
    yaml_data = yaml.load(f.read())
  return yaml_data


def parse_yaml() -> LaunchDescription:
  launch_directory = get_package_share_directory('fire')
  yaml_data = read_yaml(os.path.join(launch_directory, 'params','node.yaml'))
  launch_description_impl = LaunchDescription()
  for data in yaml_data["launch_nodes"]:
    print(data.keys(), data.values())
    define_name = list(data.keys())[0]
    launch_description_impl.add_action(
      Node(
      package=data[define_name]['package'],
      # namespace=None,
      executable=data[define_name]['executable'],
      output='screen',
      respawn=data[define_name]['respawn']
    )
    )
  return launch_description_impl


def generate_launch_description():
  return parse_yaml()