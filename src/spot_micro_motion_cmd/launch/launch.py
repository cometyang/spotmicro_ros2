from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import os
import launch_ros.actions
import pathlib

parameters_file_name = 'spot_micro_motion_cmd.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/config/' + parameters_file_name
    print(parameters_file_path)
    return LaunchDescription([
        Node(
            package='i2cpwmboard',
            executable='i2cpwm_board',
            name='i2cpwmboard'
        ),
        Node(
            package='spot_micro_motion_cmd',
            executable='spot_micro_motion_cmd_node',
            name="spot_micro_motion_cmd"
            output='screen',
            parameters=[
                parameters_file_path
            ],
         ),
    ])