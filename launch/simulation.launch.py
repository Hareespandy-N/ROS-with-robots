"""Start only the numerical simulator; no robot drivers or Nav2 launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='warehouse'),
        DeclareLaunchArgument('controller', default_value='regulated'),
        DeclareLaunchArgument('max_steps', default_value='0'),
        Node(package='amr_research_lab', executable='sim_node', output='screen',
             parameters=[{'scenario': LaunchConfiguration('scenario'),
                          'controller': LaunchConfiguration('controller'),
                          'max_steps': ParameterValue(LaunchConfiguration('max_steps'), value_type=int)}]),
    ])
