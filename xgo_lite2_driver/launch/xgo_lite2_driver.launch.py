"""Launch the XGO-Lite2 driver node with a YAML parameter file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    default_params = PathJoinSubstitution([
        FindPackageShare('xgo_lite2_driver'), 'config', 'xgo_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the ROS 2 parameter YAML file.',
        ),
        Node(
            package='xgo_lite2_driver',
            executable='xgo_node',
            name='xgo_lite2_driver',
            output='screen',
            parameters=[params_file],
            emulate_tty=True,
        ),
    ])
