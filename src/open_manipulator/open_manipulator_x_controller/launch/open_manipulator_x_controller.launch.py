# launch/open_manipulator_x_controller.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_yaml = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_controller'),
        'config',
        'waypoints.yaml'
    ])

    waypoints_yaml_arg = DeclareLaunchArgument(
        'waypoints_yaml',
        default_value=default_yaml,
        description='Absolute path to waypoints YAML file'
    )

    return LaunchDescription([
        waypoints_yaml_arg,
        Node(
            package='open_manipulator_x_controller',
            executable='controller_node',           
            name='open_manipulator_x_controller',    
            output='screen',
            parameters=[{
                'waypoints_yaml': LaunchConfiguration('waypoints_yaml'),

            }]
        ),
    ])

