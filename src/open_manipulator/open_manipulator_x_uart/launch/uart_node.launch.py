from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    baud_arg = DeclareLaunchArgument('baudrate', default_value='115200')
    topic_arg = DeclareLaunchArgument('topic', default_value='nucleo_uart')

     # 초음파 Receiver 인자
    us_port_arg = DeclareLaunchArgument('us_port', default_value='/dev/ttyACM1')
    us_baud_arg = DeclareLaunchArgument('us_baudrate', default_value='115200')
    us_topic_arg = DeclareLaunchArgument('us_topic', default_value='ultrasonic_distance')

    uart_node = Node(
        package='open_manipulator_x_uart',
        executable='uart_receiver',
        name='uart_receiver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'topic_name': LaunchConfiguration('topic'),
            'csv_hint': True,
            'reconnect_sec': 3.0,
        }],
        output='screen'
    )

    # 새로운 Ultrasonic Receiver 노드
    ultrasonic_node = Node(
        package='open_manipulator_x_uart',
        executable='ultrasonic_receiver',
        name='ultrasonic_receiver',
        parameters=[{
            'port': LaunchConfiguration('us_port'),
            'baudrate': LaunchConfiguration('us_baudrate'),
            'topic_name': LaunchConfiguration('us_topic'),
        }],
        output='screen'
    )


    return LaunchDescription([
        port_arg, baud_arg, topic_arg,
        us_port_arg, us_baud_arg, us_topic_arg,
        uart_node,
        ultrasonic_node
    ])