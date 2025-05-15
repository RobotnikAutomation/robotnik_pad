from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name='robot_id',
            default_value=EnvironmentVariable('ROBOT_ID', default_value='robot'),
            description='Robot ID'
        ),
        DeclareLaunchArgument(
            name='log_level',
            default_value=EnvironmentVariable('LOG_LEVEL', default_value='info'),
            description='Log level',
        ),

        # PS5 pad driver
        DeclareLaunchArgument(
            name='desired_freq',
            default_value='40.0',
            description='Desired frequency for PS5 pad driver',
        ),
        DeclareLaunchArgument(
            name='serial_number',
            default_value='',
            description='Serial number of the PS5 pad',
        ),
        DeclareLaunchArgument(
            name='deadzone',
            default_value=EnvironmentVariable('ROBOT_PAD_DEADZONE', default_value='0.1'),
            description='Deadzone for the joystick',
        ),

        # PS4 pad driver (joy)
        DeclareLaunchArgument(
            name='device',
            default_value=EnvironmentVariable('ROBOT_PAD_DEV', default_value='/dev/input/js_base'),
            description='Device of the PS4 pad',
        ),
        DeclareLaunchArgument(
            name='autorepeat_rate',
            default_value='0.0',
            description='Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message',
        ),
        DeclareLaunchArgument(
            name='pad_model',
            default_value=EnvironmentVariable('ROBOT_PAD_MODEL', default_value='ps5'),
            description='Pad model: ps4 or ps5',
        )
    ]

    config_file = os.path.join(get_package_share_directory("robotnik_pad"),
                'config', 'pad.yaml')

    # Create LaunchConfigurations
    robot_id = LaunchConfiguration('robot_id')
    log_level = LaunchConfiguration('log_level')
    desired_freq = LaunchConfiguration('desired_freq')
    serial_number = LaunchConfiguration('serial_number')
    deadzone = LaunchConfiguration('deadzone')
    device = LaunchConfiguration('device')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')
    pad_model = LaunchConfiguration('pad_model')

    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace=robot_id),

            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'dev': device,
                    'deadzone': deadzone,
                    'autorepeat_rate': autorepeat_rate
                }],
                arguments=['--ros-args', '--log-level', log_level],
                condition=IfCondition(
                    PythonExpression(['"', pad_model, '" == "ps4"'])
                )
            ),

            Node(
                package='ps5_pad_driver',
                executable='robotnik_joy',
                name='joy_node',
                output='screen',
                parameters=[{
                    'serial_number': serial_number,
                    'deadzone': deadzone,
                    'desired_freq': desired_freq
                }],
                condition=IfCondition(
                    PythonExpression(['"', pad_model, '" == "ps5"'])
                )
            ),

            Node(
                package='robotnik_pad',
                executable='robotnik_pad',
                name='robotnik_pad',
                output='screen',
                parameters=[config_file],
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )

    return LaunchDescription(declared_arguments + [load_nodes])