from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from robotnik_common.launch import ExtendedArgument
from robotnik_common.launch import AddArgumentParser
from robotnik_common.launch import RewrittenYaml
from launch.substitutions import PythonExpression

def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
        use_env=True,
        environment='ROBOT_ID',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='log_level',
        description='Log level',
        default_value='info',
        use_env=True,
        environment='LOG_LEVEL',
    )
    add_to_launcher.add_arg(arg)

    # PS5 pad driver
    arg = ExtendedArgument(
        name='desired_freq',
        description = 'Desired frequency',
        default_value='40.0',
        use_env=False
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='serial_number',
        description='Serial number of the PS5 pad',
        default_value='',
        use_env=False
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name = 'deadzone',
        description = 'Deadzone for the joystick',
        default_value = '0.1',
        use_env = True,
        environment = 'ROBOT_PAD_DEADZONE'
    )
    add_to_launcher.add_arg(arg)

    # PS4 pad driver (joy)
    arg = ExtendedArgument(
        name='device',
        description='Device of the PS4 pad',
        default_value='/dev/input/js_base',
        use_env=True,
        environment='ROBOT_PAD_DEV'
    )
    add_to_launcher.add_arg(arg)

    # Robotnik pad
    arg = ExtendedArgument(
        name='config_file',
        description='Path to the configuration file',
        default_value=[
            FindPackageShare("robotnik_pad"),
            '/config/pad.yaml'
        ],
        use_env=False
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name = 'pad_model',
        description = 'Pad model',
        default_value = 'ps5',
        use_env = True,
        environment = 'ROBOT_PAD_MODEL'
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()
    config_file_rewritten = RewrittenYaml(
        source_file = params['config_file'],
        root_key = params['robot_id'],
        param_rewrites = {},
        convert_types = True
    )

    load_nodes = GroupAction(
        actions = [
            PushRosNamespace(
                namespace = params['robot_id'],
            ),
            Node(
                package = 'joy_linux',
                executable = 'joy_linux_node',
                name = 'joy_node',
                output = 'screen',
                parameters = [
                    {
                        'device': params['device'],
                        'deadzone': params['deadzone'],
                    }
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    params['log_level']
                ],
                condition = IfCondition(
                    PythonExpression(
                        ['"', params['pad_model'], '" == "ps4"']
                    )
                )
            ),
            Node(
                package = 'ps5_pad_driver',
                executable = 'robotnik_joy',
                name = 'joy_node',
                output = 'screen',
                parameters = [
                    {
                        'serial_number': params['serial_number'],
                        'deadzone': params['deadzone'],
                        'desired_freq': params['desired_freq'],
                    }
                ],
                condition = IfCondition(
                    PythonExpression(
                        ['"', params['pad_model'], '" == "ps5"']
                    )
                ),
            ),
            Node(
                package = 'robotnik_pad',
                executable = 'robotnik_pad',
                name = 'robotnik_pad',
                output = 'screen',
                parameters = [config_file_rewritten],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    params['log_level']
                ]
            )
        ]
    )

    ld.add_action(load_nodes)
    return ld
