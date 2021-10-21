import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import xacro


def generate_launch_description():
    # Get directories for various projects
    
    ## Racecar directories ##
    racecar_dir = get_package_share_directory('racecar_gazebo')
    racecar_launch_dir = os.path.join(racecar_dir, 'launch')
    racecar_description_dir = get_package_share_directory('racecar_description')

    ## Camera directories ##
    camera_dir = get_package_share_directory('base')
    camera_launch_dir = os.path.join(camera_dir, 'launch')

    # Load and config racecar
    racecar_xacro = os.path.join(racecar_description_dir, 'urdf', 'racecar.xacro')
    xacro_parse = xacro.parse(open(racecar_xacro))
    xacro.process_doc(xacro_parse)
    robot_description_value = xacro_parse.toxml()
    racecar_params = {"robot_description": robot_description_value}

    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[racecar_params]
    )



    # Launch config variables for D435 and T265 cameras
    camera_name1 = LaunchConfiguration('camera_name1')
    device_type1 = LaunchConfiguration('device_type1')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_camera1_name = DeclareLaunchArgument(
            'camera_name1',
            default_value='D435',
            description='Name of the D435 camera')

    declare_device1_type = DeclareLaunchArgument(
            'device_type1',
            default_value='d435',
            description='Type of camera being used')

    declare_base_frame = DeclareLaunchArgument(
            'base_frame_id',
            default_value='/base_link',
            description='')

    declare_odom_frame = DeclareLaunchArgument(
            'odom_frame_id',
            default_value='/odom',
            description='')
    
    # NOTE: THIS IS DEPRECATED VERSION OF LOADING
    load_joint_state_controller = ExecuteProcess(
         cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
         output='screen'
     )

    # NOTE: THIS IS DEPRECATED VERSION OF LOADING CONTROLLER
    controllers = ['left_rear_wheel_velocity_controller', 'right_rear_wheel_velocity_controller', 'left_front_wheel_velocity_controller', 'right_front_wheel_velocity_controller', 'left_steering_hinge_position_controller', 'right_steering_hinge_position_controller']
    load_controllers = [ ExecuteProcess(
                cmd=['ros2', 'control', 'load_start_controller', state],
                output='screen'
                )
        for state in controllers]

    racecar_controller = os.path.join(
        get_package_share_directory("racecar_control"),
        "config",
        "racecar_control.yaml",
        )

    controller_event = RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=load_controllers,
           )
    )

    cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_launch_dir, '')),
            launch_arguments={
                'camera_name1': camera_name1,
                'device_type1': device_type1,
                'base_frame_id': base_frame_id,
                'odom_frame_id': odom_frame_id}.items())
    ])
            

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_camera1_name)
    ld.add_action(declare_device1_type)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_odom_frame)
    
    ld.add_action(node_robot_state_publisher)
    ld.add_action(controller_event)

    ld.add_action(cmd_group)

    return ld
