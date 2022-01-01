import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
			    IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
import xacro


def generate_launch_description():
	# Get directories for various projects
	config_dir = os.path.join(get_package_share_directory('racecar_realsense'), 'config')
	
	## Racecar directories ##
	racecar_description_dir = get_package_share_directory('racecar_description')

	## Camera directories ##
	d435_config = os.path.join(config_dir, 'D435.yaml')
	t265_config = os.path.join(config_dir, 'T265.yaml')

	## Depth image to Laserscan directories ##
	depthimage_config = os.path.join(config_dir, 'depthimage.yaml')

	## SLAM Toolbox directories ##
	slam_toolbox_config = os.path.join(config_dir, 'slam_toolbox.yaml');


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

	# Load cameras
	d435_node = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='D435',
		output='screen',
		parameters=[d435_config]
	)    

	t265_node = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='T265',
		output='screen',
		parameters=[t265_config],
		remappings=[('/T265/odom/sample', '/odom')]
	)    

	depthimage_to_laserscan_node = Node(
		package='depthimage_to_laserscan',
		executable='depthimage_to_laserscan_node',
		output='screen',
		parameters=[depthimage_config],
		remappings=[('depth','/depth/image_rect_raw'),
			('depth_camera_info', '/depth/camera_info')],
	)

	start_async_slam_toolbox_node = Node(
        parameters=[slam_toolbox_config],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
	)
		

	# Create the launch description and populate
	ld = LaunchDescription()
	
	# Launch robot specific transforms
	ld.add_action(node_robot_state_publisher)

	# Camera launches
	ld.add_action(d435_node)
	ld.add_action(t265_node)

	# Mapping launches
	ld.add_action(depthimage_to_laserscan_node)
	ld.add_action(start_async_slam_toolbox_node)

	return ld
