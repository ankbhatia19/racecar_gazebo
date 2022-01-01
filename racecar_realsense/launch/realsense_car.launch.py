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
	
	## Racecar directories ##
	racecar_dir = get_package_share_directory('racecar_gazebo')
	racecar_launch_dir = os.path.join(racecar_dir, 'launch')
	racecar_description_dir = get_package_share_directory('racecar_description')

	## Camera directories ##
	config_dir = os.path.join(get_package_share_directory('racecar_realsense'), 'config')
	d435_config = os.path.join(config_dir, 'D435.yaml')
	t265_config = os.path.join(config_dir, 'T265.yaml')

	## Depth image to Laserscan directories ##
	depthimage_config = os.path.join(config_dir, 'depthimage.yaml')

	## Cartographer directories ##
	cartographer_config = 'racecar_cartographer.lua'

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
		#namespace='D435',
		output='screen',
		parameters=[d435_config],
		remappings=[]
	)    

	t265_node = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='T265',
		#namespace='T265',
		output='screen',
		parameters=[t265_config],
		remappings=[('/T265/odom/sample', '/odom')]
	)    

	# Load cartographer
	map_odom_statictf = Node(
		## Configure the TF of the robot to the origin of the map coordinates
		package='tf2_ros',
		executable='static_transform_publisher',
		output='screen',
		arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_frame']
	)

	depthimage_to_laserscan_node = Node(
		package='depthimage_to_laserscan',
		executable='depthimage_to_laserscan_node',
		name='scan',
		output='screen',
		parameters=[depthimage_config],
		remappings=[('depth','/depth/image_rect_raw'),
			('depth_camera_info', '/depth/camera_info')],
	)

	cartographer_node = Node(
		package='cartographer_ros',
		executable='cartographer_node',
		output='log',
		arguments=['-configuration_directory', config_dir, '-configuration_basename', cartographer_config]
	)

	start_async_slam_toolbox_node = Node(
        parameters=[slam_toolbox_config],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
	)

	occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
	)
	
	stdout_use_envvar = SetEnvironmentVariable(
		'RCUTILS_LOGGING_USE_STDOUT', '1')

		

	# Create the launch description and populate
	ld = LaunchDescription()

	# Set environment variables
	# ld.add_action(stdout_use_envvar)
	
	# Launch robot specific transforms
	ld.add_action(node_robot_state_publisher)

	# Camera launches
	ld.add_action(d435_node)
	ld.add_action(t265_node)

	# Cartographer launches
	# ld.add_action(map_odom_statictf)
	ld.add_action(depthimage_to_laserscan_node)
	# ld.add_action(cartographer_node)
	ld.add_action(start_async_slam_toolbox_node)
	# 	ld.add_action(occupancy_grid_node)

	return ld
