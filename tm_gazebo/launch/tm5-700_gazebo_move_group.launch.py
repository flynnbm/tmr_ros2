############################################################################################### 
#  tm5-700_run_move_group.launch.py
#   
#  Various portions of the code are based on original source from 
#  The reference: "https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo/launch"
#  and are used in accordance with the following license.
#  "https://github.com/moveit/moveit2/blob/main/LICENSE.txt"
############################################################################################### 

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node

import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import SetParameter

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    args = []
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # Configure robot_description
    moveit_config_path = 'tm5-700_moveit_config'    
    srdf_path = 'config/tm5-700.srdf'
    rviz_path = '/launch/run_move_group.rviz'     

    # Robot SDF Description
    pkg_project_description = get_package_share_directory('tm_description')
    robot_description_config  =  xacro.process_file(os.path.join(pkg_project_description, 'models', 'tm5-700', 'model.sdf'))
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF Configuration
    robot_description_semantic_config = load_file(moveit_config_path, srdf_path)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_path, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_path, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(moveit_config_path, 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    # Trajectory Execution Functionality
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Joint limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_path, 'config/joint_limits.yaml'
        )
    }

    # Gazebo World and Simulator
    pkg_project_gazebo = get_package_share_directory('tm_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim_DART = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', f"-r {pkg_project_gazebo}/worlds/tm5-700.sdf")
        ],
    )

    gz_sim_bullet_featherstone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', f"-r {pkg_project_gazebo}/worlds/tm5-700.sdf --physics-engine gz-physics-bullet-featherstone-plugin")
        ],
    )

    # ROS <--> Gazebo Sim Communication Bridge
    pkg_project_bringup = get_package_share_directory('tm_gazebo')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'tm5-700_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,           
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    # RViz configuration
    rviz_config_file = (
        get_package_share_directory(moveit_config_path) + rviz_path
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    load_arm_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tmr_arm_controller"],
        output="screen",
    )

    load_hand_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller"],
        output="screen",
    )

    set_sim_time = SetParameter(
        name="use_sim_time",
        value=True
    )

    # Launching all the nodes
    return LaunchDescription(
        [
            gz_sim_DART,
            bridge,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            load_arm_controller,
            set_sim_time,
        ]
    )
