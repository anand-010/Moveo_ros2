import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import ( OnProcessStart, OnProcessExit )
from launch. launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveo_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor( publish_robot_description=True, publish_robot_description_semantic=True, publish_planning_scene=True )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    use_sim_time = { "use_sim_time": True }
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )
    rviz_base = os.path.join(
        get_package_share_directory(
            "moveo_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        arguments=['-d', rviz_full_config],
        remappings=[("/joint_states", "/joint_states_desired")]
    )
    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0","world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    spawn_entity = Node( package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'moveo_arm'],
        output='screen')
        # Create the Gazebo node
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    # )
    # gazebo = ExecuteProcess(
    #     cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"], output="screen"
    # )
    return LaunchDescription([
        static_tf_node,
        robot_state_publisher,
        spawn_entity,
        gazebo,
        move_group_node,
        rviz_node
    ])