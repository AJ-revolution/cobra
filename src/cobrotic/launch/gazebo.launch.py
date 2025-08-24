from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # --------------------------
    # Package paths
    # --------------------------
    pkg_cobrotic = FindPackageShare("cobrotic")
    pkg_cobrotic_moveit = FindPackageShare("cobrotic_moveit")
    pkg_gazebo_ros = FindPackageShare("gazebo_ros")

    # Files
    default_world = PathJoinSubstitution([pkg_cobrotic, "worlds", "empty.world"])
    urdf_file = PathJoinSubstitution([pkg_cobrotic, "urdf", "cobra.urdf"])
    rviz_config_file = PathJoinSubstitution([pkg_cobrotic_moveit, "config", "moveit.rviz"])
    gazebo_launch_file = PathJoinSubstitution([pkg_gazebo_ros, "launch", "gazebo.launch.py"])

    # --------------------------
    # Launch arguments
    # --------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file"
    )

    # --------------------------
    # Launch Gazebo
    # --------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # --------------------------
    # Spawn robot in Gazebo
    # --------------------------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cobra", "-file", urdf_file],
        output="screen",
    )

    # --------------------------
    # Get the config from MoveIt2
    # --------------------------
    moveit_config = MoveItConfigsBuilder("cobra", package_name="cobrotic_moveit").to_moveit_configs()

    # --------------------------
    # Controller Manager Node
    # --------------------------
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen"
    )

    # --------------------------
    # Controller Spawners (delayed)
    # --------------------------
    joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )]
    )

    cobra_arm_controller = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cobra_arm_controller"],
            output="screen"
        )]
    )

    hand_controller = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hand_controller"],
            output="screen"
        )]
    )

    # --------------------------
    # MoveIt2
    # --------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": moveit_config.robot_description},
            {"robot_description_semantic": moveit_config.robot_description_semantic},
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"fake_controller": False},  # use real Gazebo controllers
        ]
    )

    # --------------------------
    # RViz (delayed)
    # --------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config_file)],
        parameters=[
            {"robot_description": moveit_config.robot_description},
            {"robot_description_semantic": moveit_config.robot_description_semantic},
        ],
        output="screen"
    )

    rviz_timer = TimerAction(
        period=4.5,  # wait for Gazebo & controllers
        actions=[rviz_node]
    )

    # --------------------------
    # Return Launch Description
    # --------------------------
    return LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        controller_manager_node,
        joint_state_broadcaster,
        cobra_arm_controller,
        hand_controller,
        move_group,
        rviz_timer
    ])
