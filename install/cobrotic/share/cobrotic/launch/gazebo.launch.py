from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_cobrotic = FindPackageShare("cobrotic")
    default_world = PathJoinSubstitution([pkg_cobrotic, "worlds", "empty.world"])
    urdf_file = PathJoinSubstitution([pkg_cobrotic, "urdf", "cobra.urdf"])
    controllers_file = PathJoinSubstitution([pkg_cobrotic, "config", "ros2_controllers.yaml"])

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file"
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # ros2_control node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[urdf_file, controllers_file],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cobra", "-file", urdf_file],
        output="screen",
    )

    # Controller spawners with delays
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    cobra_arm_controller_spawner = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cobra_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    hand_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hand_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        controller_manager,
        spawn_entity,
        joint_state_broadcaster_spawner,
        cobra_arm_controller_spawner,
        hand_controller_spawner,
    ])
