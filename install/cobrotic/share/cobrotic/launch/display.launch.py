from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Find the package share directory
    cobrotic_path = FindPackageShare('cobrotic')

    # URDF and RViz paths
    default_model_path = PathJoinSubstitution([cobrotic_path, 'urdf', 'cobra.urdf'])
    default_rviz_config_path = PathJoinSubstitution([cobrotic_path, 'rviz', 'cobra.rviz'])

    # GUI argument for joint_state_publisher
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Enable joint_state_publisher GUI'
    )
    ld.add_action(gui_arg)

    # Launch arguments for URDF model and RViz config
    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF file'
    ))
    ld.add_action(DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    ))

    # Include the display launch (from urdf_launch package) to show URDF in RViz
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'cobrotic',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')
        }.items()
    ))

    return ld
