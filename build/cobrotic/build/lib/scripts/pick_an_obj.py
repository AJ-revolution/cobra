import rclpy
import yaml
import os
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, roscpp_initialize

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rclpy.init()
    roscpp_initialize([])

    arm = MoveGroupCommander('manipulator')

    # Load configs
    waypoints_file = os.path.join(os.path.dirname(__file__), 'waypoints.yaml')
    velocity_file = os.path.join(os.path.dirname(__file__), 'velocity_config.yaml')

    waypoints_data = load_yaml(waypoints_file)
    velocity_data = load_yaml(velocity_file)

    arm.set_max_velocity_scaling_factor(velocity_data.get('velocity_scaling', 0.2))
    arm.set_max_acceleration_scaling_factor(velocity_data.get('acceleration_scaling', 0.1))

    for wp in waypoints_data['waypoints']:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = wp['position']
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = wp['orientation']

        arm.set_pose_target(pose)
        arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
