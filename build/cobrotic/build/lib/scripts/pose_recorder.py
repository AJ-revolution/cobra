#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_group/display_planned_path/goal',  # OR use your actual end-effector topic
            self.pose_callback,
            10
        )
        self.latest_pose = None
        self.saved_poses = []
        self.yaml_file = os.path.expanduser('~/poses.yaml')

        self.get_logger().info("Pose Recorder Ready! Press Ctrl+C to save and exit.")

    def pose_callback(self, msg):
        self.latest_pose = msg.pose

    def save_pose(self):
        if self.latest_pose is None:
            self.get_logger().warn("No pose received yet.")
            return
        pose_dict = {
            'position': {
                'x': self.latest_pose.position.x,
                'y': self.latest_pose.position.y,
                'z': self.latest_pose.position.z
            },
            'orientation': {
                'x': self.latest_pose.orientation.x,
                'y': self.latest_pose.orientation.y,
                'z': self.latest_pose.orientation.z,
                'w': self.latest_pose.orientation.w
            }
        }
        self.saved_poses.append(pose_dict)
        self.get_logger().info(f"Pose recorded! Total poses: {len(self.saved_poses)}")

    def write_to_yaml(self):
        with open(self.yaml_file, 'w') as f:
            yaml.dump({'poses': self.saved_poses}, f)
        self.get_logger().info(f"Saved poses to {self.yaml_file}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            user_input = input("Press Enter to record pose or type 'q' to quit: ")
            if user_input.lower() == 'q':
                break
            node.save_pose()
    except KeyboardInterrupt:
        pass
    finally:
        node.write_to_yaml()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
