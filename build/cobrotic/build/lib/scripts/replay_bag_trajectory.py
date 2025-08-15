import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import rosbag2_py
import rclpy.serialization
import time

# CHANGE THIS to match your robot joints
JOINT_NAMES = ['bottom_to_pedestal', 'pedestal_to_arm', 'arm_to_forearm']

class BagTrajectoryReplayer(Node):
    def __init__(self, bag_path, loops=1):
        super().__init__('bag_trajectory_replayer')
        self.publisher = self.create_publisher(JointTrajectory, '/cobra_arm_controller/joint_trajectory', 10)
        self.trajectory_points = self.load_bag_as_points(bag_path)
        self.loops = loops

    def load_bag_as_points(self, bag_path):
        self.get_logger().info(f"Loading bag from {bag_path}...")
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        points = []
        start_time = None

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != "/cobra_arm_controller/controller_state":
                continue

            msg = JointTrajectoryControllerState()
            rclpy.serialization.deserialize_message(data, msg)

            if start_time is None:
                start_time = t

            point = JointTrajectoryPoint()
            point.positions = msg.actual.positions
            dt = (t - start_time) / 1e9
            point.time_from_start.sec = int(dt)
            point.time_from_start.nanosec = int((dt - int(dt)) * 1e9)
            points.append(point)

        self.get_logger().info(f"Loaded {len(points)} points from bag.")
        return points

    def play(self):
        for loop_idx in range(self.loops):
            self.get_logger().info(f"Playing loop {loop_idx + 1}/{self.loops}...")
            traj = JointTrajectory()
            traj.joint_names = JOINT_NAMES
            traj.points = self.trajectory_points
            self.publisher.publish(traj)
            duration = self.trajectory_points[-1].time_from_start.sec \
                     + self.trajectory_points[-1].time_from_start.nanosec / 1e9
            time.sleep(duration + 1.0)  # wait until playback finishes

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 replay_bag_trajectory.py <bag_path> [loops]")
        return
    bag_path = sys.argv[1]
    loops = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    rclpy.init()
    node = BagTrajectoryReplayer(bag_path, loops)
    node.play()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
