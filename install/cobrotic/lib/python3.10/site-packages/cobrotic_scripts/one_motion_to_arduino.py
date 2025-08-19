# my_arduino_bridge/one_motion_to_arduino.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_angles',
            self.listener_callback,
            10)

        self.arduino = serial.Serial('/dev/ttyACM0', 115200)

    def listener_callback(self, msg):
        if len(msg.data) >= 2:
            print(msg)
            angles = [max(0, min(180, int(a))) for a in msg.data[:4]]
            command = ','.join(map(str, angles)) + '\n'
            self.arduino.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()