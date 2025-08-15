import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from serial import SerialException

class JointStateToArduino(Node):
    def __init__(self):
        super().__init__('joint_state_to_arduino')

        # Declare parameters for flexibility
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic', '/joint_states')

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            topic,
            self.listener_callback,
            10
        )

        # Try to connect to Arduino
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port} at {baudrate} baud")
        except SerialException as e:
            self.arduino = None
            self.get_logger().warn(f"Failed to connect to Arduino: {e}")

    def listener_callback(self, msg: JointState):
        if self.arduino is None:
            self.get_logger().warn("Arduino not connected. Skipping message.")
            return

        try:
            # Convert radians to degrees
            degrees = [int(angle * 180.0 / 3.14159) for angle in msg.position]
            data_str = ','.join(map(str, degrees)) + '\n'

            # Send to Arduino
            self.arduino.write(data_str.encode())
            self.get_logger().info(f"Sent to Arduino: {data_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        if node.arduino:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()