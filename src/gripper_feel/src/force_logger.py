import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import time

class ForceLogger(Node):
    def __init__(self):
        super().__init__('force_logger')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10
        )
        self.data = []
        self.start_time = time.time()
        self.get_logger().info("Force Logger Node started")

    def wrench_callback(self, msg):
        elapsed_time = time.time() - self.start_time
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        torque_x = msg.wrench.torque.x
        torque_y = msg.wrench.torque.y
        torque_z = msg.wrench.torque.z

        self.data.append([elapsed_time, force_x, force_y, force_z, torque_x, torque_y, torque_z])
        self.get_logger().info(f"Recorded force: {force_x}, {force_y}, {force_z}")

    def save_data(self, filename="force_data.csv"):
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"])
            writer.writerows(self.data)
        self.get_logger().info(f"Data saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ForceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
        node.get_logger().info("Shutting down, data saved.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
