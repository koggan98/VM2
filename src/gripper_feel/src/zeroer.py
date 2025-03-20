import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty

class ForceTorqueOffsetNode(Node):
    def __init__(self):
        super().__init__('force_torque_offset_node')

        # Subscriber für die originalen Wrench-Daten
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10
        )

        # Publisher für die korrigierten Wrench-Daten
        self.publisher = self.create_publisher(WrenchStamped, '/wrench_zeroed', 10)

        # Service zum Nullsetzen der Kraftwerte
        self.srv = self.create_service(Empty, 'reset_wrench_offset', self.reset_wrench_callback)

        # Offset für Kraft und Drehmoment (standardmäßig auf None)
        self.force_offset = None
        self.torque_offset = None

        self.get_logger().info("🔹 Force-Torque Offset Node gestartet. Service: /reset_wrench_offset")

    def wrench_callback(self, msg):
        if self.force_offset is None:
            # Falls kein Offset gesetzt wurde, speichere den ersten Wert als Nullpunkt
            self.force_offset = msg.wrench.force
            self.torque_offset = msg.wrench.torque
            self.get_logger().info("🔹 Offset gespeichert: Setze aktuelle Kraft- und Drehmomentwerte als Null.")

        # Korrigierte Werte berechnen
        corrected_msg = WrenchStamped()
        corrected_msg.header = msg.header
        corrected_msg.wrench.force.x = msg.wrench.force.x - self.force_offset.x
        corrected_msg.wrench.force.y = msg.wrench.force.y - self.force_offset.y
        corrected_msg.wrench.force.z = msg.wrench.force.z - self.force_offset.z
        corrected_msg.wrench.torque.x = msg.wrench.torque.x - self.torque_offset.x
        corrected_msg.wrench.torque.y = msg.wrench.torque.y - self.torque_offset.y
        corrected_msg.wrench.torque.z = msg.wrench.torque.z - self.torque_offset.z

        # Publiziere die "genullten" Werte auf einem neuen Topic
        self.publisher.publish(corrected_msg)

    def reset_wrench_callback(self, request, response):
        # Setze Offset zurück, damit die nächste eingehende Kraft als neue "0" erkannt wird
        self.force_offset = None
        self.torque_offset = None
        self.get_logger().info("🔄 Kraft- und Drehmoment-Offsets wurden zurückgesetzt!")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueOffsetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
