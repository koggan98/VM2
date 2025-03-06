import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
import numpy as np

class LiveForceAndTorquePlot(Node):
    def __init__(self):
        super().__init__('live_force_torque_plot')

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10
        )

        self.history_length = 100  
        self.time_data = np.linspace(-self.history_length, 0, self.history_length)

        self.force_x = np.zeros(self.history_length)
        self.force_y = np.zeros(self.history_length)
        self.force_z = np.zeros(self.history_length)
        self.torque_x = np.zeros(self.history_length)
        self.torque_y = np.zeros(self.history_length)
        self.torque_z = np.zeros(self.history_length)

        self.fig_force, self.ax_force = plt.subplots()
        self.fig_torque, self.ax_torque = plt.subplots()

        self.ax_force.set_title("Live Force Data")
        self.ax_force.set_xlabel("Time")
        self.ax_force.set_ylabel("Force (N)")
        self.ax_force.set_xlim(-self.history_length, 0)
        
        self.line_force_x, = self.ax_force.plot(self.time_data, self.force_x, label="Force X", color='r')
        self.line_force_y, = self.ax_force.plot(self.time_data, self.force_y, label="Force Y", color='g')
        self.line_force_z, = self.ax_force.plot(self.time_data, self.force_z, label="Force Z", color='b')
        self.ax_force.legend()

        self.ax_torque.set_title("Live Torque Data")
        self.ax_torque.set_xlabel("Time")
        self.ax_torque.set_ylabel("Torque (Nm)")
        self.ax_torque.set_xlim(-self.history_length, 0)
        self.ax_torque.set_ylim(-1, 1)

        self.line_torque_x, = self.ax_torque.plot(self.time_data, self.torque_x, label="Torque X", color='r')
        self.line_torque_y, = self.ax_torque.plot(self.time_data, self.torque_y, label="Torque Y", color='g')
        self.line_torque_z, = self.ax_torque.plot(self.time_data, self.torque_z, label="Torque Z", color='b')
        self.ax_torque.legend()

        plt.ion()
        plt.show()

        self.create_timer(0.05, self.update_plot)

    def wrench_callback(self, msg):
        self.force_x = np.roll(self.force_x, -1)
        self.force_y = np.roll(self.force_y, -1)
        self.force_z = np.roll(self.force_z, -1)
        self.torque_x = np.roll(self.torque_x, -1)
        self.torque_y = np.roll(self.torque_y, -1)
        self.torque_z = np.roll(self.torque_z, -1)

        self.force_x[-1] = msg.wrench.force.x
        self.force_y[-1] = msg.wrench.force.y
        self.force_z[-1] = msg.wrench.force.z
        self.torque_x[-1] = msg.wrench.torque.x
        self.torque_y[-1] = msg.wrench.torque.y
        self.torque_z[-1] = msg.wrench.torque.z

    def update_plot(self):
        self.line_force_x.set_ydata(self.force_x)
        self.line_force_y.set_ydata(self.force_y)
        self.line_force_z.set_ydata(self.force_z)

        self.line_torque_x.set_ydata(self.torque_x)
        self.line_torque_y.set_ydata(self.torque_y)
        self.line_torque_z.set_ydata(self.torque_z)

        force_min = min(self.force_x.min(), self.force_y.min(), self.force_z.min())
        force_max = max(self.force_x.max(), self.force_y.max(), self.force_z.max())

        self.ax_force.set_ylim(force_min - abs(force_min) * 0.5 - 5,
                               force_max + abs(force_max) * 0.5 + 5)

        self.ax_torque.set_ylim(min(self.torque_x.min(), self.torque_y.min(), self.torque_z.min()) - 0.1,
                                max(self.torque_x.max(), self.torque_y.max(), self.torque_z.max()) + 0.1)

        self.fig_force.canvas.draw()
        self.fig_torque.canvas.draw()

        self.fig_force.canvas.flush_events()
        self.fig_torque.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LiveForceAndTorquePlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Live Plot.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
