#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import time
import os
import threading
import sys

def get_next_filename(base_name="Hammer_links", extension=".csv"):
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_logs")
    os.makedirs(log_dir, exist_ok=True)  # Stelle sicher, dass der Ordner existiert
    
    i = 1
    while os.path.exists(os.path.join(log_dir, f"{base_name}{i}{extension}")):
        i += 1
    return os.path.join(log_dir, f"{base_name}{i}{extension}")

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
        self.recording = False
        self.start_time = time.time()
        self.running = True  # Kontrollvariable für Input-Thread
        self.get_logger().info("Force Logger Node started. Press 's' to start/stop recording.")
        
        # Starte einen Thread für Tasteneingaben
        self.input_thread = threading.Thread(target=self.listen_for_input, daemon=True)
        self.input_thread.start()

    def listen_for_input(self):
        while self.running:
            key = sys.stdin.read(1)  # Wartet auf eine einzelne Taste
            if key.lower() == 's':
                self.toggle_recording()

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            self.start_time = time.time()
            self.data = []
            self.get_logger().info("Recording started.")
        else:
            self.get_logger().info("Recording stopped. Saving data...")
            self.save_data()

    def wrench_callback(self, msg):
        if self.recording:
            elapsed_time = time.time() - self.start_time
            force_x = msg.wrench.force.x
            force_y = msg.wrench.force.y
            force_z = msg.wrench.force.z
            torque_x = msg.wrench.torque.x
            torque_y = msg.wrench.torque.y
            torque_z = msg.wrench.torque.z

            self.data.append([elapsed_time, force_x, force_y, force_z, torque_x, torque_y, torque_z])
            self.get_logger().info(f"Recording force: {force_x}, {force_y}, {force_z}")

    def save_data(self):
        if not self.data:
            self.get_logger().info("No data recorded, skipping file save.")
            return
        filename = get_next_filename()
        self.get_logger().info(f"Saving to {filename}")
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"])
            writer.writerows(self.data)
        self.get_logger().info(f"Data saved to {filename}")

    def shutdown(self):
        self.running = False  # Beende Input-Thread
        self.get_logger().info("Shutting down.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ForceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
