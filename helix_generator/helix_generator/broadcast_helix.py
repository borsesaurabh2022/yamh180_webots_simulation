#!/usr/bin/python3
# Licensed under MIT. See LICENSE file. Copyright Saurabh Borse.
import math
import sys
from threading import Thread

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from rclpy.node import Node
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class HelixPublisher(Node):
    def __init__(self):
        super().__init__("helix_publisher")
        self.declare_parameter("factor", 1.0)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.helix_pub = self.create_publisher(Float64MultiArray, "helix", QoSProfile(depth=10))
        self.helix_points = []
        self.directional_vector_array = None
        self.is_broadcasting = False

    def generate_and_publish(self, radius, pitch, samples):
        factor = self.get_parameter("factor").get_parameter_value().double_value
        self.helix_points = self.generate_helix_points(radius, pitch, factor, samples)
        # self.get_logger().info(f"Helix points: {self.helix_points}")

        self.directional_vector_array = np.array(
            [self.helix_directional_vector(t, pitch, radius) for t in np.linspace(0, pitch * factor, samples)]
        )
        # self.get_logger().info(f"Helix directional vector: {self.directional_vector_array}")
        self.get_logger().info(
            f"Publishinh helix with parameters : radius={radius}, pitch={pitch}, no.of samplepoints={samples}"
        )
        self.publish_helix_points()

    def generate_helix_points(self, radius, pitch, factor, samples):
        ts = np.linspace(0, pitch * factor, samples)
        helix_points = [
            [
                radius * np.cos(2 * np.pi * t / pitch),
                radius * np.sin(2 * np.pi * t / pitch),
                t,
            ]
            for t in ts
        ] - np.array([radius, 0, 0])
        return helix_points

    def helix_directional_vector(self, t, pitch, radius):
        dx_dt = -2 * np.pi * radius * np.sin(2 * np.pi * t / pitch) / pitch
        dy_dt = 2 * np.pi * radius * np.cos(2 * np.pi * t / pitch) / pitch
        dz_dt = 1
        return np.array([dx_dt, dy_dt, dz_dt])

    def publish_helix_points(self):
        msg = Float64MultiArray()
        msg.data = [point for sublist in self.helix_points for point in sublist]
        self.helix_pub.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "mh180/base_link"
        transform.child_frame_id = "helix_reference"

        transform.transform.translation.x = 2.25
        transform.transform.translation.y = 1.0
        transform.transform.translation.z = 1.0
        transform.transform.rotation.x = 0.7071068
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.7071068

        self.tf_broadcaster.sendTransform(transform)

        for i, point in enumerate(self.helix_points):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "helix_reference"
            transform.child_frame_id = f"helix_point_{i}"

            transform.transform.translation.x = point[0]
            transform.transform.translation.y = point[1]
            transform.transform.translation.z = point[2]

            r = R.from_euler(
                "xyz",
                [
                    self.directional_vector_array[i][0] + (math.pi / 2),
                    self.directional_vector_array[i][1],
                    self.directional_vector_array[i][2],
                ],
            )
            transform.transform.rotation.x = r.as_quat()[0]
            transform.transform.rotation.y = r.as_quat()[1]
            transform.transform.rotation.z = r.as_quat()[2]
            transform.transform.rotation.w = r.as_quat()[3]

            self.tf_broadcaster.sendTransform(transform)

    def start_broadcasting(self):
        self.is_broadcasting = True

    def stop_broadcasting(self):
        self.is_broadcasting = False


class HelixGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        self.radius_label = QLabel("Radius:")
        self.radius_input = QLineEdit()
        layout.addWidget(self.radius_label)
        layout.addWidget(self.radius_input)

        self.pitch_label = QLabel("Pitch:")
        self.pitch_input = QLineEdit()
        layout.addWidget(self.pitch_label)
        layout.addWidget(self.pitch_input)

        self.samples_label = QLabel("Samples:")
        self.samples_input = QLineEdit()
        layout.addWidget(self.samples_label)
        layout.addWidget(self.samples_input)

        self.parameter_display = QLabel("Parameters will be shown here.")
        layout.addWidget(self.parameter_display)

        self.generate_button = QPushButton("Generate Helix")
        self.generate_button.clicked.connect(self.on_generate)
        layout.addWidget(self.generate_button)

        self.stop_button = QPushButton("Stop Broadcast")
        self.stop_button.clicked.connect(self.on_stop)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)
        self.setWindowTitle("Helix Generator")

        # Timer for continuous publishing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(100)  # 100 ms interval for publishing

    def on_generate(self):
        try:
            radius = float(self.radius_input.text())
            pitch = float(self.pitch_input.text())
            samples = int(self.samples_input.text())
            self.node.generate_and_publish(radius, pitch, samples)
            self.node.start_broadcasting()  # Start broadcasting after generation
            # Update the displayed parameters
            self.parameter_display.setText(f"Radius: {radius}, Pitch: {pitch}, Samples: {samples}")
        except ValueError:
            print("Invalid input! Please enter numeric values for radius, pitch, and samples.")

    def on_stop(self):
        self.node.stop_broadcasting()

    def on_timer(self):
        if self.node.is_broadcasting:
            self.node.publish_helix_points()


def run_ros_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    helix_publisher = HelixPublisher()

    # Start ROS 2 spin in a separate thread
    ros_thread = Thread(target=run_ros_node, args=(helix_publisher,))
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = HelixGUI(helix_publisher)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        helix_publisher.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == "__main__":
    main()
