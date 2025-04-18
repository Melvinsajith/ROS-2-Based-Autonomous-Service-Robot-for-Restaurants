import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSlider
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer, QTime

# 🗺️ Define named goal positions
named_locations = {
    "Table 1": [-1.31, -2.78, 0.0],
    "Table 2": [-2.5, -3.0, 0.0],
    "Table 3": [-3.5, -3.2, 0.0],
    "Table 4": [-4.5, -3.4, 0.0],
    "Table 5": [-1.31, -4.5, 0.0],
    "Table 6": [-2.5, -4.7, 0.0],
    "Table 7": [-3.5, -4.9, 0.0],
    "Table 8": [-4.5, -5.1, 0.0],
    "Kitchen": [-8.23, -5.81, 1.57],
}

# 🔧 ROS2 Navigation Client Node
class NavGUI(Node):
    def __init__(self):
        super().__init__('gui_nav_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_goal(self, location_name, status_callback):
        if location_name not in named_locations:
            status_callback("❌ Unknown location")
            return

        x, y, yaw = named_locations[location_name]
        q = quaternion_from_euler(0, 0, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda f: self.goal_response_callback(f, status_callback))

    def goal_response_callback(self, future, status_callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            status_callback("❌ Goal Rejected")
            return

        self.current_goal_handle = goal_handle
        status_callback("🚀 Navigating...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda f: self.get_result_callback(f, status_callback))

    def get_result_callback(self, future, status_callback):
        result = future.result().result
        status_callback("✅ Arrived")
        self.current_goal_handle = None

    def cancel_goal(self, status_callback):
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: status_callback("❌ Cancelled"))
        else:
            status_callback("⚠️ No active goal")

    def send_velocity(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)


# 🎨 GUI Layout & Styling
class NavApp(QWidget):
    def __init__(self, rcl_node):
        super().__init__()
        self.setWindowTitle("🤖 Restaurant Navigation Console")
        self.setStyleSheet(self.dark_style())
        self.setFont(QFont("Segoe UI", 10))
        self.rcl_node = rcl_node
        self.linear_speed = 0.3
        self.angular_speed = 0.8

        self.status_label = QLabel("Status: 🟡 Idle")
        self.status_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFixedHeight(180)

        self.cancel_button = QPushButton("❌ Cancel Goal")
        self.cancel_button.clicked.connect(self.cancel_goal)
        self.cancel_button.setStyleSheet("background-color: #b71c1c; color: white;")

        # Location grid
        grid = QGridLayout()
        for i, name in enumerate(named_locations):
            btn = QPushButton(name)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda _, n=name: self.send_goal(n))
            grid.addWidget(btn, i // 3, i % 3)

        # --- Manual Control Section ---
        btn_forward = QPushButton("⬆️")
        btn_backward = QPushButton("⬇️")
        btn_left = QPushButton("⬅️")
        btn_right = QPushButton("➡️")
        btn_stop = QPushButton("⏹️")

        for btn in [btn_forward, btn_backward, btn_left, btn_right, btn_stop]:
            btn.setFixedSize(60, 60)
            btn.setFont(QFont("Segoe UI", 12, QFont.Bold))
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2e7d32;
                    border: 2px solid #1b5e20;
                    border-radius: 12px;
                    color: white;
                }
                QPushButton:hover {
                    background-color: #388e3c;
                }
                QPushButton:pressed {
                    background-color: #1b5e20;
                }
            """)

        btn_forward.clicked.connect(lambda: self.rcl_node.send_velocity(self.linear_speed, 0.0))
        btn_backward.clicked.connect(lambda: self.rcl_node.send_velocity(-self.linear_speed, 0.0))
        btn_left.clicked.connect(lambda: self.rcl_node.send_velocity(0.0, self.angular_speed))
        btn_right.clicked.connect(lambda: self.rcl_node.send_velocity(0.0, -self.angular_speed))
        btn_stop.clicked.connect(lambda: self.rcl_node.send_velocity(0.0, 0.0))

        manual_grid = QGridLayout()
        manual_grid.addWidget(btn_forward, 0, 1)
        manual_grid.addWidget(btn_left, 1, 0)
        manual_grid.addWidget(btn_stop, 1, 1)
        manual_grid.addWidget(btn_right, 1, 2)
        manual_grid.addWidget(btn_backward, 2, 1)

        # Sliders
        self.linear_slider = QSlider(Qt.Vertical)
        self.linear_slider.setRange(0, 100)
        self.linear_slider.setValue(int(self.linear_speed * 100))
        self.linear_slider.valueChanged.connect(self.update_linear_speed)

        self.angular_slider = QSlider(Qt.Vertical)
        self.angular_slider.setRange(0, 200)
        self.angular_slider.setValue(int((self.angular_speed / 2.0) * 100))
        self.angular_slider.valueChanged.connect(self.update_angular_speed)

        self.linear_label = QLabel(f"Linear: {self.linear_speed:.2f} m/s")
        self.angular_label = QLabel(f"Angular: {self.angular_speed:.2f} rad/s")

        reset_btn = QPushButton("🔄 Reset Speed")
        reset_btn.clicked.connect(self.reset_speeds)
        reset_btn.setStyleSheet("background-color: #455a64; color: white;")

        slider_box = QVBoxLayout()
        slider_box.addWidget(QLabel("🔼 Linear Speed"))
        slider_box.addWidget(self.linear_slider)
        slider_box.addWidget(self.linear_label)
        slider_box.addSpacing(10)
        slider_box.addWidget(QLabel("🔁 Angular Speed"))
        slider_box.addWidget(self.angular_slider)
        slider_box.addWidget(self.angular_label)
        slider_box.addWidget(reset_btn)

        controls_with_sliders = QHBoxLayout()
        controls_with_sliders.addLayout(manual_grid)
        controls_with_sliders.addSpacing(20)
        controls_with_sliders.addLayout(slider_box)

        # Main layout
        layout = QVBoxLayout()
        layout.addLayout(grid)
        layout.addWidget(self.status_label)
        layout.addWidget(self.cancel_button)
        layout.addWidget(QLabel("📜 Logs:"))
        layout.addWidget(self.log_box)
        layout.addWidget(QLabel("🎮 Manual Controls & Speed:"))
        layout.addLayout(controls_with_sliders)
        self.setLayout(layout)
        self.resize(750, 750)

        # ROS2 spin timer
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.rcl_node, timeout_sec=0.1))
        self.timer.start(100)

    def update_linear_speed(self, value):
        self.linear_speed = value / 100.0
        self.linear_label.setText(f"Linear: {self.linear_speed:.2f} m/s")

    def update_angular_speed(self, value):
        self.angular_speed = (value / 100.0) * 2.0
        self.angular_label.setText(f"Angular: {self.angular_speed:.2f} rad/s")

    def reset_speeds(self):
        self.linear_slider.setValue(30)
        self.angular_slider.setValue(40)

    def send_goal(self, location):
        self.update_status(f"Sending goal to {location}...", icon="🟡")
        self.log(f"Goal sent → {location}")
        self.rcl_node.send_goal(location, self.update_status)

    def cancel_goal(self):
        self.rcl_node.cancel_goal(self.update_status)

    def update_status(self, message, icon=None):
        if icon is None:
            if "Arrived" in message:
                icon = "✅"
            elif "Navigating" in message:
                icon = "🚀"
            elif "Cancelled" in message or "Rejected" in message:
                icon = "❌"
            elif "Idle" in message:
                icon = "🟡"
            else:
                icon = "⚠️"
        self.status_label.setText(f"Status: {icon} {message}")
        self.log(message)

    def log(self, message):
        timestamp = QTime.currentTime().toString("HH:mm:ss")
        self.log_box.append(f"[{timestamp}] {message}")

    def dark_style(self):
        return """
        QWidget {
            background-color: #121212;
            color: #e0e0e0;
        }
        QPushButton {
            background-color: #1e88e5;
            border: none;
            padding: 10px;
            color: white;
            font-weight: bold;
            border-radius: 6px;
        }
        QPushButton:hover {
            background-color: #1565c0;
        }
        QPushButton:pressed {
            background-color: #0d47a1;
        }
        QLabel {
            margin: 4px;
        }
        QTextEdit {
            background-color: #1e1e1e;
            color: #dcdcdc;
            border: 1px solid #333;
            font-family: Consolas, monospace;
        }
        """

# 🚀 Main Launcher
def main():
    rclpy.init()
    node = NavGUI()
    app = QApplication(sys.argv)
    gui = NavApp(node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
