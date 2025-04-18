import sys
import subprocess
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QTextEdit,
    QVBoxLayout, QMessageBox
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QTime, QThread, pyqtSignal


class CommandRunner(QThread):
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(list)

    def __init__(self, commands_with_delay):
        super().__init__()
        self.commands_with_delay = commands_with_delay
        self.processes = []

    def run(self):
        for cmd, delay in self.commands_with_delay:
            self.log_signal.emit(f"▶️ Running: {' '.join(cmd)}")
            try:
                p = subprocess.Popen(cmd)
                self.processes.append(p)
            except Exception as e:
                self.log_signal.emit(f"❌ Error: {e}")
            time.sleep(delay)
        self.finished_signal.emit(self.processes)


class LauncherGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 Restaurant Robot Launcher")
        self.setStyleSheet(self.dark_style())
        self.setFont(QFont("Segoe UI", 10))
        self.processes = []

        # Buttons
        self.start_button = QPushButton("🚀 Start Full System")
        self.stop_button = QPushButton("🛑 Stop System")
        self.stop_button.setEnabled(False)

        self.start_button.clicked.connect(self.launch_all)
        self.stop_button.clicked.connect(self.stop_all)

        self.start_button.setFixedHeight(50)
        self.stop_button.setFixedHeight(50)
        self.stop_button.setStyleSheet("background-color: #b71c1c; color: white;")

        # Logs
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFixedHeight(300)

        layout = QVBoxLayout()
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(QLabel("📜 Launch Log:"))
        layout.addWidget(self.log_box)
        self.setLayout(layout)
        self.resize(600, 450)

    def launch_all(self):
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.log("🚦 Starting all systems...")

        commands = [
            (["ros2", "launch", "restaurant_robot", "launch_sim.launch.py",
              "world:=/home/melvin/new_world_rest.world"], 5),
            (["ros2", "launch", "slam_toolbox", "online_async_launch.py",
              "slam_params_file:=/home/melvin/mini_pro_ws/src/restaurant_robot/config/mapper_params_online_async.yaml",
              "use_sim_time:=true"], 5),
            (["ros2", "launch", "nav2_bringup", "navigation_launch.py",
              "use_sim_time:=true"], 5),
            (["rviz2", "-d", "/home/melvin/mini_pro_ws/src/restaurant_robot/config/new_main.rviz"], 2),
            (["python3", "/home/melvin/mini_pro_ws/install/nav_locations_restaurant/lib/nav_locations_restaurant/gui_nav_node"], 1),
        ]

        self.runner = CommandRunner(commands)
        self.runner.log_signal.connect(self.log)
        self.runner.finished_signal.connect(self.store_processes)
        self.runner.start()

    def store_processes(self, processes):
        self.processes = processes
        self.log("✅ All systems launched.")

    def stop_all(self):
        if not self.processes:
            QMessageBox.warning(self, "⚠️ Warning", "No processes to stop.")
            return

        self.log("🛑 Stopping all systems...")
        for p in self.processes:
            try:
                p.terminate()
                self.log(f"⛔ Terminated PID {p.pid}")
            except Exception as e:
                self.log(f"⚠️ Error killing process: {e}")

        self.processes = []
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        self.log("🟡 All processes stopped.")

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
            background-color: #00acc1;
            border: none;
            padding: 12px;
            color: white;
            font-weight: bold;
            border-radius: 6px;
        }
        QPushButton:hover {
            background-color: #00838f;
        }
        QPushButton:pressed {
            background-color: #006064;
        }
        QTextEdit {
            background-color: #1e1e1e;
            color: #dcdcdc;
            border: 1px solid #444;
            font-family: Consolas, monospace;
        }
        QLabel {
            margin: 4px;
        }
        """
def main():
    app = QApplication(sys.argv)
    launcher = LauncherGUI()
    launcher.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
