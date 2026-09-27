"""ROS 1 Node and PyQt Thread bridge."""

import json
import os
import shutil
import subprocess
import sys
import time
import cv2
import numpy as np

try:
    from PyQt6.QtCore import QThread, pyqtSignal
    from PyQt6.QtGui import QImage
except ImportError:
    from PyQt5.QtCore import QThread, pyqtSignal
    from PyQt5.QtGui import QImage

try:
    import rospy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    HAS_ROS1 = True
except ImportError:
    HAS_ROS1 = False

try:
    from .config import TOPICS, DETECTION
except (ImportError, ValueError):
    from Dobot_UI.config import TOPICS, DETECTION


class RosBridge(QThread):
    """Bridges ROS 1 subscriptions and publishers with Qt signals."""
    image_received      = pyqtSignal(QImage, float)
    status_received     = pyqtSignal(dict)
    detections_received = pyqtSignal(dict)
    log_message         = pyqtSignal(str, str)

    def __init__(self, mock_mode=False, parent=None):
        super().__init__(parent)
        self.mock_mode = mock_mode or not HAS_ROS1
        self._running = True
        self.pub_cmd = None
        self._fps_count = 0
        self._last_time = time.time()
        self.fps = 0.0
        self._mock_telemetry = {
            "state": "IDLE",
            "message": "Ready (Simulation)",
            "x": 200.0,
            "y": 0.0,
            "z": 80.0,
            "r": 0.0,
            "suction": False,
        }

    def run(self):
        if not self.mock_mode and HAS_ROS1:
            try:
                if not rospy.core.is_initialized():
                    rospy.init_node("dobot_ui_node", anonymous=True, disable_signals=True)

                mock_param = rospy.get_param("~mock_mode", False)
                if mock_param:
                    self.mock_mode = True
                    self.log_message.emit("WARN", "Running in Simulation / Mock Mode (via ROS parameter).")
                    self._run_mock()
                    return

                comp_topic = rospy.get_param("~image_compressed_topic", TOPICS.get("image_comp", "detected_objects_image/compressed"))
                status_topic = rospy.get_param("~status_topic", TOPICS.get("status", "dobot_status"))
                det_topic = rospy.get_param("~detections_topic", TOPICS.get("detections", "detected_objects"))
                cmd_topic = rospy.get_param("~command_topic", TOPICS.get("command", "dobot_ui_cmd"))

                self.sub_comp = rospy.Subscriber(comp_topic, CompressedImage, self._on_comp_img, queue_size=1, buff_size=2**24)
                self.sub_status = rospy.Subscriber(status_topic, String, self._on_status, queue_size=10)
                self.sub_det = rospy.Subscriber(det_topic, String, self._on_detections, queue_size=10)
                self.pub_cmd = rospy.Publisher(cmd_topic, String, queue_size=10)
                self.log_message.emit("INFO", "Connected to ROS 1 topics.")

                rate = rospy.Rate(20)
                while self._running and not rospy.is_shutdown():
                    rate.sleep()
            except Exception as e:
                self.log_message.emit("ERROR", f"ROS 1 error: {e}")
        else:
            self.log_message.emit("WARN", "Running in Simulation / Mock Mode.")
            self._run_mock()

    def _on_comp_img(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                self._emit_qimage(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        except Exception:
            pass

    def _emit_qimage(self, rgb):
        self._fps_count += 1
        now = time.time()
        if now - self._last_time >= 1.0:
            self.fps = self._fps_count / (now - self._last_time)
            self._fps_count = 0
            self._last_time = now
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()
        self.image_received.emit(qimg, self.fps)

    def _on_status(self, msg):
        try:
            self.status_received.emit(json.loads(msg.data))
        except Exception:
            pass

    def _on_detections(self, msg):
        try:
            self.detections_received.emit(json.loads(msg.data))
        except Exception:
            pass

    def send_cmd(self, payload: dict):
        """Sends command dictionary as JSON string to the command topic."""
        cmd_str = json.dumps(payload)
        cmd = payload.get("cmd")
        if not self.mock_mode and self.pub_cmd and HAS_ROS1:
            msg = String()
            msg.data = cmd_str
            self.pub_cmd.publish(msg)
            self.log_message.emit("INFO", f"Sent command: {cmd}")
        else:
            self.log_message.emit("INFO", f"[MOCK] Sent command: {cmd}")
            # In simulation / mock mode, update simulated state so UI controls reflect changes
            if hasattr(self, "_mock_telemetry"):
                if cmd == "jog":
                    axis = payload.get("axis", "x")
                    step = float(payload.get("step", 10.0)) * int(payload.get("direction", 1))
                    self._mock_telemetry[axis] = round(self._mock_telemetry.get(axis, 0.0) + step, 1)
                    self._mock_telemetry["state"] = "IDLE"
                    self._mock_telemetry["message"] = f"Jogged {axis.upper()} to {self._mock_telemetry[axis]}"
                    self.status_received.emit(dict(self._mock_telemetry))
                elif cmd == "move_to":
                    for a in ["x", "y", "z", "r"]:
                        if a in payload:
                            self._mock_telemetry[a] = round(float(payload[a]), 1)
                    self._mock_telemetry["state"] = "IDLE"
                    self._mock_telemetry["message"] = "Target pose reached"
                    self.status_received.emit(dict(self._mock_telemetry))
                elif cmd == "suction":
                    self._mock_telemetry["suction"] = bool(payload.get("enable", False))
                    self.status_received.emit(dict(self._mock_telemetry))
                elif cmd == "preset":
                    pname = payload.get("name")
                    if pname == "home":
                        self._mock_telemetry.update({"x": 200.0, "y": 0.0, "z": 80.0, "r": 0.0, "message": "Homed"})
                    elif pname == "hover":
                        self._mock_telemetry.update({"x": 200.0, "y": 0.0, "z": 80.0, "message": "Hovering"})
                    elif pname == "dropoff":
                        self._mock_telemetry.update({"x": 200.0, "y": 0.0, "z": 30.0, "message": "At Drop-off"})
                    elif pname == "zero_r":
                        self._mock_telemetry.update({"r": 0.0, "message": "R rotation reset"})
                    self.status_received.emit(dict(self._mock_telemetry))

    def restart_detection_node(self):
        """Resets vision grid tracking cleanly via ROS topic without duplicate process conflicts."""
        self.log_message.emit("INFO", "Resetting vision grid tracking...")
        if HAS_ROS1 and not self.mock_mode:
            try:
                reset_topic = TOPICS.get("reset_grid", "reset_grid")
                pub = rospy.Publisher(reset_topic, String, queue_size=1)
                time.sleep(0.05)
                msg = String()
                msg.data = "reset"
                pub.publish(msg)
                self.log_message.emit("SUCCESS", "Published grid reset to /reset_grid.")
            except Exception as e:
                self.log_message.emit("WARN", f"Could not publish reset: {e}")

    def stop(self):
        self._running = False
        self.wait(500)

    def _run_mock(self):
        """Placeholder frame generator for headless/mock testing."""
        w, h = 640, 480
        if hasattr(self, "_mock_telemetry"):
            self.status_received.emit(dict(self._mock_telemetry))
        while self._running:
            frame = np.full((h, w, 3), 30, dtype=np.uint8)
            cv2.rectangle(frame, (170, 90), (470, 390), (160, 160, 160), 2)
            cv2.putText(frame, "SIMULATED VISION FEED", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (56, 189, 248), 2)
            self._emit_qimage(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            time.sleep(0.1)
