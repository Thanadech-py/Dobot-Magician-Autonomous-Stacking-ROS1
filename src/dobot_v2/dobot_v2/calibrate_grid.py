#!/usr/bin/env python3
"""
Interactive Field Grid Calibration Tool for Dobot Vision (ROS 1).

Allows clicking the 4 corners of the 3x3 Grid on the camera stream to achieve
exact pixel alignment. Automatically calculates Homography, previews the 3x3
grid lines and 9 cell boxes in real-time, and saves the corner coordinates into detection_node.yaml.
Also publishes directly to /set_grid_corners if detection_node is running.

Supports both:
  1. ROS 1 subscription (when dobot_vision.launch / usb_cam is running)
  2. Standalone OpenCV VideoCapture (when no ROS node is using the camera)

Usage:
  rosrun dobot_v2 calibrate_grid.py
Or:
  python3 src/dobot_v2/dobot_v2/calibrate_grid.py
"""

import os
import sys
import cv2
import json
import time
import numpy as np

try:
    import rospy
    import rospkg
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    HAS_ROS1 = True
except ImportError:
    HAS_ROS1 = False


class GridCalibrator:
    def __init__(self, yaml_path=None):
        self.points = []
        self.current_frame = None
        self.yaml_path = yaml_path
        self.running = True
        self.grid_size_mm = 114.5
        self.cell_size_mm = 25.0
        self.cell_pitch_mm = 35.0
        self.ros_publisher = None

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < 4:
                self.points.append([float(x), float(y)])
                names = ["Top-Left (TL)", "Top-Right (TR)", "Bottom-Right (BR)", "Bottom-Left (BL)"]
                print(f"[{len(self.points)}/4] Recorded {names[len(self.points)-1]}: ({x:.1f}, {y:.1f})")

    def save_to_yaml(self, corners):
        flat_corners = [round(float(c), 1) for pt in corners for c in pt]
        print("\n" + "="*60)
        print("EXACT CORNERS FOR detection_node.yaml:")
        print(f"grid_corners: {flat_corners}")
        print("="*60 + "\n")

        # Save to YAML files
        yaml_candidates = [
            os.path.expanduser("~/catkin_ws/src/dobot_v2/config/detection_node.yaml"),
            os.path.expanduser("~/dobot_ws/src/dobot_v2/config/detection_node.yaml"),
            os.path.join(os.path.dirname(__file__), "..", "config", "detection_node.yaml"),
        ]
        if HAS_ROS1:
            try:
                rp = rospkg.RosPack()
                yaml_candidates.insert(0, os.path.join(rp.get_path("dobot_v2"), "config", "detection_node.yaml"))
            except Exception:
                pass

        if self.yaml_path and self.yaml_path not in yaml_candidates:
            yaml_candidates.insert(0, self.yaml_path)

        for path in yaml_candidates:
            real_path = os.path.realpath(path)
            if os.path.exists(real_path):
                try:
                    with open(real_path, 'r', encoding='utf-8') as f:
                        content = f.read()

                    import re
                    new_line = f"    grid_corners: {flat_corners}"
                    updated = re.sub(r"^\s*grid_corners:\s*\[.*?\]", new_line, content, flags=re.MULTILINE)

                    with open(real_path, 'w', encoding='utf-8') as f:
                        f.write(updated)
                    print(f"SUCCESS: Saved grid_corners to: {real_path}")
                except Exception as e:
                    print(f"Error saving to {path}: {e}")

        # Publish live to detection_node if publisher is available
        if self.ros_publisher:
            try:
                msg = String()
                msg.data = json.dumps(corners)
                self.ros_publisher.publish(msg)
                print("SUCCESS: Published new corners to /set_grid_corners (detection_node updated live!)")
            except Exception as e:
                print(f"Could not publish corners: {e}")

    def draw_grid_preview(self, vis):
        n_pts = len(self.points)

        for i, pt in enumerate(self.points):
            cv2.circle(vis, (int(pt[0]), int(pt[1])), 6, (0, 0, 255), -1)
            cv2.putText(vis, f"C{i+1}", (int(pt[0])+10, int(pt[1])-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if n_pts >= 2:
            for i in range(n_pts - 1):
                pt1 = (int(self.points[i][0]), int(self.points[i][1]))
                pt2 = (int(self.points[i+1][0]), int(self.points[i+1][1]))
                cv2.line(vis, pt1, pt2, (0, 255, 255), 2)
        if n_pts == 4:
            pt4 = (int(self.points[3][0]), int(self.points[3][1]))
            pt1 = (int(self.points[0][0]), int(self.points[0][1]))
            cv2.line(vis, pt4, pt1, (0, 255, 255), 2)

            src = np.array(self.points, dtype=np.float32)
            S = self.grid_size_mm
            dst = np.array([[0, 0], [S, 0], [S, S], [0, S]], dtype=np.float32)
            H, _ = cv2.findHomography(dst, src)

            if H is not None:
                # Outer perimeter
                pts_perimeter = np.array([[[0, 0]], [[S, 0]], [[S, S]], [[0, S]]], dtype=np.float32)
                proj_perim = cv2.perspectiveTransform(pts_perimeter, H).reshape(-1, 2).astype(np.int32)
                cv2.polylines(vis, [proj_perim], True, (0, 255, 0), 2)

                # Draw 9 individual cells
                cell_s = self.cell_size_mm
                pitch = self.cell_pitch_mm
                for row in range(3):
                    for col in range(3):
                        cx = pitch * col + pitch / 2.0
                        cy = pitch * row + pitch / 2.0
                        half = cell_s / 2.0
                        box_mm = np.array([
                            [[cx - half, cy - half]],
                            [[cx + half, cy - half]],
                            [[cx + half, cy + half]],
                            [[cx - half, cy + half]]
                        ], dtype=np.float32)
                        proj_box = cv2.perspectiveTransform(box_mm, H).reshape(-1, 2).astype(np.int32)
                        color = (0, 255, 255) if (row == 1 and col == 1) else (0, 255, 0)
                        cv2.polylines(vis, [proj_box], True, color, 1)

        # Status text overlay
        status = f"Corners Clicked: {n_pts}/4"
        if n_pts == 4:
            status += " - Press [s] to SAVE corners, [r] to RESET"
        else:
            names = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
            status += f" -> Click {names[n_pts]}"

        cv2.rectangle(vis, (10, 10), (vis.shape[1]-10, 45), (0, 0, 0), -1)
        cv2.putText(vis, status, (15, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)


def run_standalone(video_device="/dev/video0"):
    print(f"Opening camera at {video_device} via OpenCV...")
    dev_id = 0
    if video_device.startswith("/dev/video"):
        try:
            dev_id = int(video_device.replace("/dev/video", ""))
        except ValueError:
            dev_id = 0

    cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera device {video_device}")
        return

    calibrator = GridCalibrator()
    window_name = "Field Grid Corner Calibration [OpenCV Standalone]"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, calibrator.mouse_callback)

    _print_instructions()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera.")
            break

        vis = frame.copy()
        calibrator.draw_grid_preview(vis)
        cv2.imshow(window_name, vis)

        key = cv2.waitKey(20) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('r'):
            calibrator.points = []
            print("Points reset. Click the 4 corners again.")
        elif key == ord('s') and len(calibrator.points) == 4:
            calibrator.save_to_yaml(calibrator.points)
            print("To apply in ROS 1, restart dobot_vision or check YAML.")
            break

    cap.release()
    cv2.destroyAllWindows()


def run_ros_node():
    """ROS 1 Node mode: subscribes to camera topics and publishes to /set_grid_corners."""
    rospy.init_node('calibrate_grid_node', anonymous=True)
    calibrator = GridCalibrator()
    bridge = CvBridge()
    window_name = "Field Grid Corner Calibration [ROS 1]"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, calibrator.mouse_callback)

    # Publisher to notify running detection_node immediately
    pub_corners = rospy.Publisher('set_grid_corners', String, queue_size=10)
    calibrator.ros_publisher = pub_corners

    frame_holder = [None]

    def compressed_cb(msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame_holder[0] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            pass

    def raw_cb(msg: Image):
        try:
            frame_holder[0] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, compressed_cb, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/usb_cam/image_raw', Image, raw_cb, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/image_raw/compressed', CompressedImage, compressed_cb, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/image_raw', Image, raw_cb, queue_size=1, buff_size=2**24)

    _print_instructions()
    print("Waiting for camera frames on /usb_cam/image_raw or /image_raw...")

    try:
        while not rospy.is_shutdown() and calibrator.running:
            if frame_holder[0] is not None:
                vis = frame_holder[0].copy()
                calibrator.draw_grid_preview(vis)
                cv2.imshow(window_name, vis)

            key = cv2.waitKey(20) & 0xFF
            if key in [ord('q'), 27]:
                break
            elif key == ord('r'):
                calibrator.points = []
                print("Points reset. Click the 4 corners again.")
            elif key == ord('s') and len(calibrator.points) == 4:
                calibrator.save_to_yaml(calibrator.points)
                break
    finally:
        cv2.destroyAllWindows()


def _print_instructions():
    print("\n" + "#"*60)
    print("  DOBOT V2 FIELD GRID CALIBRATION TOOL (ROS 1)")
    print("#"*60)
    print("Instructions:")
    print("  1. Position your camera pointing down at the Field Template.")
    print("  2. Click the 4 OUTER corners of the 3x3 grey grid in order:")
    print("       Corner 1: Top-Left (TL)")
    print("       Corner 2: Top-Right (TR)")
    print("       Corner 3: Bottom-Right (BR)")
    print("       Corner 4: Bottom-Left (BL)")
    print("  3. Verify the 9 green/yellow cell boxes align with the paper.")
    print("  4. Keys:")
    print("       [s]: Save corners to YAML and update detection_node")
    print("       [r]: Reset points to re-click")
    print("       [q] or [ESC]: Quit")
    print("#"*60 + "\n")


def main():
    if HAS_ROS1:
        try:
            run_ros_node()
            return
        except Exception as e:
            print(f"ROS 1 mode error: {e}. Falling back to standalone OpenCV.")
    run_standalone()


if __name__ == '__main__':
    main()
