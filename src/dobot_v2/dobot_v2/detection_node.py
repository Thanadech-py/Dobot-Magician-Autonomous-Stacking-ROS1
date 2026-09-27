#!/usr/bin/env python3
"""
ROS 1 Node: Real-Time Field Grid Tracker & Cube Detector for Dobot (dobot_v2).

All configuration parameters and defaults are loaded directly from config/detection_node.yaml
with ROS parameter server overrides.

Subsystems:
  - dobot_v2.transforms:     Coordinate transformations & cell math
  - dobot_v2.grid_detector: Pallet contour detection, verification & locking
  - dobot_v2.cube_detector: Multi-color cube detection & geometric filtering
  - dobot_v2.visualizer:    Debug HUD and visual overlays
"""

import os
import time
import json
import yaml
import cv2
import numpy as np

try:
    import rospy
    import rospkg
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from geometry_msgs.msg import PoseArray, Pose
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    HAS_ROS1 = True
except ImportError:
    HAS_ROS1 = False
    Image = object
    CompressedImage = object
    CameraInfo = object
    PoseArray = object
    Pose = object
    String = object

from dobot_v2.transforms import FieldTransform
from dobot_v2.grid_detector import GridDetector
from dobot_v2.cube_detector import CubeDetector
from dobot_v2.visualizer import DetectionVisualizer


class Ros1Logger:
    """Logger wrapper conforming to standard node logger interface."""
    @staticmethod
    def info(msg: str):
        if HAS_ROS1:
            rospy.loginfo(f"[detection_node] {msg}")
        else:
            print(f"[INFO] [detection_node] {msg}")

    @staticmethod
    def warn(msg: str):
        if HAS_ROS1:
            rospy.logwarn(f"[detection_node] {msg}")
        else:
            print(f"[WARN] [detection_node] {msg}")

    warning = warn

    @staticmethod
    def error(msg: str):
        if HAS_ROS1:
            rospy.logerr(f"[detection_node] {msg}")
        else:
            print(f"[ERROR] [detection_node] {msg}")

    @staticmethod
    def debug(msg: str):
        if HAS_ROS1:
            rospy.logdebug(f"[detection_node] {msg}")
        else:
            print(f"[DEBUG] [detection_node] {msg}")


def load_yaml_config(logger=None) -> dict:
    """Loads default configuration directly from config/detection_node.yaml."""
    candidates = []
    if HAS_ROS1:
        try:
            rp = rospkg.RosPack()
            candidates.append(os.path.join(rp.get_path('dobot_v2'), 'config', 'detection_node.yaml'))
        except Exception:
            pass

    candidates.extend([
        os.path.expanduser('~/catkin_ws/src/dobot_v2/config/detection_node.yaml'),
        os.path.expanduser('~/dobot_ws/src/dobot_v2/config/detection_node.yaml'),
        os.path.join(os.path.dirname(__file__), '..', 'config', 'detection_node.yaml'),
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'dobot_v2', 'config', 'detection_node.yaml'),
    ])

    for path in candidates:
        if os.path.isfile(path):
            try:
                with open(os.path.realpath(path), 'r', encoding='utf-8') as f:
                    raw = yaml.safe_load(f)
                for root in ['/**', 'detection_node']:
                    if root in raw and 'ros__parameters' in raw[root]:
                        if logger:
                            logger.info(f'Loaded parameter configuration from: {path}')
                        return raw[root]['ros__parameters']
                if 'ros__parameters' in raw:
                    return raw['ros__parameters']
                return raw
            except Exception as e:
                if logger:
                    logger.warn(f'Failed to read {path}: {e}')
    return {}


class DetectionNode:
    """ROS 1 Detection Node orchestrating vision pipelines and ROS interfaces."""

    def __init__(self):
        self._logger = Ros1Logger()
        self._logger.info('Initializing DetectionNode (ROS 1 / dobot_v2)...')
        self.bridge = CvBridge()
        self.last_proc_time = 0.0

        # 1. Load Parameters from config/detection_node.yaml with ROS parameter overrides
        self.cfg = self._load_all_parameters()
        self.publish_rate = float(self.cfg.get('publish_rate', 30.0))
        self.min_frame_interval = 1.0 / max(1.0, self.publish_rate)
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 70, int(cv2.IMWRITE_JPEG_OPTIMIZE), 0]

        # 2. Initialize Subsystems
        self.transform = FieldTransform(self.cfg)
        self.grid_detector = GridDetector(self.cfg, logger=self._logger)
        self.cube_detector = CubeDetector(self.cfg)

        if self.grid_detector.grid_corners is not None:
            self.transform.update_corners(self.grid_detector.grid_corners)

        # 3. Setup ROS Interfaces
        self._init_ros_communication()
        self._logger.info('DetectionNode initialized cleanly from config/detection_node.yaml.')

    def get_logger(self):
        return self._logger

    def _load_all_parameters(self) -> dict:
        yaml_defaults = load_yaml_config(logger=self._logger)
        cfg = dict(yaml_defaults)

        # Read any parameter overrides from ROS parameter server
        if HAS_ROS1:
            for k in list(yaml_defaults.keys()):
                if rospy.has_param(f'~{k}'):
                    val = rospy.get_param(f'~{k}')
                    if isinstance(val, list) and len(val) > 0 and isinstance(val[0], (int, float)):
                        cfg[k] = [float(x) for x in val]
                    else:
                        cfg[k] = val

        # Format feeders
        cfg['feeder_positions'] = [
            {'id': i, 'x': cfg[f'feeder_{i}'][0], 'y': cfg[f'feeder_{i}'][1], 'radius': cfg[f'feeder_{i}'][2]}
            for i in range(1, 5) if f'feeder_{i}' in cfg
        ]
        return cfg

    def _init_ros_communication(self):
        if not HAS_ROS1:
            return

        img_topic = self.cfg.get('image_topic', '/usb_cam/image_raw')
        if 'compressed' in img_topic.lower():
            self.sub_img = rospy.Subscriber(img_topic, CompressedImage, self._on_compressed_img, queue_size=1, buff_size=2**24)
        else:
            self.sub_img = rospy.Subscriber(img_topic, Image, self._on_raw_img, queue_size=1, buff_size=2**24)

        cam_info_topic = self.cfg.get('camera_info_topic', '/usb_cam/camera_info')
        self.sub_cam_info = rospy.Subscriber(cam_info_topic, CameraInfo, lambda _: None, queue_size=10)

        # Control topics
        self.sub_reset = rospy.Subscriber('reset_grid', String, self._on_reset_grid, queue_size=10)
        self.sub_lock = rospy.Subscriber('lock_grid', String, self._on_lock_grid, queue_size=10)
        self.sub_unlock = rospy.Subscriber('unlock_grid', String, self._on_unlock_grid, queue_size=10)
        self.sub_corners = rospy.Subscriber('set_grid_corners', String, self._on_set_corners, queue_size=10)

        # Output publishers
        self.pub_comp = rospy.Publisher('detected_objects_image/compressed', CompressedImage, queue_size=10)
        self.pub_raw = rospy.Publisher('detected_objects_image', Image, queue_size=10) if self.cfg.get('publish_raw_image', False) else None
        self.pub_json = rospy.Publisher('detected_objects', String, queue_size=10)
        self.pub_poses_robot = rospy.Publisher('detected_objects_poses', PoseArray, queue_size=10)
        self.pub_poses_grid = rospy.Publisher('grid_local_poses', PoseArray, queue_size=10)

    # -------------------------------------------------------------------------
    # Frame Ingestion & Throttling
    # -------------------------------------------------------------------------

    def _on_compressed_img(self, msg: CompressedImage):
        if self._should_throttle():
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self._process(frame, msg.header)

    def _on_raw_img(self, msg: Image):
        if self._should_throttle():
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is not None:
            self._process(frame, msg.header)

    def _should_throttle(self) -> bool:
        now = time.time()
        # 20% jitter tolerance prevents camera clock jitter from cutting FPS in half
        margin = 0.80 if self.publish_rate >= 30.0 else 0.85
        if (now - self.last_proc_time) < (self.min_frame_interval * margin):
            return True
        self.last_proc_time = now
        return False

    # -------------------------------------------------------------------------
    # Main Pipeline & Demand-Driven Publishing
    # -------------------------------------------------------------------------

    def _process(self, frame: np.ndarray, header):
        try:
            # 1. Single HSV conversion for both grid detector and cube detector
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 2. Pallet Grid Detection (grayscale only computed if grid is still searching)
            gray = None
            if not self.grid_detector.is_locked:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners = self.grid_detector.detect(frame, hsv=hsv, gray=gray)
            if corners is not None and (self.transform.homography is None or not np.array_equal(corners, getattr(self.transform, 'last_src_corners', None))):
                self.transform.update_corners(corners)
                self.transform.last_src_corners = corners.copy()

            # 3. Cube Detection using precomputed HSV
            detected_objects, counts = self.cube_detector.detect(hsv, self.transform)

            # 4. Publish Telemetry & Poses
            self._publish_data(header, detected_objects, counts)

            # 5. On-Demand Visual Overlay (Bypass when no client is subscribed -> Saves 50%+ CPU)
            has_subscribers = (
                (self.pub_comp is not None and self.pub_comp.get_num_connections() > 0) or
                (self.pub_raw is not None and self.pub_raw.get_num_connections() > 0)
            )
            if self.cfg.get('show_overlay', True) and has_subscribers:
                pct = int(min(100, (self.grid_detector.stable_counter / max(1, self.grid_detector.lock_threshold)) * 100))
                vis = DetectionVisualizer.draw(
                    frame, self.transform, corners, detected_objects, counts,
                    self.grid_detector.is_locked, pct
                )
                self._publish_images(vis, header)
        except Exception as e:
            self._logger.error(f'Error in _process: {e}')

    def _publish_data(self, header, detected_objects: list, counts: dict):
        try:
            t_sec = header.stamp.to_sec() if hasattr(header.stamp, 'to_sec') else header.stamp.sec + header.stamp.nanosec * 1e-9
            payload = {
                'timestamp': t_sec,
                'grid_locked': bool(self.grid_detector.is_locked),
                'counts': counts,
                'total_objects': len(detected_objects),
                'objects': detected_objects
            }

            if self.pub_json is not None:
                json_msg = String()
                json_msg.data = json.dumps(payload)
                self.pub_json.publish(json_msg)

            if detected_objects and self.pub_poses_robot is not None:
                pa_robot = PoseArray()
                pa_robot.header.stamp = header.stamp
                pa_robot.header.frame_id = 'dobot_base'

                pa_grid = PoseArray()
                pa_grid.header.stamp = header.stamp
                pa_grid.header.frame_id = 'grid_origin'

                for obj in detected_objects:
                    # Robot poses
                    pr = Pose()
                    rx, ry, rz = obj.get('robot_coords', (0.0, 0.0, 0.0))
                    pr.position.x = float(rx)
                    pr.position.y = float(ry)
                    pr.position.z = float(rz)
                    pa_robot.poses.append(pr)

                    # Grid local poses
                    pg = Pose()
                    gx, gy = obj.get('grid_mm', (0.0, 0.0))
                    pg.position.x = float(gx)
                    pg.position.y = float(gy)
                    pg.position.z = 0.0
                    pa_grid.poses.append(pg)

                self.pub_poses_robot.publish(pa_robot)
                if self.pub_poses_grid is not None:
                    self.pub_poses_grid.publish(pa_grid)

        except Exception as e:
            self._logger.error(f'Error in _publish_data: {e}')

    def _publish_images(self, vis: np.ndarray, header):
        try:
            if self.pub_comp is not None and self.pub_comp.get_num_connections() > 0:
                comp_msg = CompressedImage()
                comp_msg.header = header
                comp_msg.format = 'jpeg'
                comp_msg.data = np.array(cv2.imencode('.jpg', vis, self._jpeg_params)[1]).tobytes()
                self.pub_comp.publish(comp_msg)

            if self.pub_raw is not None and self.pub_raw.get_num_connections() > 0:
                self.pub_raw.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
        except Exception as e:
            self._logger.error(f'Error in _publish_images: {e}')

    # -------------------------------------------------------------------------
    # Topic Control Callbacks
    # -------------------------------------------------------------------------

    def _on_reset_grid(self, msg: String):
        self._logger.info('Resetting grid tracker via /reset_grid command.')
        self.grid_detector.reset()

    def _on_lock_grid(self, msg: String):
        self._logger.info('Force-locking grid tracker.')
        self.grid_detector.force_lock()

    def _on_unlock_grid(self, msg: String):
        self._logger.info('Unlocking grid tracker.')
        self.grid_detector.unlock()

    def _on_set_corners(self, msg: String):
        try:
            corners = json.loads(msg.data)
            if len(corners) == 4:
                corners_arr = np.array(corners, dtype=np.float32)
                self.grid_detector.grid_corners = corners_arr
                self.grid_detector.is_locked = True
                self.transform.update_corners(corners_arr)
                self._logger.info('Updated and locked manual corners via /set_grid_corners.')
        except Exception as e:
            self._logger.error(f'Failed to parse corners JSON: {e}')


def main(args=None):
    if HAS_ROS1:
        rospy.init_node('detection_node', anonymous=False)
        node = DetectionNode()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
    else:
        print("[detection_node] rospy is not installed. Please install ROS 1 / rospy.")
        sys.exit(1)


if __name__ == '__main__':
    main()