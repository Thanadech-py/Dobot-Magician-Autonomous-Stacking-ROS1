#!/usr/bin/env python3
"""USB Camera capture node for ROS 1."""

import time
import cv2
import rospy
from sensor_msgs.msg import Image


class USBCamNode:
    def __init__(self):
        rospy.init_node('usb_cam_node', anonymous=False)

        self.dev = rospy.get_param('~device_id', 0)
        self.width = int(rospy.get_param('~width', 640))
        self.height = int(rospy.get_param('~height', 480))
        self.fps = float(rospy.get_param('~fps', 30.0))
        self.frame_id = rospy.get_param('~frame_id', 'camera_frame')

        self.pub = rospy.Publisher('image_raw', Image, queue_size=1)

        self.msg = Image()
        self.msg.header.frame_id = str(self.frame_id)
        self.msg.encoding = 'bgr8'
        self.msg.is_bigendian = 0

        self.cap = None
        self._open_camera()

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
            time.sleep(0.2)

        dev = int(self.dev) if str(self.dev).isdigit() else self.dev
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        try:
            self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
            self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
        except Exception:
            pass

        rospy.loginfo(f'[USBCamNode] Camera opened on {self.dev}')

    def run(self):
        consecutive_failures = 0
        rate = rospy.Rate(max(1.0, self.fps))
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if not ret:
                consecutive_failures += 1
                rospy.logwarn(f'[USBCamNode] Frame read failed ({consecutive_failures})')
                if consecutive_failures >= 10:
                    rospy.logerr('[USBCamNode] Too many failures, reopening camera')
                    self._open_camera()
                    consecutive_failures = 0
                time.sleep(0.05)
                continue

            consecutive_failures = 0
            self.msg.header.stamp = rospy.Time.now()
            self.msg.height, self.msg.width, _ = frame.shape
            self.msg.step = self.msg.width * 3
            self.msg.data = frame.tobytes()
            self.pub.publish(self.msg)
            rate.sleep()


def main():
    node = USBCamNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()


if __name__ == '__main__':
    main()