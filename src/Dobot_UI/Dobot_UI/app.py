#!/usr/bin/env python3
"""Entry point for Dobot_UI node (ROS 1)."""

import os
import signal
import sys

# Support direct script execution (python3 app.py) as well as package imports
if __package__ is None or __package__ == "":
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(pkg_dir)
    if workspace_dir not in sys.path:
        sys.path.insert(0, workspace_dir)
    from Dobot_UI.config import WINDOW
    from Dobot_UI.main_window import DobotMainWindow
    from Dobot_UI.ros_bridge import RosBridge
else:
    from .config import WINDOW
    from .main_window import DobotMainWindow
    from .ros_bridge import RosBridge

from PyQt5.QtWidgets import QApplication

try:
    import rospy
    HAS_ROS1 = True
except ImportError:
    HAS_ROS1 = False


def main(args=None):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    cli_args = sys.argv if args is None else list(args)
    mock = (
        "--mock" in cli_args
        or "-m" in cli_args
        or "--simulate" in cli_args
        or os.environ.get("DOBOT_UI_MOCK", "0") in ("1", "true", "True")
        or not HAS_ROS1
    )

    clean_args = list(cli_args)
    if HAS_ROS1 and not mock:
        try:
            clean_args = rospy.myargv(argv=cli_args)
        except Exception:
            pass

    app = QApplication(clean_args)
    app.setApplicationName(WINDOW.get("title", "Dobot Magician UI (ROS 1)"))

    bridge = RosBridge(mock_mode=mock)
    window = DobotMainWindow(bridge)
    window.show()

    bridge.start()
    ret = app.exec_()
    bridge.stop()

    if HAS_ROS1:
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("UI closed")
        except Exception:
            pass

    sys.exit(ret)


if __name__ == "__main__":
    main()
