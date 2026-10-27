#!/usr/bin/env python3
"""Runs one scan from the command line: calibrates the gyro if needed, scans for N seconds,
stops. With simulate:=true this is the end-to-end test of the whole chain.

    ros2 launch laser_hhrp hhrp.launch.py simulate:=true      # in one terminal
    ros2 run laser_hhrp timed_scan.py 3                        # in another
    ros2 run laser_hhrp check_session.py                       # check what was recorded
"""

from __future__ import annotations

import sys
import time

import rclpy
from std_srvs.srv import Trigger

from laser_hhrp.msg import ScanStatus

CALIBRATED = 2


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = rclpy.create_node("timed_scan")
    status: list[ScanStatus] = []
    node.create_subscription(ScanStatus, "scan/status", lambda m: status.append(m), 10)

    def call(name: str) -> bool:
        client = node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=10.0):
            print(f"{name}: not available")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        print(f"{name}: {future.result().message}")
        return future.result().success

    def wait(condition, timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if status and condition(status[-1]):
                return True
        return False

    ok = wait(lambda s: True, 15.0)
    if ok and status[-1].calibration != CALIBRATED:
        ok = call("calibration/start") and wait(lambda s: s.calibration == CALIBRATED, 30.0)
    # The devices must have reported in before a scan can start.
    for _ in range(20):
        if not ok or call("scan/start"):
            break
        time.sleep(0.5)
    else:
        ok = False
    if ok:
        wait(lambda s: False, seconds)
        call("scan/stop")
        wait(lambda s: False, 1.0)  # let the recorder close the session
        print(f"frames: {status[-1].frames}")
        ok = status[-1].frames > 0
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
