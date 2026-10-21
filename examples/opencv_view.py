#!/usr/bin/env python3
"""Show the VD16GZ RGB and IR images side by side with OpenCV.

Keys: + / - exposure up/down, a auto exposure, s save a PNG pair, q quit.
Needs a desktop session (cv2.imshow); see web_stream.py for a browser view.
"""
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import vd16gz  # noqa: E402  (installed by install.sh, else taken from ../tools)


class LatestFrame(threading.Thread):
    """Keeps reading a device so that .get() always returns the newest frame and its timestamp."""

    def __init__(self, path):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open {path}; is the vd16gz-bridge service running?")
        self.lock = threading.Lock()
        self.frame, self.ts = None, None
        self.start()

    def run(self):
        while True:
            ok, img = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.ts = img, self.cap.get(cv2.CAP_PROP_POS_MSEC)

    def get(self):
        with self.lock:
            return self.frame, self.ts


def main():
    rgb, ir = LatestFrame("/dev/vd16gz-rgb"), LatestFrame("/dev/vd16gz-ir")
    sensor = vd16gz.Sensor()
    exposure = None
    while True:
        (c, tc), (i, ti) = rgb.get(), ir.get()
        if c is None or i is None:
            time.sleep(0.01)
            continue
        view = np.hstack([c, i if i.ndim == 3 else cv2.cvtColor(i, cv2.COLOR_GRAY2BGR)])
        view = cv2.resize(view, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        label = f"exposure {'auto' if exposure is None else exposure}  " \
                f"timestamps {'equal' if tc == ti else f'differ by {abs(tc - ti):.1f} ms'}"
        cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("VD16GZ  RGB | IR", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key in (ord("+"), ord("=")):
            exposure = min(int((exposure or sensor.get("exposure")) * 1.25) + 1, 2000)
            sensor.set_exposure(exposure)
        elif key == ord("-"):
            exposure = max(int((exposure or sensor.get("exposure")) / 1.25), 21)
            sensor.set_exposure(exposure)
        elif key == ord("a"):
            exposure = None
            sensor.auto_exposure()
        elif key == ord("s"):
            name = time.strftime("vd16gz_%Y%m%d_%H%M%S")
            cv2.imwrite(name + "_rgb.png", c)
            cv2.imwrite(name + "_ir.png", i)
            print("saved", name + "_rgb.png", name + "_ir.png")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
