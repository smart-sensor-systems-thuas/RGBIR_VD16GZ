#!/usr/bin/env python3
"""Check that /dev/vd16gz-rgb and /dev/vd16gz-ir deliver the same sensor frames.

Test 1, timestamps: both devices are read at the same time and every RGB frame is
paired with the IR frame carrying the same sensor timestamp. Synchronized streams
give pairs with exactly 0 us difference and no near misses.

Test 2, content: the sensor exposure is switched between two values, so the image
brightness jumps on one particular sensor frame. That jump has to show up in RGB
and IR at the same timestamp; a frame offset between the streams would show here.
The camera needs to see a scene that is not completely dark or saturated.

Requires vd16gz-bridge to be running. Sensor settings are restored afterwards.
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vd16gz  # noqa: E402


class Reader(threading.Thread):
    """Reads one device and records (timestamp in us, mean brightness) per frame."""

    def __init__(self, path):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            sys.exit(f"cannot open {path}; is vd16gz-bridge running?")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        self.path = path
        self.frames = []
        self.recording = False
        self.stop = False

    def run(self):
        while not self.stop:
            ok, img = self.cap.read()
            if not ok:
                continue
            ts = round(self.cap.get(cv2.CAP_PROP_POS_MSEC) * 1000)
            if self.recording:
                self.frames.append((ts, float(img[::4, ::4].mean())))
        self.cap.release()



def record(rgb, ir, seconds):
    """Record both readers for the same period; returns {timestamp: brightness} per stream."""
    rgb.frames, ir.frames = [], []
    rgb.recording = ir.recording = True
    time.sleep(seconds)
    rgb.recording = ir.recording = False
    return dict(rgb.frames), dict(ir.frames)


def timestamp_test(rgb, ir, seconds):
    print(f"Test 1: timestamps ({seconds:.0f} s)")
    a, b = record(rgb, ir, seconds)
    ta, tb = np.array(sorted(a)), np.array(sorted(b))
    period = np.median(np.diff(ta))
    common = np.intersect1d(ta, tb)
    # nearest IR timestamp for every RGB frame
    idx = np.clip(np.searchsorted(tb, ta), 1, len(tb) - 1)
    nearest = np.minimum(np.abs(tb[idx] - ta), np.abs(tb[idx - 1] - ta))
    near_miss = int(((nearest > 0) & (nearest < period / 2)).sum())
    print(f"  RGB frames {len(ta)}, IR frames {len(tb)}, frame period {period / 1000:.3f} ms "
          f"({1e6 / period:.1f} fps)")
    print(f"  RGB frames with an IR frame of identical timestamp: {len(common)} "
          f"({100 * len(common) / len(ta):.1f} %)")
    print(f"  pairs offset by less than half a frame (would mean out of sync): {near_miss}")
    unmatched = len(ta) - len(common)
    if unmatched:
        print(f"  {unmatched} frames without a twin: dropped by this reader, not offset")
    ok = near_miss == 0 and len(common) >= 0.9 * len(ta)
    print("  PASS" if ok else "  FAIL")
    return ok


def steps(frames, threshold):
    """Timestamps where brightness jumps between two consecutive frames."""
    ts = sorted(frames)
    out = {}
    for t0, t1 in zip(ts, ts[1:]):
        m0, m1 = frames[t0], frames[t1]
        if min(m0, m1) > 1 and abs(np.log(m1 / m0)) > threshold:
            out[t1] = t0
    return out


def content_test(rgb, ir, seconds, toggle):
    print(f"Test 2: exposure steps ({seconds:.0f} s, switching every {toggle} s)")
    with vd16gz.Sensor() as s:
        saved = {k: s.get(k) for k in ("auto_exposure", "exposure")}
        e1 = max(saved["exposure"], 120)
        e2 = max(e1 // 3, 21)
        result = {}

        def toggler():
            n = 0
            end = time.time() + seconds
            while time.time() < end:
                s.set_exposure(e1 if n % 2 == 0 else e2)
                n += 1
                time.sleep(toggle)

        try:
            s.set_exposure(e1)
            time.sleep(0.5)
            t = threading.Thread(target=toggler)
            t.start()
            result["a"], result["b"] = record(rgb, ir, seconds)
            t.join()
        finally:
            s.set("auto_exposure", saved["auto_exposure"])
            if saved["auto_exposure"] == 1:
                s.set("exposure", saved["exposure"])
    a, b = result["a"], result["b"]
    print(f"  exposure {e1} <-> {e2} lines")
    sa, sb = steps(a, 0.15), steps(b, 0.15)
    # only judge steps where both streams have the frame before and the frame of the step
    judged = [t for t in sa if t in b and sa[t] in b]
    judged += [t for t in sb if t in a and sb[t] in a and t not in sa]
    same = [t for t in judged if t in sa and t in sb]
    print(f"  brightness steps: RGB {len(sa)}, IR {len(sb)}; seen by both streams: {len(judged)}, "
          f"at the same frame: {len(same)}")
    if not judged:
        print("  no usable steps: is the scene too dark or saturated?")
        print("  INCONCLUSIVE")
        return None
    ok = len(same) == len(judged)
    print("  PASS" if ok else "  FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rgb", default="/dev/vd16gz-rgb")
    ap.add_argument("--ir", default="/dev/vd16gz-ir")
    ap.add_argument("--seconds", type=float, default=5)
    ap.add_argument("--toggle", type=float, default=0.25, help="seconds between exposure changes")
    ap.add_argument("--no-exposure-test", action="store_true",
                    help="skip test 2 (it changes the sensor exposure for a few seconds)")
    args = ap.parse_args()

    rgb, ir = Reader(args.rgb), Reader(args.ir)
    rgb.start()
    ir.start()
    time.sleep(1)  # let both readers settle

    results = [timestamp_test(rgb, ir, args.seconds)]
    if not args.no_exposure_test:
        results.append(content_test(rgb, ir, args.seconds, args.toggle))
    rgb.stop = ir.stop = True
    rgb.join()
    ir.join()
    if False in results:
        sys.exit("NOT synchronized")
    print("RGB and IR are synchronized" if None not in results else "Timestamps synchronized; content test inconclusive")


if __name__ == "__main__":
    main()
