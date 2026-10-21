"""Sensor controls for the VD16GZ (exposure, gain, frame rate) without extra dependencies.

The RGB and IR images come from the loopback devices made by vd16gz-bridge; the
sensor itself is controlled through its V4L2 sub-device, /dev/vd16gz-sensor.
Controls can be changed while streaming.

    import vd16gz
    cam = vd16gz.Sensor()
    cam.set_exposure(800)        # manual exposure in lines (one line = 7.69 us)
    cam.set_gain(4)              # analogue gain step 0..28
    cam.auto_exposure()          # back to the sensor's own auto exposure
    print(cam.get("exposure"), cam.temperature)

The same works from a shell:
    v4l2-ctl -d /dev/vd16gz-sensor -c auto_exposure=1,exposure=800
"""
import fcntl
import os
import struct

import numpy as np

SENSOR = "/dev/vd16gz-sensor"
RAW = "/dev/vd16gz-raw"
PIXEL_RATE = 160_800_000  # pixels per second
LINE_LENGTH = 1124 + 112  # width + horizontal blanking, in pixels
HEIGHT = 1364

VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C

CONTROLS = {
    "auto_exposure": 0x009A0901,  # 0 = auto (on-sensor AE), 1 = manual
    "exposure": 0x00980911,  # lines, 21..frame length
    "analogue_gain": 0x009E0903,  # 0..28
    "digital_gain": 0x009F0905,  # 256 = 1.0x .. 2048 = 8.0x
    "vertical_blanking": 0x009E0901,  # lines; sets the frame rate
    "auto_exposure_bias": 0x009A0913,  # menu index 0..16, 8 = 0 EV
    "ae_light_level_target": 0x00981921,  # 0..100, auto exposure target brightness
    "dark_calibration_pedestal": 0x00981924,  # black level of the 10-bit output
    "test_pattern": 0x009F0903,  # 0 = off, 2 = colour bars, ...
    "temperature": 0x00981920,  # read-only, degrees Celsius
}


def quarter_rgbi(raw, black=64):
    """Split a raw mosaic (H x W, 10-bit) into linear R, G, B, IR at quarter resolution.

    Every 4x4 block of the colour pattern (datasheet figure 22) gives one value per channel,
    the average of its 2 R, 8 G, 2 B and 4 IR pixels; nothing is interpolated.
    Returns an (H/4) x (W/4) x 4 float32 array, black level removed, in raw units (0..1023-black).
    """
    lin = np.clip(np.asarray(raw, np.float32) - black, 0, None)
    h, w = lin.shape[0] // 4, lin.shape[1] // 4
    lin = lin[: 4 * h, : 4 * w]

    def blocks(a):  # average the 2x2 samples of a half-resolution plane per 4x4 block
        return a.reshape(h, 2, w, 2).mean((1, 3))

    r = (lin[3::4, 0::4] + lin[1::4, 2::4]) / 2
    b = (lin[1::4, 0::4] + lin[3::4, 2::4]) / 2
    g = blocks((lin[0::2, 0::2] + lin[1::2, 1::2]) / 2)
    ir = blocks(lin[0::2, 1::2])
    return np.dstack([r, g, b, ir]).astype(np.float32)


def read_raw(device=RAW, frames=1, skip=0):
    """Read the raw mosaic from vd16gz-bridge, averaged over a number of frames (float64)."""
    import cv2  # only needed here

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {device}; is vd16gz-bridge running with the raw output?")
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    acc, n = None, 0
    try:
        for i in range(skip + frames):
            ok, img = cap.read()
            if not ok:
                raise RuntimeError(f"no frame from {device}")
            if i >= skip:
                acc = img.astype(np.float64) if acc is None else acc + img
                n += 1
    finally:
        cap.release()
    return acc.reshape(HEIGHT, -1) / n


def analog_gain_factor(code):
    """Analogue gain factor for control code 0..28: 32 / (32 - code), x1 (0 dB) .. x8 (18 dB).

    Datasheet DS14163, section 2.4.4."""
    return 32 / (32 - code)


def analog_gain_code(factor):
    """Control code for an analogue gain factor 1..8 (nearest step)."""
    return min(max(round(32 - 32 / factor), 0), 28)


class Sensor:
    def __init__(self, path=SENSOR):
        self.fd = os.open(path, os.O_RDWR)

    def close(self):
        os.close(self.fd)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def get(self, name):
        buf = bytearray(struct.pack("Ii", CONTROLS[name], 0))
        fcntl.ioctl(self.fd, VIDIOC_G_CTRL, buf)
        return struct.unpack("Ii", buf)[1]

    def set(self, name, value):
        fcntl.ioctl(self.fd, VIDIOC_S_CTRL, bytearray(struct.pack("Ii", CONTROLS[name], int(value))))

    def auto_exposure(self):
        self.set("auto_exposure", 0)

    def set_exposure(self, lines, gain=None):
        """Manual exposure in lines (exposure time = lines * LINE_LENGTH / PIXEL_RATE)."""
        self.set("auto_exposure", 1)
        self.set("exposure", lines)
        if gain is not None:
            self.set("analogue_gain", gain)

    def set_exposure_us(self, microseconds, gain=None):
        self.set_exposure(round(microseconds * 1e-6 * PIXEL_RATE / LINE_LENGTH), gain)

    def set_gain(self, gain):
        """Analogue gain control code 0..28 (factor 32 / (32 - code), see analog_gain_factor)."""
        self.set("analogue_gain", gain)

    def set_gain_x(self, factor):
        """Analogue gain as a factor, 1.0 .. 8.0 (rounded to the nearest of the 29 steps)."""
        self.set("analogue_gain", analog_gain_code(factor))

    @property
    def gain_x(self):
        return analog_gain_factor(self.get("analogue_gain"))

    def set_fps(self, fps):
        """Frame rate via vertical blanking (default 60 fps, sensor maximum about 88 fps).

        Exposure can not be longer than the frame, so lower the exposure first when
        raising the frame rate."""
        lines = round(PIXEL_RATE / (LINE_LENGTH * fps))
        self.set("vertical_blanking", max(lines - HEIGHT, 110))

    @property
    def fps(self):
        return PIXEL_RATE / (LINE_LENGTH * (HEIGHT + self.get("vertical_blanking")))

    @property
    def temperature(self):
        return self.get("temperature")
