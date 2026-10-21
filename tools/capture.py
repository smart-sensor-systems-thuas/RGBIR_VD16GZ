#!/usr/bin/env python3
"""Capture linear RGB-IR stills from the VD16GZ for analysis and colour calibration.

Reads the unprocessed mosaic from /dev/vd16gz-raw (published by vd16gz-bridge),
optionally averages several frames, and saves:
  <prefix>_raw.npy    10-bit mosaic (H x W, uint16)
  <prefix>_rgbi.npy   linear R, G, B, IR in 0..1 (H x W x 4, float32), black level removed
  <prefix>_rgb.png    preview (white balanced, sRGB; uses the colour matrix if given)
  <prefix>_ir.png     IR preview (sRGB)

The interpolation is the same as in vd16gz-bridge (normalized convolution).
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
from PIL import Image

WIDTH, HEIGHT = 1124, 1364
BLACK_LEVEL = 64  # dark_calibration_pedestal, 10-bit

# 4x4 colour filter layout at full resolution with no flips, top-left pixel first.
CFA = [
    "GIGI",
    "BGRG",
    "GIGI",
    "RGBG",
]
CHANNELS = "RGBI"
KERNELS = {"G": [1, 2, 1], "I": [1, 2, 1], "R": [1, 2, 3, 2, 1], "B": [1, 2, 3, 2, 1]}


def grab_raw(device, frames, skip):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {device}; is vd16gz-bridge running with the raw output enabled?")
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    acc = np.zeros((HEIGHT, WIDTH), np.float64)
    n = 0
    for i in range(skip + frames):
        ok, img = cap.read()
        if not ok:
            raise SystemExit(f"no frame from {device}")
        if i >= skip:
            acc += img.reshape(HEIGHT, WIDTH)
            n += 1
    cap.release()
    return acc / n


def cfa_masks(swap_rb):
    tile = [row.translate(str.maketrans("RB", "BR")) if swap_rb else row for row in CFA]
    reps = (HEIGHT // 4 + 1, WIDTH // 4 + 1)
    return {
        c: np.tile(np.array([[ch == c for ch in row] for row in tile]), reps)[:HEIGHT, :WIDTH]
        for c in CHANNELS
    }


def _sep_conv(img, k):
    r = len(k) // 2
    p = np.pad(img, r, mode="reflect")
    tmp = sum(w * p[:, i : i + img.shape[1]] for i, w in enumerate(k))
    return sum(w * tmp[i : i + img.shape[0], :] for i, w in enumerate(k))


def demosaic(raw, masks):
    """Normalized-convolution interpolation; returns HxWx4 float32 (R,G,B,IR), linear, 0..1."""
    lin = np.clip(raw.astype(np.float32) - BLACK_LEVEL, 0, None) / (1023 - BLACK_LEVEL)
    out = np.empty((HEIGHT, WIDTH, 4), np.float32)
    for i, c in enumerate(CHANNELS):
        m = masks[c].astype(np.float32)
        k = np.array(KERNELS[c], np.float32)
        num, den = _sep_conv(lin * m, k), _sep_conv(m, k)
        if (den == 0).any():  # image corners with no sample in reach: use a wider kernel there
            wide = np.convolve(k, k)
            num, den = np.where(den == 0, _sep_conv(lin * m, wide), num), np.where(den == 0, _sep_conv(m, wide), den)
        out[..., i] = np.where(masks[c], lin, num / den)  # keep measured pixels exact
    return out


def srgb_8bit(x):
    x = np.clip(x, 0, 1)
    return (np.where(x <= 0.0031308, x * 12.92, 1.055 * x ** (1 / 2.4) - 0.055) * 255 + 0.5).astype(np.uint8)


def preview(rgbi, ccm):
    rgb = rgbi[..., :3] if ccm is None else rgbi @ ccm.T
    if ccm is None:  # grey world white balance
        means = rgb.reshape(-1, 3).mean(0)
        rgb = rgb * (means[1] / np.maximum(means, 1e-6))
    return srgb_8bit(rgb), srgb_8bit(rgbi[..., 3])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output", nargs="?", help="output name prefix (default vd16gz_<date>_<time>)")
    ap.add_argument("--device", default="/dev/vd16gz-raw")
    ap.add_argument("--average", type=int, default=1, metavar="N", help="average N frames to lower noise")
    ap.add_argument("--skip", type=int, default=5, help="frames to discard first (default 5)")
    ap.add_argument("--ccm", default="/etc/vd16gz/ccm.json", help="colour matrix for the preview, if present")
    ap.add_argument("--swap-rb", action="store_true", help="swap the R and B sites of the CFA")
    args = ap.parse_args()

    prefix = args.output or time.strftime("vd16gz_%Y%m%d_%H%M%S")
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    ccm = None
    if args.ccm and os.path.exists(args.ccm):
        with open(args.ccm) as f:
            ccm = np.array(json.load(f)["matrix"], np.float32)

    raw = grab_raw(args.device, args.average, args.skip)
    rgbi = demosaic(raw, cfa_masks(args.swap_rb))
    rgb8, ir8 = preview(rgbi, ccm)
    np.save(prefix + "_raw.npy", np.round(raw).astype(np.uint16))
    np.save(prefix + "_rgbi.npy", rgbi)
    Image.fromarray(rgb8).save(prefix + "_rgb.png")
    Image.fromarray(ir8).save(prefix + "_ir.png")
    print(f"Saved {prefix}_rgb.png, _ir.png, _rgbi.npy (HxWx4 linear R,G,B,IR), _raw.npy (mosaic)")


if __name__ == "__main__":
    main()
