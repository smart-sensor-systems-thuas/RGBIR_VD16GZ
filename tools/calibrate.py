#!/usr/bin/env python3
"""Fit a colour correction matrix for the VD16GZ from a ColorChecker Classic shot.

Fits linear sRGB = M @ [R, G, B, IR] (3x4), which corrects colour and removes IR
leakage in one step, and saves it to ccm.json.

Usage:
  capture.py chart --average 8          # ColorChecker filling a good part of the image
  calibrate.py chart_rgbi.npy --corners x,y x,y x,y x,y
  sudo cp ccm.json /etc/vd16gz/ccm.json && sudo systemctl restart vd16gz-bridge
Corners are the centres of the dark skin, bluish green, white and black patches
(pixel coordinates in chart_rgb.png).
"""
import argparse
import json
import os

import numpy as np
from PIL import Image, ImageDraw

# ColorChecker Classic reference, sRGB 8-bit (D65), patches 1-24 in reading order
REFERENCE_SRGB = np.array([
    [115, 82, 68], [194, 150, 130], [98, 122, 157], [87, 108, 67], [133, 128, 177], [103, 189, 170],
    [214, 126, 44], [80, 91, 166], [193, 90, 99], [94, 60, 108], [157, 188, 64], [224, 163, 46],
    [56, 61, 150], [70, 148, 73], [175, 54, 60], [231, 199, 31], [187, 86, 149], [8, 133, 161],
    [243, 243, 242], [200, 200, 200], [160, 160, 160], [122, 122, 121], [85, 85, 85], [52, 52, 52],
], np.float64) / 255
NAMES = ["dark skin", "light skin", "blue sky", "foliage", "blue flower", "bluish green",
         "orange", "purplish blue", "moderate red", "purple", "yellow green", "orange yellow",
         "blue", "green", "red", "yellow", "magenta", "cyan",
         "white", "neutral 8", "neutral 6.5", "neutral 5", "neutral 3.5", "black"]


def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(c):
    c = np.clip(c, 0, 1)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)


def linear_to_lab(rgb):
    xyz = rgb @ np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]]).T
    xyz = xyz / np.array([0.95047, 1.0, 1.08883])
    f = np.where(xyz > (6 / 29) ** 3, np.cbrt(xyz), xyz / (3 * (6 / 29) ** 2) + 4 / 29)
    return np.stack([116 * f[:, 1] - 16, 500 * (f[:, 0] - f[:, 1]), 200 * (f[:, 1] - f[:, 2])], -1)


def delta_e2000(lab1, lab2):
    L1, a1, b1 = lab1.T
    L2, a2, b2 = lab2.T
    C7 = ((np.hypot(a1, b1) + np.hypot(a2, b2)) / 2) ** 7
    G = 0.5 * (1 - np.sqrt(C7 / (C7 + 25 ** 7)))
    a1p, a2p = a1 * (1 + G), a2 * (1 + G)
    C1p, C2p = np.hypot(a1p, b1), np.hypot(a2p, b2)
    h1p, h2p = np.degrees(np.arctan2(b1, a1p)) % 360, np.degrees(np.arctan2(b2, a2p)) % 360
    dLp, dCp = L2 - L1, C2p - C1p
    dh = h2p - h1p
    dh = np.where(C1p * C2p == 0, 0, np.where(dh > 180, dh - 360, np.where(dh < -180, dh + 360, dh)))
    dHp = 2 * np.sqrt(C1p * C2p) * np.sin(np.radians(dh / 2))
    Lbp, Cbp = (L1 + L2) / 2, (C1p + C2p) / 2
    hsum = h1p + h2p
    hbp = np.where(C1p * C2p == 0, hsum,
                   np.where(np.abs(h1p - h2p) <= 180, hsum / 2, np.where(hsum < 360, (hsum + 360) / 2, (hsum - 360) / 2)))
    T = (1 - 0.17 * np.cos(np.radians(hbp - 30)) + 0.24 * np.cos(np.radians(2 * hbp))
         + 0.32 * np.cos(np.radians(3 * hbp + 6)) - 0.20 * np.cos(np.radians(4 * hbp - 63)))
    SL = 1 + 0.015 * (Lbp - 50) ** 2 / np.sqrt(20 + (Lbp - 50) ** 2)
    SC, SH = 1 + 0.045 * Cbp, 1 + 0.015 * Cbp * T
    Cb7 = Cbp ** 7
    RT = -2 * np.sqrt(Cb7 / (Cb7 + 25 ** 7)) * np.sin(np.radians(60 * np.exp(-(((hbp - 275) / 25) ** 2))))
    return np.sqrt((dLp / SL) ** 2 + (dCp / SC) ** 2 + (dHp / SH) ** 2 + RT * (dCp / SC) * (dHp / SH))


def patch_centres(corners):
    """Bilinear grid from the centres of patches 1, 6, 19 and 24 (dark skin, bluish green, white, black)."""
    p1, p6, p19, p24 = (np.array(c, np.float64) for c in corners)
    centres = []
    for row in range(4):
        v = row / 3
        for col in range(6):
            u = col / 5
            centres.append((1 - v) * ((1 - u) * p1 + u * p6) + v * ((1 - u) * p19 + u * p24))
    return np.array(centres)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rgbi", help="linear 4-channel capture from capture.py (<prefix>_rgbi.npy)")
    ap.add_argument("--corners", nargs=4, required=True, metavar="X,Y",
                    help="centres of patches dark skin, bluish green, white, black")
    ap.add_argument("--box", type=int, default=30, help="half-size of the sampling box in pixels")
    ap.add_argument("--out", default="ccm.json")
    ap.add_argument("--max-de", type=float, default=5.0,
                    help="only save the matrix if the mean dE2000 is below this (default 5)")
    args = ap.parse_args()

    rgbi = np.load(args.rgbi).astype(np.float64)
    centres = patch_centres([tuple(map(float, c.split(","))) for c in args.corners])
    b = args.box
    measured = np.array([np.median(rgbi[int(y) - b:int(y) + b, int(x) - b:int(x) + b].reshape(-1, 4), 0)
                         for x, y in centres])
    if (measured[:, :3] >= 0.98).any():
        print("WARNING: some patches are clipped; recapture with a lower --exposure")

    # Overlay sampling boxes on the preview so the patch positions can be checked
    prefix = args.rgbi.removesuffix("_rgbi.npy")
    preview = Image.open(prefix + "_rgb.png").convert("RGB")
    draw = ImageDraw.Draw(preview)
    for i, (x, y) in enumerate(centres):
        draw.rectangle([x - b, y - b, x + b, y + b], outline=(255, 0, 255), width=3)
        draw.text((x - b + 4, y - b + 4), str(i + 1), fill=(255, 0, 255))
    preview.save(prefix + "_patches.png")

    target = srgb_to_linear(REFERENCE_SRGB)
    ref_lab = linear_to_lab(target)
    results = {}
    for name, cols in (("3x3 (RGB only)", 3), ("3x4 (RGB + IR)", 4)):
        X = measured[:, :cols]
        M, *_ = np.linalg.lstsq(X, target, rcond=None)
        de = delta_e2000(ref_lab, linear_to_lab(np.clip(X @ M, 0, None)))
        results[name] = (M.T, de)
        print(f"{name}: mean dE2000 {de.mean():.2f}, max {de.max():.2f} ({NAMES[de.argmax()]})")

    M, de = results["3x4 (RGB + IR)"]
    print("\nPer patch dE2000 (3x4):")
    for n, d in zip(NAMES, de):
        print(f"  {n:14s} {d:5.2f}")
    print("\nMatrix (rows: out R, G, B; cols: in R, G, B, IR):")
    print(np.array2string(M, precision=4, suppress_small=True))

    corrected = linear_to_srgb(np.clip(np.einsum("ij,hwj->hwi", M, rgbi), 0, 1))
    Image.fromarray((corrected * 255 + 0.5).astype(np.uint8)).save(prefix + "_corrected.png")
    if de.mean() > args.max_de:
        print(f"\nNOT saving {args.out}: mean dE2000 {de.mean():.2f} is above --max-de {args.max_de}. "
              "Check the lighting (even, with enough blue) and the patch overlay.")
        return
    with open(args.out, "w") as f:
        json.dump({"matrix": M.tolist(), "source": os.path.abspath(args.rgbi),
                   "mean_dE2000": round(float(de.mean()), 3), "max_dE2000": round(float(de.max()), 3)}, f, indent=2)
    print(f"\nSaved {args.out}, {prefix}_corrected.png and {prefix}_patches.png")
    print(f"Use it in the bridge: sudo cp {args.out} /etc/vd16gz/ccm.json && sudo systemctl restart vd16gz-bridge")


if __name__ == "__main__":
    main()
