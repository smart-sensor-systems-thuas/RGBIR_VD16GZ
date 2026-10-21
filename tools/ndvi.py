#!/usr/bin/env python3
"""NDVI (normalized difference vegetation index) from the VD16GZ RGB-IR sensor.

    NDVI = (NIR - Red) / (NIR + Red)

NIR is the IR pixel signal. Red is R - K * IR, because the R pixels also see near-infrared
about as strongly as the IR pixels (datasheet DS14163, figure 19). Values come from the raw
mosaic at quarter resolution (281 x 341 per channel), so every value is measured, not
interpolated, and R and IR come from the same exposure.

For real NDVI the signals must be converted to reflectance with a reference panel in the
image: a grey or white target with known, flat reflectance in red and NIR. Without a panel
the result is uncalibrated and only useful for comparing areas within one image.

Coordinates (--panel, --roi) are pixels of the full-resolution image (1124 x 1364) after
--rotate, so they can be read from the RGB image or from <prefix>_ndvi.png.

Examples:
  ndvi.py tree --panel 480,900,60,60 --panel-reflectance 0.5 --roi 600,700,200,150
  ndvi.py tree --select                 # draw the panel and areas with the mouse (needs a display)
  ndvi.py --live --panel 480,900,60,60  # live NDVI window
  ndvi.py old --input old_raw.npy       # from a capture.py raw file

Saves <prefix>_ndvi.png (colour preview | NDVI map | scale), <prefix>_ndvi.npy (float NDVI,
NaN where the signal is too low) and prints NDVI statistics for the panel and every area.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vd16gz  # noqa: E402

SCALE = 4  # quarter-resolution block size in full-resolution pixels
# RdYlGn-like colour scale (RGB), evenly spread over the display range
STOPS = np.array([(165, 0, 38), (244, 109, 67), (255, 255, 191), (102, 189, 99), (0, 104, 55)], np.float32)


def parse_box(text):
    x, y, w, h = (int(v) for v in text.split(","))
    return x, y, w, h


def box_slices(box):
    """Full-resolution box -> slices in the quarter-resolution arrays."""
    x, y, w, h = box
    return slice(y // SCALE, max((y + h) // SCALE, y // SCALE + 1)), slice(x // SCALE, max((x + w) // SCALE, x // SCALE + 1))


def compute(rgbi, args, panel):
    """Returns (ndvi, red, nir, info) for a quarter-resolution R, G, B, IR array."""
    nir = rgbi[..., 3]
    red = np.clip(rgbi[..., 0] - args.ir_subtract * nir, 0, None)
    info = {}
    if panel is not None:
        sl = box_slices(panel)
        p_red, p_nir = float(np.median(red[sl])), float(np.median(nir[sl]))
        if p_red <= 0 or p_nir <= 0:
            raise SystemExit("the panel area has no red or NIR signal; check its position and the exposure")
        refl_red, refl_nir = args.panel_reflectance
        red_r, nir_r = red / p_red * refl_red, nir / p_nir * refl_nir
        info.update(panel_red=p_red, panel_nir=p_nir)
    else:
        red_r, nir_r = red, nir
    total = red_r + nir_r
    with np.errstate(invalid="ignore", divide="ignore"):
        ndvi = np.clip((nir_r - red_r) / total, -1, 1)
    ndvi[(red + nir) < args.min_signal] = np.nan
    return ndvi, red, nir, info


def colourize(ndvi, lo, hi):
    t = np.clip((np.nan_to_num(ndvi, nan=lo) - lo) / (hi - lo), 0, 1) * (len(STOPS) - 1)
    i = np.minimum(t.astype(int), len(STOPS) - 2)
    f = (t - i)[..., None]
    rgb = STOPS[i] * (1 - f) + STOPS[i + 1] * f
    rgb[np.isnan(ndvi)] = (60, 60, 60)
    return rgb.astype(np.uint8)


def colour_preview(rgbi, ir_subtract):
    rgb = np.clip(rgbi[..., :3] - ir_subtract * rgbi[..., 3:4], 0, None)
    rgb = rgb * (rgb[..., 1].mean() / np.maximum(rgb.reshape(-1, 3).mean(0), 1e-6))
    rgb = rgb / max(np.percentile(rgb, 99.5), 1e-6)
    return (np.clip(rgb, 0, 1) ** (1 / 2.2) * 255).astype(np.uint8)


def scale_bar(height, lo, hi):
    bar = np.zeros((height, 110, 3), np.uint8) + 255
    vals = np.linspace(hi, lo, height)[:, None]
    bar[:, 10:50] = colourize(np.repeat(vals, 40, 1), lo, hi)
    for v in np.arange(np.ceil(lo * 10) / 10, hi + 1e-6, 0.1 if hi - lo <= 1.2 else 0.2):
        y = int((hi - v) / (hi - lo) * (height - 1))
        cv2.line(bar, (50, y), (58, y), (0, 0, 0), 2)
        cv2.putText(bar, f"{v:+.1f}", (60, min(max(y + 8, 16), height - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return bar


def render(rgbi, ndvi, args, panel, rois):
    """Colour preview | NDVI map | scale, at full resolution, as RGB."""
    up = lambda a: cv2.resize(a, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
    left, right = up(colour_preview(rgbi, args.ir_subtract)), up(colourize(ndvi, *args.range))
    for img in (left, right):
        if panel is not None:
            x, y, w, h = panel
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 3)
            cv2.putText(img, "panel", (x, max(y - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        for n, (x, y, w, h) in enumerate(rois, 1):
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 3)
            label = f"{n}: {np.nanmedian(ndvi[box_slices((x, y, w, h))]):+.2f}"
            cv2.putText(img, label, (x, max(y - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
            cv2.putText(img, label, (x, max(y - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    title = "NDVI" + ("" if panel is not None else " (uncalibrated: no panel)")
    cv2.putText(right, title, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 5)
    cv2.putText(right, title, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 2)
    return np.hstack([left, right, scale_bar(right.shape[0], *args.range)])


def stats_line(name, values):
    v = values[~np.isnan(values)]
    if v.size == 0:
        return f"  {name:10s} no valid pixels"
    return (f"  {name:10s} median {np.median(v):+.3f}  mean {v.mean():+.3f}  std {v.std():.3f}  "
            f"valid {100 * v.size / values.size:.0f} %")


def acquire(args, frames):
    if args.input:
        raw = np.load(args.input).astype(np.float64)
    else:
        raw = vd16gz.read_raw(args.device, frames=frames, skip=args.skip)
    if raw.max() >= 1000:
        print(f"WARNING: {np.mean(raw >= 1000) * 100:.2f} % of the pixels are saturated; "
              "lower the exposure for reliable NDVI")
    rgbi = vd16gz.quarter_rgbi(raw, args.black)
    if args.rotate:
        rgbi = np.rot90(rgbi, args.rotate // 90).copy()
    return rgbi


def select_boxes(rgbi, args):
    """Let the user draw the panel and areas on the colour preview."""
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise SystemExit("--select needs a desktop session; give --panel / --roi coordinates instead")
    img = cv2.cvtColor(cv2.resize(colour_preview(rgbi, args.ir_subtract), None, fx=SCALE, fy=SCALE), cv2.COLOR_RGB2BGR)
    show = cv2.resize(img, None, fx=0.6, fy=0.6)
    to_full = lambda b: tuple(int(round(v / 0.6)) for v in b)
    print("Draw the reference panel, then press Enter (Esc / c: no panel)")
    p = cv2.selectROI("select panel", show)
    cv2.destroyWindow("select panel")
    panel = to_full(p) if p[2] and p[3] else None
    print("Draw the areas to measure, Enter after each one, Esc when done")
    rois = [to_full(b) for b in cv2.selectROIs("select areas", show) if b[2] and b[3]]
    cv2.destroyAllWindows()
    return panel, rois


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output", nargs="?", help="output name prefix (default vd16gz_<date>_<time>)")
    ap.add_argument("--input", help="raw mosaic .npy from capture.py instead of the live camera")
    ap.add_argument("--device", default=vd16gz.RAW)
    ap.add_argument("--frames", type=int, default=16, help="frames to average (default 16)")
    ap.add_argument("--skip", type=int, default=3, help="frames to discard first (default 3)")
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270),
                    help="rotate the image (degrees counter-clockwise), e.g. 180 for an upside-down camera")
    ap.add_argument("--panel", type=parse_box, metavar="X,Y,W,H", help="reference panel area")
    ap.add_argument("--panel-reflectance", default="1", metavar="R[,NIR]",
                    help="panel reflectance in red and NIR, e.g. 0.5 or 0.48,0.50 (default 1)")
    ap.add_argument("--roi", type=parse_box, action="append", default=[], metavar="X,Y,W,H",
                    help="area to report NDVI for (repeat for more areas)")
    ap.add_argument("--select", action="store_true", help="draw the panel and areas with the mouse")
    ap.add_argument("--ir-subtract", type=float, default=1.0, help="red = R - K * IR (default 1)")
    ap.add_argument("--black", type=float, default=64, help="black level of the raw data (default 64)")
    ap.add_argument("--min-signal", type=float, default=8,
                    help="mask blocks where red + NIR is below this many raw units (default 8)")
    ap.add_argument("--range", default="-0.2,0.9", metavar="LO,HI", help="colour scale range (default -0.2,0.9)")
    ap.add_argument("--live", action="store_true", help="live NDVI window instead of a still (needs a display)")
    args = ap.parse_args()

    refl = [float(v) for v in args.panel_reflectance.split(",")]
    args.panel_reflectance = (refl[0], refl[-1])
    args.range = tuple(float(v) for v in args.range.split(","))

    if args.live:
        if args.input:
            raise SystemExit("--live reads the camera; it can't be combined with --input")
        cap_frames = 1
        while True:
            rgbi = acquire(args, cap_frames)
            ndvi, *_ = compute(rgbi, args, args.panel)
            view = cv2.cvtColor(render(rgbi, ndvi, args, args.panel, args.roi), cv2.COLOR_RGB2BGR)
            cv2.imshow("VD16GZ NDVI (q to quit)", cv2.resize(view, None, fx=0.5, fy=0.5))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        return

    rgbi = acquire(args, args.frames)
    panel, rois = args.panel, args.roi
    if args.select:
        panel, rois = select_boxes(rgbi, args)
    ndvi, red, nir, info = compute(rgbi, args, panel)

    prefix = args.output or time.strftime("vd16gz_%Y%m%d_%H%M%S")
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    np.save(prefix + "_ndvi.npy", ndvi)
    cv2.imwrite(prefix + "_ndvi.png", cv2.cvtColor(render(rgbi, ndvi, args, panel, rois), cv2.COLOR_RGB2BGR))

    if panel is None:
        print("No reference panel: NDVI is UNCALIBRATED (raw signal ratio). Compare areas within this image only.")
    else:
        print(f"Panel {panel}: red {info['panel_red']:.1f}, NIR {info['panel_nir']:.1f} raw units, "
              f"reflectance red {args.panel_reflectance[0]}, NIR {args.panel_reflectance[1]}")
        print(stats_line("panel", ndvi[box_slices(panel)]))
    print(stats_line("image", ndvi))
    for n, box in enumerate(rois, 1):
        print(stats_line(f"area {n}", ndvi[box_slices(box)]) + f"   {box}")
    print(f"Saved {prefix}_ndvi.png and {prefix}_ndvi.npy ({ndvi.shape[1]} x {ndvi.shape[0]})")


if __name__ == "__main__":
    main()
