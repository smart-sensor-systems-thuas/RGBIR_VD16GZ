# RGBIR_VD16GZ

Use the ST **VD16GZ** RGB-IR global shutter sensor on a **Raspberry Pi 5** as two ordinary
V4L2 cameras: one colour (RGB) and one near-infrared (IR). Any program that can open a webcam
(OpenCV, GStreamer, ffmpeg, a browser) can read them, and both come from the same exposure.

| Device | Format | Content |
|---|---|---|
| `/dev/vd16gz-rgb` | RGB24, 1124 x 1364, 60 fps | colour image |
| `/dev/vd16gz-ir` | GREY, 1124 x 1364, 60 fps | near-infrared image |
| `/dev/vd16gz-raw` | Y16, 1124 x 1364, 60 fps | unprocessed 4x4 RGB-IR mosaic, 10-bit values |
| `/dev/vd16gz-sensor` | V4L2 sub-device | exposure, gain, frame rate, ... |

Everything needed is in this repository. You don't need to download ST's STSW-IMG504 package
separately, because the ST driver source is included in [`driver/`](driver).

## How it works

```
VD16GZ ──CSI-2──> rp1-cfe ──/dev/videoN (Y10P mosaic)──> vd16gz-bridge ──> /dev/vd16gz-rgb
 (vd56g3 driver)                                         (demosaic)      ├─> /dev/vd16gz-ir
                                                                         └─> /dev/vd16gz-raw
```

* **Kernel driver**: ST's `vd56g3` driver (VD56G3 / VD66GY / VD16GZ), built with DKMS so it is
  rebuilt automatically after kernel updates. The driver only has basic VD16GZ support. Linux has
  no format code for RGB-IR mosaics, so the driver reports the sensor as 10-bit greyscale.
* **vd16gz-bridge**: a small C service. It configures the Pi 5 camera pipeline, splits every
  sensor frame into RGB and IR, and writes the images to
  [v4l2loopback](https://github.com/umlaeute/v4l2loopback) devices. Pixels missing from the
  mosaic are interpolated by normalized convolution: G and IR with a 3x3 kernel, R and B with 5x5.
  Measured pixels are kept as they are.
* **Sensor controls**: exposure and gain are set on the sensor itself (`/dev/vd16gz-sensor`),
  also while streaming.

## Installation

Hardware: Raspberry Pi 5 with the VD16GZ camera board (ST adapter PCB4188) on the **CAM0**
or **CAM1** connector. Software: Raspberry Pi OS bookworm, 64-bit.

```bash
git clone https://github.com/smart-sensor-systems-thuas/RGBIR_VD16GZ.git
cd RGBIR_VD16GZ
sudo ./install.sh cam0      # or cam1, the connector the camera is on
sudo reboot
```

After the reboot, `ls /dev/vd16gz-*` shows the four devices. The installer:

* installs `dkms`, kernel headers, `v4l2loopback-dkms` and `v4l-utils`
* builds and installs the `vd56g3` driver with DKMS
* compiles the `pcb4188_vd16gz` overlay and adds `dtoverlay=pcb4188_vd16gz,cam0` to
  `/boot/firmware/config.txt`
* installs `vd16gz-bridge`, its systemd service, the v4l2loopback and udev settings, and the
  Python helper `vd16gz.py`

Remove everything again with `sudo ./uninstall.sh`.

## Using the camera

### OpenCV (Python)

```python
import cv2

rgb = cv2.VideoCapture("/dev/vd16gz-rgb", cv2.CAP_V4L2)
ir = cv2.VideoCapture("/dev/vd16gz-ir", cv2.CAP_V4L2)
while True:
    ok1, colour = rgb.read()   # BGR, as usual in OpenCV
    ok2, nir = ir.read()
    t1 = rgb.get(cv2.CAP_PROP_POS_MSEC)   # sensor timestamp of the frame
    t2 = ir.get(cv2.CAP_PROP_POS_MSEC)
    ...
```

Frames from the RGB and IR devices that belong together have **exactly the same timestamp**
(`CAP_PROP_POS_MSEC`, CLOCK_MONOTONIC). Use it to pair frames when your program can fall
behind. [`examples/opencv_view.py`](examples/opencv_view.py) shows both images live, with
keys for exposure.

Note: OpenCV converts the IR image to 3 identical channels. Use `nir[..., 0]`, or
`ir.set(cv2.CAP_PROP_CONVERT_RGB, 0)` to get a single channel.

### Exposure, gain and frame rate

From Python ([`tools/vd16gz.py`](tools/vd16gz.py), installed system-wide by `install.sh`):

```python
import vd16gz

cam = vd16gz.Sensor()
cam.set_exposure_us(5000)   # manual exposure, 5 ms
cam.set_gain(8)             # analogue gain step 0..28
cam.auto_exposure()         # the sensor's own auto exposure (default)
cam.set_fps(30)             # frame rate via vertical blanking
print(cam.get("exposure"), cam.temperature)
```

or from a shell:

```bash
v4l2-ctl -d /dev/vd16gz-sensor -L                                # list all controls
v4l2-ctl -d /dev/vd16gz-sensor -c auto_exposure=1,exposure=800   # manual, 800 lines
v4l2-ctl -d /dev/vd16gz-sensor -c analogue_gain=8
v4l2-ctl -d /dev/vd16gz-sensor -c auto_exposure=0                # back to auto
```

Exposure is set in lines of 7.69 us (1236 pixels at 160.8 MHz). At 60 fps it can be at most
2093 lines (16.1 ms). For longer exposures, lower the frame rate first.

### GStreamer, ffmpeg

```bash
# record 10 s of RGB and IR to video files
ffmpeg -f v4l2 -i /dev/vd16gz-rgb -t 10 -c:v libx264 -crf 18 rgb.mp4
ffmpeg -f v4l2 -i /dev/vd16gz-ir  -t 10 -c:v libx264 -crf 18 ir.mp4

# show the RGB stream
gst-launch-1.0 v4l2src device=/dev/vd16gz-rgb ! videoconvert ! autovideosink
```

### Live view in a browser

```bash
python3 examples/web_stream.py      # then open http://<pi-address>:8000
```

## Synchronization

RGB and IR are not two cameras that need syncing: they are made from **one exposure** of a
global shutter sensor. Each mosaic frame is split into one RGB image and one IR image, and both
get that frame's capture timestamp from the CSI-2 receiver. Pixel (x, y) in the RGB image and
pixel (x, y) in the IR image therefore show the same point at the same moment.

[`tools/verify_sync.py`](tools/verify_sync.py) checks this on a running system:

```
$ python3 tools/verify_sync.py
Test 1: timestamps (5 s)
  RGB frames 257, IR frames 257, frame period 16.666 ms (60.0 fps)
  RGB frames with an IR frame of identical timestamp: 257 (100.0 %)
  pairs offset by less than half a frame (would mean out of sync): 0
  PASS
Test 2: exposure steps (5 s, switching every 0.25 s)
  exposure 120 <-> 40 lines
  brightness steps: RGB 19, IR 19; seen by both streams: 19, at the same frame: 19
  PASS
RGB and IR are synchronized
```

Test 1 pairs frames by timestamp. Test 2 makes the brightness jump on one particular sensor
frame by switching the exposure, and checks that the jump appears in RGB and IR at the same
timestamp. That proves the image contents belong together, not only the timestamps.

## Configuration

Bridge options go in `/etc/default/vd16gz-bridge`. Apply them with
`sudo systemctl restart vd16gz-bridge`. See `vd16gz-bridge --help`:

| Option | Effect |
|---|---|
| `--half` | 560 x 682 output (2x2 binned demosaic), far less CPU |
| `--linear` | linear 8-bit output instead of sRGB gamma (for measurements) |
| `--wb auto\|none\|R,G,B` | white balance: grey world (default), none, or fixed gains |
| `--ccm FILE` | 3x4 colour matrix (default `/etc/vd16gz/ccm.json` if it exists) |
| `--ir-subtract K` | subtract K x IR from R, G and B (IR leakage) |
| `--raw none` | don't publish `/dev/vd16gz-raw` |
| `--exposure auto\|N`, `--gain N` | sensor settings at start-up |

**CPU load.** The bridge only processes a device while a program is streaming from it. When
nobody reads, it uses no CPU. Measured on a Raspberry Pi 5 at 60 fps:

| Reading | Full resolution | `--half` |
|---|---|---|
| nothing | 0 % | 0 % |
| IR only | ~1 core | |
| RGB + IR | ~2.9 cores (~14 ms per frame on 4 threads) | ~0.8 core |

If your own processing needs the CPU, use `--half`, or read only the device you need.

### Colour calibration (optional)

Without calibration, colours are white balanced with the grey world method. For accurate
colour, photograph a ColorChecker Classic under your lighting and fit a 3x4 matrix. The matrix
also uses the IR channel, so it removes IR leakage from the colours:

```bash
python3 tools/capture.py chart --average 8      # saves chart_rgb.png, chart_rgbi.npy, ...
# look up the pixel centres of the dark skin, bluish green, white and black patches in chart_rgb.png
python3 tools/calibrate.py chart_rgbi.npy --corners 210,330 820,330 210,740 820,740
sudo cp ccm.json /etc/vd16gz/ccm.json && sudo systemctl restart vd16gz-bridge
```

`tools/capture.py` also works on its own for linear data. It saves the raw mosaic and a linear
H x W x 4 (R, G, B, IR) float array for analysis.

## Limitations

* **One program per device at a time.** v4l2loopback 0.13 allows a single streaming reader
  per device. Different programs can read RGB and IR at the same time.
* **Full resolution only** (1124 x 1364). Cropping or binning on the sensor would shift the
  colour filter pattern.
* **Don't use the sensor's `horizontal_flip` / `vertical_flip` controls.** They change the
  colour filter layout, which the bridge doesn't follow. Flip the images in your own code
  (`cv2.flip`) instead.
* The sensor itself can run at up to about 88 fps (`set_fps`), but full-resolution
  processing reaches about 70 fps. Use `--half` for higher frame rates.
* While the service runs, it owns the camera. Programs that open the CSI-2 receiver directly
  (`/dev/video0`, `rpicam-*`) won't work until you `sudo systemctl stop vd16gz-bridge`.

## Troubleshooting

```bash
systemctl status vd16gz-bridge            # is the bridge running?
journalctl -u vd16gz-bridge -f            # its log, including which devices are in use
dkms status                               # is vd56g3 built for the running kernel?
lsmod | grep vd56g3                       # is the driver loaded?
grep pcb4188 /boot/firmware/config.txt    # is the overlay enabled?
dmesg | grep -i vd56g3                    # did the driver find the sensor?
```

* `VD16GZ not found`: check the ribbon cable and the camera connector given to `install.sh`.
* No `/dev/vd16gz-rgb`: v4l2loopback was already loaded with other settings. Reboot.

## Repository layout

```
driver/      ST vd56g3 kernel driver v1.2.1 (from STSW-IMG504), dkms.conf, device tree overlay
bridge/      vd16gz-bridge (C), mosaic -> RGB / IR / raw V4L2 devices
systemd/     service, v4l2loopback options, udev rules
tools/       vd16gz.py (sensor controls), verify_sync.py, capture.py, calibrate.py
examples/    opencv_view.py, web_stream.py
```

## Licences

* `driver/`: STMicroelectronics, **GPL-2.0** (see [`driver/LICENSE`](driver/LICENSE)). ST
  distributes the package under licence agreement [SLA0103](driver/SLA0103.pdf), which allows
  redistribution as long as ST's copyright notices are kept. The files are unchanged from ST's
  release; only `dkms.conf` was added.
* Everything else: **MIT** (see [`LICENSE`](LICENSE)).
