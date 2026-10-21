#!/bin/bash
# Install the VD16GZ RGB-IR camera on a Raspberry Pi 5 (Raspberry Pi OS bookworm, 64-bit):
#  - vd56g3 kernel driver (ST, GPL-2.0) through DKMS, so it is rebuilt after kernel updates
#  - device tree overlay pcb4188_vd16gz and the dtoverlay line in config.txt
#  - v4l2loopback devices + vd16gz-bridge service: /dev/vd16gz-rgb, /dev/vd16gz-ir, /dev/vd16gz-raw
#
# Usage: sudo ./install.sh [cam0|cam1]     (camera connector, default cam0)
set -euo pipefail

PORT=${1:-cam0}
case "$PORT" in cam0 | cam1) ;; *) echo "usage: sudo $0 [cam0|cam1]"; exit 2 ;; esac
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0 $*"; exit 1; }

HERE=$(cd "$(dirname "$0")" && pwd)
DRV_NAME=vd56g3
DRV_VERSION=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$HERE/driver/dkms.conf")
KVER=$(uname -r)
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
OVERLAY=pcb4188_vd16gz

step() { echo; echo "==> $*"; }

step "Installing packages"
apt-get update -q
apt-get install -y -q dkms device-tree-compiler build-essential v4l-utils v4l2loopback-dkms \
	python3-numpy python3-opencv python3-pil \
	"linux-headers-$KVER" || apt-get install -y -q dkms device-tree-compiler build-essential \
	v4l-utils v4l2loopback-dkms python3-numpy python3-opencv python3-pil linux-headers-rpi-2712

step "Kernel driver $DRV_NAME $DRV_VERSION (DKMS)"
# Older manual installs put vd56g3.ko straight into /lib/modules/<version>/; that copy
# would not follow kernel updates, so move it out of the way.
for f in /lib/modules/*/vd56g3.ko; do
	[ -e "$f" ] || continue
	echo "moving manually installed $f to $f.manual-backup"
	mv "$f" "$f.manual-backup"
done
if dkms status "$DRV_NAME/$DRV_VERSION" | grep -q .; then
	dkms remove "$DRV_NAME/$DRV_VERSION" --all
fi
rm -rf "/usr/src/$DRV_NAME-$DRV_VERSION"
mkdir -p "/usr/src/$DRV_NAME-$DRV_VERSION"
cp -r "$HERE/driver/src" "$HERE/driver/dkms.conf" "/usr/src/$DRV_NAME-$DRV_VERSION/"
dkms add "$DRV_NAME/$DRV_VERSION"
dkms build "$DRV_NAME/$DRV_VERSION" -k "$KVER"
dkms install "$DRV_NAME/$DRV_VERSION" -k "$KVER" --force
depmod -a "$KVER"

step "Device tree overlay $OVERLAY ($PORT)"
dtc -q -I dts -O dtb -o "/boot/firmware/overlays/$OVERLAY.dtbo" "$HERE/driver/dts/$OVERLAY.dts"
REBOOT=0
if grep -qE "^\s*dtoverlay=$OVERLAY,$PORT\s*$" "$CONFIG"; then
	echo "$CONFIG already has dtoverlay=$OVERLAY,$PORT"
else
	if grep -qE "^\s*dtoverlay=$OVERLAY" "$CONFIG"; then
		sed -i -E "s/^(\s*dtoverlay=$OVERLAY.*)$/# \1  (replaced by vd16gz install.sh)/" "$CONFIG"
	fi
	printf '\n[all]\ndtoverlay=%s,%s\n' "$OVERLAY" "$PORT" >>"$CONFIG"
	echo "added dtoverlay=$OVERLAY,$PORT to $CONFIG"
	REBOOT=1
fi

step "Bridge, loopback devices and service"
make -C "$HERE/bridge" clean >/dev/null
make -C "$HERE/bridge"
make -C "$HERE/bridge" install
install -D -m 644 "$HERE/systemd/vd16gz-loopback.conf" /etc/modprobe.d/vd16gz-loopback.conf
echo v4l2loopback >/etc/modules-load.d/vd16gz.conf
install -D -m 644 "$HERE/systemd/99-vd16gz.rules" /etc/udev/rules.d/99-vd16gz.rules
install -D -m 644 "$HERE/systemd/vd16gz-bridge.service" /etc/systemd/system/vd16gz-bridge.service
[ -f /etc/default/vd16gz-bridge ] || install -D -m 644 "$HERE/systemd/vd16gz-bridge.default" /etc/default/vd16gz-bridge
mkdir -p /etc/vd16gz
PYDIR=$(python3 -c 'import sys; print(f"/usr/local/lib/python3.{sys.version_info.minor}/dist-packages")')
install -D -m 644 "$HERE/tools/vd16gz.py" "$PYDIR/vd16gz.py"

udevadm control --reload
if ! lsmod | grep -q '^v4l2loopback '; then
	modprobe v4l2loopback
elif [ "$(cat /sys/module/v4l2loopback/parameters/max_buffers 2>/dev/null)" != 4 ] ||
	! grep -qx "VD16GZ RGB" /sys/class/video4linux/video*/name 2>/dev/null; then
	echo "v4l2loopback is already loaded with other settings; the VD16GZ devices appear after a reboot"
	REBOOT=1
fi
udevadm trigger --subsystem-match=video4linux
systemctl daemon-reload
systemctl enable vd16gz-bridge.service

if [ "$REBOOT" = 1 ] || ! lsmod | grep -q "^$DRV_NAME "; then
	step "Done. Reboot to start the camera: sudo reboot"
else
	systemctl restart vd16gz-bridge.service
	sleep 2
	systemctl --no-pager --lines=5 status vd16gz-bridge.service || true
	step "Done. Devices: $(ls /dev/vd16gz-* 2>/dev/null | tr '\n' ' ')"
fi
