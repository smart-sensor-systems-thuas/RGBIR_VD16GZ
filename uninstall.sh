#!/bin/bash
# Remove everything install.sh added. The v4l2loopback package and the build tools stay installed.
set -uo pipefail
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0"; exit 1; }

HERE=$(cd "$(dirname "$0")" && pwd)
DRV_VERSION=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$HERE/driver/dkms.conf")
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt

systemctl disable --now vd16gz-bridge.service 2>/dev/null
rm -f /etc/systemd/system/vd16gz-bridge.service /usr/local/bin/vd16gz-bridge
rm -f /etc/modprobe.d/vd16gz-loopback.conf /etc/modules-load.d/vd16gz.conf /etc/udev/rules.d/99-vd16gz.rules
rm -f /usr/local/lib/python3.*/dist-packages/vd16gz.py
systemctl daemon-reload
udevadm control --reload

dkms remove "vd56g3/$DRV_VERSION" --all 2>/dev/null
rm -rf "/usr/src/vd56g3-$DRV_VERSION"
rm -f /boot/firmware/overlays/pcb4188_vd16gz.dtbo
sed -i -E '/^\s*dtoverlay=pcb4188_vd16gz/d' "$CONFIG"

echo "Removed. /etc/default/vd16gz-bridge and /etc/vd16gz/ were kept. Reboot to finish."
