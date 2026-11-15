#!/usr/bin/env bash
# One-time setup on the Raspberry Pi 5 (Raspberry Pi OS Bookworm). Run from the pi/ folder:
#   bash setup_pi.sh
set -euo pipefail
CONFIG=/boot/firmware/config.txt

sudo apt-get update
sudo apt-get install -y python3-venv python3-picamera2 python3-libcamera i2c-tools

# venv that can still see the apt-installed picamera2
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt

# I2C bus 1 for the left eye, software bus 3 (GPIO 23 SDA, 24 SCL) for the right eye,
# and full current on the USB ports: the XY-3606 cannot negotiate USB power delivery.
add_line() { grep -qxF "$1" "$CONFIG" || echo "$1" | sudo tee -a "$CONFIG" > /dev/null; }
add_line "dtparam=i2c_arm=on"
add_line "dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24"
add_line "usb_max_current_enable=1"
sudo usermod -aG dialout,i2c,video "$USER"

echo "Done. Reboot, then check the eyes with:  i2cdetect -y 1  and  i2cdetect -y 3  (0x3c on each)."
echo "Start at boot:  sudo cp humanoid.service /etc/systemd/system/ && sudo systemctl enable --now humanoid"
