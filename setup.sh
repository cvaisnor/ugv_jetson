#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "This script must be run with sudo."
    echo "Use 'sudo ./setup.sh' instead of './setup.sh'"
    echo "Exiting..."
    exit 1
fi

# Install required software
echo "# Install required software."
sudo apt update
sudo apt upgrade -y
sudo apt install -y libopenblas-dev libatlas3-base libcamera-dev python3-opencv portaudio19-dev
sudo apt install -y util-linux procps hostapd iproute2 iw haveged dnsmasq iptables espeak

# disable serial login
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo udevadm trigger

# install jupyterlab
sudo apt install -y nodejs npm
sudo pip install jupyter jupyterlab
sudo pip3 install -U jetson-stats
sudo -H pip3 install -U jetson-stats

sudo apt install -y python3-venv python3-pip build-essential python3-dev