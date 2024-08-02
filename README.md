![GitHub top language](https://img.shields.io/github/languages/top/waveshareteam/ugv_jetson) ![GitHub language count](https://img.shields.io/github/languages/count/waveshareteam/ugv_jetson)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/waveshareteam/ugv_jetson)
![GitHub repo size](https://img.shields.io/github/repo-size/waveshareteam/ugv_jetson) ![GitHub](https://img.shields.io/github/license/waveshareteam/ugv_jetson) ![GitHub last commit](https://img.shields.io/github/last-commit/waveshareteam/ugv_jetson)

# Waveshare UGV Robots
![](./media/UGV-Rover-details-23.jpg)

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the upper computer, which is typically a Jetson Orin Nano or a Jetson Orin NX in this setup.  

The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used.  

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a Jetson Orin Nano or a Jetson Orin NX, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

After powering on the robot, Jetson will automatically establish a hotspot, and the OLED screen will display a series of system initialization messages:  

![](./media/RaspRover-LED-screen.png)
- The first line `E` displays the IP address of the Ethernet port, which allows remote access to the Raspberry Pi. If it shows No Ethernet, it indicates that the Raspberry Pi is not connected to an Ethernet cable.
- The second line `W` indicates the robot's wireless mode. In Access Point (AP) mode, the robot automatically sets up a hotspot with the default IP address `192.168.50.5`. In Station (STA) mode, the Raspberry Pi connects to a known WiFi network and displays the IP address for remote access.
- The third line `F/J` specifies the Ethernet port numbers. Port `5000` provides access to the robot control Web UI, while port `8888` grants access to the JupyterLab interface.
- The fourth line `STA` indicates that the WiFi is in Station (STA) mode. The time value represents the duration of robot usage. The dBm value indicates the signal strength RSSI in STA mode.  


You can access the robot web app using a mobile phone or PC. Simply open your browser and enter `[IP]:5000` (for example, `192.168.10.50:5000`) in the URL bar to control the robot.  

## Install

After cloning the repository, run the following command to install the required packages:
```
sudo ./setup.sh
```

Next, build the python virtual environment, activate it, and install dependencies:
```
python -m venv ugv-env

source ugv-env/bin/activate

pip install -r requirements.txt
```

Next, check the NumPy version:
```
pip list | grep numpy
```
It will likely display 2.0.0, we need to downgrade to 1.24.6 due to compatibility issues.
```
pip uninstall numpy

python -m pip install 'numpy==1.24.6'
```

Next, add user to the dialout group to allow access to the serial port:
```
sudo usermod -a -G dialout $USER
```

Reboot the system to apply the changes:
```
sudo reboot
```