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

