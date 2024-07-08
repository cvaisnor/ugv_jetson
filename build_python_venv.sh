#!/bin/bash

echo "# Create a Python virtual environment."
# Create a Python virtual environment 
python -m venv ugv-env

echo "# Installing dependencies from requirements.txt"
sudo -H -u $USER bash -c 'source $PWD/ugv-env/bin/activate && pip install --upgrade setuptools pip && pip install -r requirements.txt'

echo "# Add current user to group so it can use serial."
sudo usermod -aG dialout $USER