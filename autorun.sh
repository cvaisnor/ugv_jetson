#!/bin/bash

if [ -n "$SUDO_USER" ] || [ -n "$SUDO_UID" ]; then
    echo "This script was executed with sudo."
    echo "Use './autorun.sh' instead of 'sudo ./autorun.sh'"
    echo "Exiting..."
    exit 1
fi

# Define the first cron job and its schedule
cron_job1="@reboot sleep 1 && whoami && pulseaudio --start && sleep 1 && XDG_RUNTIME_DIR=/run/user/$(id -u) ~/ugv_jetson/ugv-env/bin/python ~/ugv_jetson/app.py >> ~/ugv.log 2>&1"

# Check if the first cron job already exists in the user's crontab
if crontab -l | grep -q "$cron_job1"; then
    echo "First cron job is already set, no changes made."
else
    # Add the first cron job for the user
    (crontab -l 2>/dev/null; echo "$cron_job1") | crontab -
    echo "First cron job added successfully."
fi