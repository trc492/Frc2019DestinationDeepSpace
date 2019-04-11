#!/bin/sh
# This is a quick installer for TraceLogGrabberScripts.
# Deliver this through a USB drive with the rest of TraceLogGrabberScripts in the same folder, and run this on the RIO.
echo "Installing shell scripts..."
cp robot_autorun.sh /home/lvuser/
cp robotautorunner.sh /home/lvuser/
cp 99-usb-grabber.rules /etc/udev/rules.d/
echo "Done."
echo "Making shell scripts executable..."
chmod 755 /home/lvuser/robot_autorun.sh
chmod 755 /home/lvuser/robotautorunner.sh
echo "Done."
echo "Installation successful."
