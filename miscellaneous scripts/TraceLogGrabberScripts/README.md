### To use these scripts, follow the instructions below:
- Copy the scripts to the root folder of a USB flash drive.
- Log on to the admin account on the RoboRIO by using some SSH terminal such as PuTTY (user: admin, password: \<blank\>)
# Manual Installation
- Copy the script robotautorunner.sh to the RoboRIO folder ``/home/lvuser``
- Make it executable by ``chmod +x /home/lvuser/robotautorunner.sh``
- On RoboRIO, edit the file in ``/etc/udev/rules.d/????`` and add a rule:
    ``ACTION=="add", ATTR(idVendor)=="0a81", ATTRS{idProduct}=="0101", RUN+="/home/lvuser/robotautorunner.sh"``
- OR copy ``99-usb-grabber.rules`` to ``/etc/udev/rules.d``
- Reboot the RoboRIO to make it in effect.
# Automatic Installation
- Copy the folder TraceLogGrabberScripts to the RoboRIO and make the installer executable by ``chmod 755 INSTALL.sh``.
- Install the grabber scripts with ``./INSTALL.sh`` 
- Reboot the RoboRIO to make it in effect.

