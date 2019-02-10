#!/bin/bash
# This script is licensed under the Apache License version 2.0. You may freely run it, redistribute it or modify it.
# Please run this on a RoboRIO or other ARMv7-based device to install ADB.
#
# Author: Victor Du (DrDab)
#
if [[ $EUID -ne 0 ]]; then
   echo "[ADBRio] Please run this script with administrator privileges. Sorry~ X///3"
   exit 1
fi

if [[ $(uname -a) != *"armv7"* ]];then
    echo "[ADBRio] Please run this script on a RoboRIO with an armv7 architecture. Sorry~ X///3"
    exit 1
fi

echo "      [ ADBRio Quick Installer v1.0 ] "
echo "Would you like to install ADB on your RoboRIO? (Y/N)"
old_stty_cfg=$(stty -g)
stty raw -echo ; answer=$(head -c 1) ; stty $old_stty_cfg
if echo "$answer" | grep -iq "^y" ;then
   echo "[ADBRio] Installing ADB for you... >w<"
   cd install_files
   echo "[ADBRio] Copying files..."
   cp * / -r
   echo "[ADBRio] Done."
   echo "[ADBRio] Setting attributes and permissions..."
   chown root:root /etc/init.d/adb.sh
   chmod 777 /etc/init.d/adb.sh
   update-rc.d /etc/init.d/adb.sh defaults
   chmod 777 /usr/bin/adb
   chmod 777 /usr/lib/android-sdk/platform-tools/adb
   echo "[ADBRio] Done. I'm almost there! X3"
   echo "[ADBRio] Registering dynamic libraries..."
   ldconfig
   echo "[ADBRio] Done."
   echo
   echo "[ADBRio] Finished installing ADB on your RoboRIO. Yay!~ <:"
else
   echo "[ADBRio] ADB installation cancelled. T_T"
fi
