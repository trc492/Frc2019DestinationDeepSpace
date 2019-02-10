# ADB-RoboRIO-Installer
A port of ADB for the NI RoboRIO. Intended for use in First Robotics Challenge.

### Installing ADB on your RoboRIO
1. Clone this repository onto a USB flash drive.
2. Plug the flash drive into the RoboRIO.
3. SSH into the RoboRIO using the SSH client of your choice as user ```admin@roboRIO-{your team number}-frc.local```. (i.e. PuTTY)
4. cd into the directory of the installer.
5. Run the following commands:
```
chmod 755 install.sh 
./install.sh
```

### Building ADB for the RoboRIO from sources.
1. Clone this repository onto your computer.
2. cd into the repository and enter the ```adb_toolchain-builder``` directory.
3. Run the following commands:
```
sudo su
chmod 755 *.sh
./get-dependencies.sh
./build-adb.sh
```
After adb is built, you can replace the existing ADB executable in ```install_files/usr/lib/android-sdk/platform-tools/``` with the compiled executable (which should be in your current ```adb_toolchain-builder``` directory as ```adb```.)
