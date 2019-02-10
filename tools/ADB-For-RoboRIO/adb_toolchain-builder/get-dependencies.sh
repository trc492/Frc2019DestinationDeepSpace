#!/bin/bash
if [[ $EUID -ne 0 ]]; then
	echo "Please run this script with administrator privileges. Sorry~ X///3"
	exit 1
fi
if hash apt-get >/dev/null; then
        echo "Adding FRC Toolchain repositories..."
	add-apt-repository ppa:wpilib/toolchain
	apt-get update
	echo "Done."
	echo "Installing FRC Toolchain..."
	apt-get install frc-toolchain frcmake binutils-frc-armel-cross
	apt-get install zlib1g-dev libssl-dev git make
	echo "Done."
	echo "Please run build.sh to build ADB for your RoboRIO."
else
        echo "APT is not installed. Please install APT on your computer."
	exit 1
fi
