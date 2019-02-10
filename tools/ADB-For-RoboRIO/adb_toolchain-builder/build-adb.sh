#!/bin/bash

# Original script by bonnyfone. (https://github.com/bonnyfone)
# This script was adapted by Victor Du to build for the RoboRIO with a newer ADB version.
# The ADB builder is unlicensed. ADB is licensed under version 2 of the Apache license under AOSP. (Android Open Source Project)

# Branch to checkout from Android source code repo
branch=android-7.0.0_r5

# Makefile to use (will be automatically copied into system/core/adb)
makefile=makefile.sample


# DOWNLOAD necessary files
# -------------------------
echo "\n>> >>> ADB for RoboRIO <<< \n"
echo "\n>> Downloading necessay files ($branch branch)\n"
mkdir android-adb
cd android-adb
mkdir system
cd system
git clone -b $branch https://android.googlesource.com/platform/system/core
git clone -b $branch https://android.googlesource.com/platform/system/extras
cd ..
mkdir external
cd external
git clone -b $branch https://android.googlesource.com/platform/external/zlib
git clone -b $branch https://android.googlesource.com/platform/external/openssl
git clone -b $branch https://android.googlesource.com/platform/external/libselinux
cd ..


# MAKE
# -------------------------
echo "\n>> Copying makefile into system/core/adb...\n"
cp ../$makefile system/core/adb/makefile -f
cd system/core/adb/
echo "\n>> Make... \n"
make clean
make
echo "\n>> Copying adb back into current dir...\n"
cp adb ../../../../
echo "\n>> FINISH!\n"


