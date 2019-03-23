#!/bin/bash

set -e

echo "Robot script running"

mkdir -p /home/lvuser/tracelogbackup
cp /home/lvuser/tracelog/* /media/sda1/TraceLog
mv /home/lvuser/tracelog/* /home/lvuser/tracelogbackup

echo "Robot script done"
