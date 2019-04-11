#!/bin/bash

DEVICEROOT=/media/sda1
SCRIPTNAME=robot_autorun.sh

while true
do
	echo Waiting for device...
	while [ ! -f $DEVICEROOT/$SCRIPTNAME ]
	do
		sleep 1
	done

	echo Found device
	cp $DEVICEROOT/$SCRIPTNAME /tmp
	chmod +x /tmp/$SCRIPTNAME
	su lvuser -c /tmp/$SCRIPTNAME

	echo Waiting for device to disappear...
	while [ -f $DEVICEROOT/$SCRIPTNAME ]
	do
		sleep 1
	done
done
