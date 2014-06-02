#!/bin/bash

USBBUS=`lsusb | grep 1d27:0601 | cut  -d " " -f 2`
USBPORT=`lsusb | grep 1d27:0601 | cut  -d " " -f 4`


echo suspend >/sys/bus/usb/devices/1-2/power/level

echo auto >/sys/bus/usb/devices/1-2/power/level

./usbreset /dev/bus/usb/${USBBUS[0]}/${USBPORT[0]}






exit 0
