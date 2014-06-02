#!/bin/bash

USBBUS=`lsusb | grep 1d27:0601 | cut  -d " " -f 2`
USBPORT=`lsusb | grep 1d27:0601 | cut  -d " " -f 4`


./usbreset /dev/bus/usb/${USBBUS[0]}/${USBPORT[0]}




exit 0
