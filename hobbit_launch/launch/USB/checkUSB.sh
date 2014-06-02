#!/bin/bash

if [ -e usbreset ]
then
  echo "OK , usb reset tool is there" 
else
   gcc usbReset.c -s -O3 -o usbreset 
fi


exit 0
