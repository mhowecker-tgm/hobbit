#!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8090 -e "echo \"Head is alive\""
 sleep 1
done

exit 0

