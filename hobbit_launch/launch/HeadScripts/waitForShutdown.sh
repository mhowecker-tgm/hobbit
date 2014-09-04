#!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8082 -e "echo \"Shutting down\" && systemCommandsPi shutdown"
 sleep 1
done

exit 0

