#!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8084 -e "echo \"Refreshing..\" && ./runSystemCommandPi.sh refresh && echo \"kthxbye..\""
 sleep 1
done

exit 0

