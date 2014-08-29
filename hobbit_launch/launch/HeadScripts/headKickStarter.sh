#!/bin/bash

source /home/pi/.bashrc

./headSyncTime.sh&

while true; do
 nc6 -l -p 8080 -e "./headStart.sh"
 sleep 1
done

exit 0

