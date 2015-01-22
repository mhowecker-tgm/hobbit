 #!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8092 -e "echo \"Making RPI readWrite again..\" && ./runSystemCommandPi.sh rw && echo \"kthxbye..\""
 echo "Served one client"
 #sleep 1
done

exit 0

