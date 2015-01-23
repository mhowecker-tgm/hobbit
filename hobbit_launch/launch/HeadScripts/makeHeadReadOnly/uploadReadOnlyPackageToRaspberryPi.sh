 #!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

scp make_raspi_unionfs.tgz pi@192.168.2.199:/tmp

echo "Please do the following now :)"
echo "ssh pi@192.168.2.199 -c arcfour"
echo "after beeing connected to the pi"
echo "cd /tmp"
echo "tar xzf make_raspi_unionfs.tgz"
echo "cd make_raspi_unionfs"

echo "sudo ./make_raspi_unionfs.sh"
echo "reboot"
echo "try df -h"
echo "hope it works ;)"

exit 0
