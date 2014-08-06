#!/bin/bash

red=$(printf "\033[31m")
normal=$(printf "\033[m")

#Check Super User
if [ $( id -u ) -eq 0 ]; then
 echo $red 
 echo "Please run this script as a regular user , it will ask you for super user powers when needed "
 echo $normal
 exit 0
else
 echo "Working.."
fi

sudo echo "Asked for super user"

#Change to Script directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

#Link hobbit to startup
./activateHobbitAutomaticStartup.sh

#Generate Scripts
OURUSER=`whoami`

mkdir System
sh -c 'echo "#!/bin/bash \n sudo init 0\n exit 0\n" > System/shutdown.sh' 
sudo chown root:root System/shutdown.sh
sudo chmod 777 System/shutdown.sh
sudo chmod +s System/shutdown.sh

sh -c 'echo "#!/bin/bash \n sudo init 6\n exit 0\n" > System/restart.sh' 
sudo chown root:root System/restart.sh
sudo chmod 777 System/restart.sh
sudo chmod +s System/restart.sh

sh -c 'echo "#!/bin/bash \n \
             DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" && pwd )\" \n  \
             cd \$DIR  \n \
             cd ..        \n \
             cd ..        \n \
             cd ..        \n \
             svn update . \n exit 0 \n" > System/update.sh' 

sudo chown "$OURUSER":"$OURUSER" System/update.sh
#sudo chmod 777 System/update.sh
#sudo chmod +s System/update.sh



sh -c 'echo "#!/bin/bash \n \
             sudo apt-get update && sudo apt-get upgrade -y \n exit 0 \n" > System/updateGlobal.sh'

sudo chown root:root System/updateGlobal.sh
sudo chmod 777 System/updateGlobal.sh
sudo chmod +s System/updateGlobal.sh


exit 0
