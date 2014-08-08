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

sh -c 'echo "#!/bin/bash \n \
             DIR=\"\$( cd \"\$( dirname \"\${BASH_SOURCE[0]}\" )\" && pwd )\" \n  \
             cd \$DIR  \n \
             cd ..        \n \
             cd ..        \n \
             cd ..        \n \
             svn update . \n \
             cd ..        \n \
             catkin_make  \n \
             exit 0 \n" > System/update.sh' 

sudo chown "$OURUSER":"$OURUSER" System/update.sh
sudo chmod +x System/update.sh



#Multi System Tool
sudo rm System/systemCommands
gcc System/systemCommands.c -o System/systemCommands
sudo chown root:root System/systemCommands
sudo chmod +s System/systemCommands



exit 0
