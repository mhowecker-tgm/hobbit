#!/bin/bash

red=$(printf "\033[31m")
normal=$(printf "\033[m")


if [ $( id -u ) -eq 0 ]; then
echo "Will setup things that need super user powers now"
else
 echo $red
 echo "Initializer must be run as root in order to have system permissions.."
 echo "Please re run using sudo ./initializeHobbit.sh , exiting now.."
 echo $normal
 exit 0
fi


exit 0
