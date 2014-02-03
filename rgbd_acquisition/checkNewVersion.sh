#!/bin/bash

echo "Checking for a new version"
 

red=$(printf "\033[31m")
green=$(printf "\033[32m") 
normal=$(printf "\033[m")

SITE_TO_CHECK="http://cvrlcode.ics.forth.gr/web_share/RGBDAcquisitionROS/"

WHATVERSIONWEHAVE=`cat lastbuild.txt`

ISOURVERSIONCURRENT=`timeout -s KILL 30 wget -qO- $SITE_TO_CHECK | grep $WHATVERSIONWEHAVE`


if [ -z "$ISOURVERSIONCURRENT" ]
then 
 echo "" 
 echo "" 
 echo "" 
 echo $red "This build ( $WHATVERSIONWEHAVE ) is no longer the latest version!" $normal
 echo "Please visit $SITE_TO_CHECK to get the latest version"
 echo "" 
 echo "" 
 echo "" 
 sleep 2
else 
 echo $green "Our version seems up to date!" $normal
fi

 

exit 0
