#!/bin/bash

#Quick Settings
HOBBITDIR="/opt/ros/hobbit_hydro/"
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=10
MAX_RETRIES=10
 
 
#Ammar's installation at FORTH may override hobbit dir
if [ -d $HOBBITDIR ] 
then
 echo "Hobbit Normal root dir"
elif [ -d ~/Documents/Programming/FORTH/Hobbit/hobbit_hydro ] 
then
 HOBBITDIR="~/Documents/Programming/FORTH/Hobbit/hobbit_hydro/"
fi


#Try to bring up our node for the first time!
echo "Trying to bring BaseCam Node up with a first try"
screen -d -m -S "basecam" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:=\"#2\" &>> ~/debugBaseCam.txt "
sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES


#We already tried once !
i=2
ISRGBDUP=`rostopic list | grep /basecam/rgb/image_rect_color/compressed`
while [ -z "$ISRGBDUP" ]
do 
 screen -S "basecam" -X quit
 echo "BaseCam node not started yet $i/$MAX_RETRIES ( $ISRGBDUP )"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
 #Do Startup Here
  screen -d -m -S "basecam" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:=\"#2\" &>> ~/debugBaseCam.txt"
 #Do Startup Here
 
 ISRGBDUP=`rostopic list | grep /basecam/rgb/image_rect_color/compressed`
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting service , todo try to restart a device or something here"
  exit 1
 fi
done

echo "Base cam - Started succesfully"

exit 0
