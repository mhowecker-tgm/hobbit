#!/bin/bash

SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=10
MAX_RETRIES=10
 
i=1
ISRGBDUP=`rostopic list | grep /headcam/rgb/image_rect_color/compressed`
while [ -z "$ISRGBDUP" ]
do 
 echo "Not Started yet $i/$MAX_RETRIES ( $ISRGBDUP )"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
 #Do Startup Here

 #Do Startup Here
 
 ISRGBDUP=`rostopic list | grep /headcam/rgb/image_rect_color/compressed`
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting service"
  exit 1
 fi
done
 echo "Started succesfully"



exit 0
