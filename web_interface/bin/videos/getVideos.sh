#!/bin/bash

STARTDIR=`pwd` 
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

VideosToCheck="PT2-Gestures-Bad_practice_sideview.mp4 PT2-Gestures-Good_practice_sideview.mp4 PT2-Gestures-Bad_practice_robotview.mp4 PT2-Gestures-Good_practice_robotview.mp4 Hobbit_gestures_Elderly2014b.mp4  "

for i in $VideosToCheck;
do
 if [ -e $i ]
  then
     echo "Video $i already exists"
   else
     echo "wget $i"
     wget "http://cvrlcode.ics.forth.gr/web_share/Hobbit/GestureVideos/$i"
 fi
done 


exit 0
