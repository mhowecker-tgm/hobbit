#!/bin/bash

if [ -d NiTE2 ]
then 
  echo "You seem to already have Nite , not downloading it"
  exit 0
fi 

echo "This seems to be the first time you are running rgbd_acquisition node"
echo "We will now try to download the Nite2 dependencies (100MB) from an external server"
echo "Put there in order to reduce the size of this repository"

echo "In case that the download does not work please contact ammarkov@ics.forth.gr , papoutsa@ics.forth.gr"
wget "http://cvrlcode.ics.forth.gr/web_share/Hobbit/rgbd_acquisition_data.tar.gz" -O data.tar.gz
tar xvzf data.tar.gz

exit 0
