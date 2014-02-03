#!/bin/bash

# https://github.com/AmmarkoV/RGBDAcquisition <- this is the library that provides
# unified access  to OpenNI2 etc , to get it just execute 
# git clone git://github.com/AmmarkoV/RGBDAcquisition
# After you successfully compile it , please update the DATASETS path that follows 
DATASETS="/home/ammar/Documents/Programming/RGBDAcquisition"

#Switch to this directory
STARTDIR=`pwd` 
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
#-----------------------------------------------------------

cd include  
ln -s $DATASETS/acquisition/Acquisition.h
cd ..



cd bin
#Copy things 
ln -s $DATASETS/acquisition/libAcquisition.so
ln -s $DATASETS/tools/Calibration/libCalibrationLibrary.a
ln -s $DATASETS/tools/OperatingSystem/libOperatingSystem.a
ln -s $DATASETS/openni2_acquisition_shared_library/libOpenNI2Acquisition.so
ln -s $DATASETS/template_acquisition_shared_library/libTemplateAcquisition.so
ln -s $DATASETS/v4l2_acquisition_shared_library/libV4L2Acquisition.so
ln -s $DATASETS/v4l2stereo_acquisition_shared_library/libV4L2StereoAcquisition.so
ln -s $DATASETS/libfreenect_acquisition_shared_library/libFreenectAcquisition.so


ln -s $DATASETS/editor/Editor
ln -s $DATASETS/viewer/Viewer

v4l2_acquisition_shared_library




#-----------------------------------------------------------
cd "$STARTDIR"
exit 0
