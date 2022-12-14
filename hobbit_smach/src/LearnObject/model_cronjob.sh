#!/bin/bash

# author: Markus Bajones (bajo)
# email: bajones@acin.tuwien.ac.at

# This shell script will create the model and the recognizer structure
# needed for the obejct recognition service

rm /localhome/demo/.ros/sift_flann.idx
rm /opt/ros/hobbit_hydro/src/recognition_service/sift_flann.idx

cd /localhome/demo/data
mkdir -p /localhome/demo/finished_data/
for i in *; do
    /opt/ros/hobbit_hydro/src/hobbit_smach/src/LearnObject/build_model.sh $i
    mv $i /localhome/demo/finished_data/
done
