#!/bin/bash

# author: Markus Bajones (bajo)
# email: bajones@acin.tuwien.ac.at

# This shell script will create the model and the recognizer structure
# needed for the obejct recognition service

# lower part of the object
/opt/ros/acin/faat_framework/objectmodelbuild/hobbit_object_modeller /opt/ros/acin/faat_framework/object_modeller/hobbit_config.txt \
    -reader.inputPath=/localhome/demo/data/${1}/down/ \
    -writer.outputPath=/opt/ros/hobbit_hydro/src/recognition_service/recognition_structure/${1}_down.pcd/ \
    -modelWriter.outputPath=/opt/ros/hobbit_hydro/src/recognition_service/models \
    -modelWriter.pattern=${1}_down.pcd \
    -reader.max_files=500

# upper part of the object
/opt/ros/acin/faat_framework/objectmodelbuild/hobbit_object_modeller /opt/ros/acin/faat_framework/object_modeller/hobbit_config.txt \
    -reader.inputPath=/localhome/demo/data/${1}/up/ \
    -writer.outputPath=/opt/ros/hobbit_hydro/src/recognition_service/recognition_structure/${1}_up.pcd/ \
    -modelWriter.outputPath=/opt/ros/hobbit_hydro/src/recognition_service/models \
    -modelWriter.pattern=${1}_up.pcd \
    -reader.max_files=500

