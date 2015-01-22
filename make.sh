#!/bin/bash

#Easy way to compile as release without needing to remember the flags etc..
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release $@


exit 0
