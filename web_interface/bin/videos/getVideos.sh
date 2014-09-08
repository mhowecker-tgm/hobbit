#!/bin/bash

STARTDIR=`pwd` 
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

./getGestureVideos.sh&

echo "getVideos.sh done"

exit 0
